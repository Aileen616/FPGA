#include "jdy31_motor.h"
#include "perf_monitor.h"
#include "mecanum_manual.h"
/* 来源：esp_diag_wb与motor_forward_wb板级外设；BX72无STM32 HAL，仅此文件封装MMIO。 */
#define JDY_BASE UINT32_C(0x20002000)
#define MOTOR_BASE UINT32_C(0x20003000)
static jdy_motor_t motor; /* 静态协议状态；物理位与算法轮序的换算仅在drive内进行。 */
static char tx[512]; /* 有界整包发送队列；避免通信拥塞继续运动。 */
static unsigned head,tail;

/* 功能：读板级MMIO；base为两外设基址，offset为合法4字节对齐偏移；返回32位值。 */
static uint32_t rd(uint32_t base,unsigned offset)
{uint32_t v=*(volatile uint32_t *)(uintptr_t)(base+offset);__asm__ volatile("":::"memory");return v;}
/* 功能：写板级MMIO；base为两外设基址，offset为合法偏移，v为寄存器值；无返回。 */
static void wr(uint32_t base,unsigned offset,uint32_t v)
{__asm__ volatile("":::"memory");*(volatile uint32_t *)(uintptr_t)(base+offset)=v;}
/* 当前官方PicoRV32无数据缓存、总线逐笔完成，volatile配合编译器屏障保持MMIO次序；
   不依赖旧核未承诺支持的FENCE指令。迁移至有缓存或多笔并发访问的CPU时须重审。 */
/* 功能：整包入队；ctx不用可空，data不可空，n为字节数；容量足够返回true。 */
static bool enqueue(void *ctx,const char *data,size_t n)
{
    (void)ctx;if(n>(tail+sizeof(tx)-head-1U)%sizeof(tx)){perf_monitor_tx_drop();return false;}
    for(size_t i=0;i<n;i++){tx[head]=data[i];head=(head+1U)%sizeof(tx);}perf_monitor_queue((head+sizeof(tx)-tail)%sizeof(tx));return true;
}
/* 功能：提交保护事件；ctx不用可空，event为JDY_M枚举，seq保留；无返回。 */
static void guard(void *ctx,unsigned event,uint32_t seq)
{
    (void)ctx;(void)seq;
    if(event==JDY_M_ARM){wr(JDY_BASE,0x0c,2);return;}
    /* 停车必须先撤销动力，再改变通信状态；清故障本身不得启动任何轮子。 */
    wr(MOTOR_BASE,0x08,4);wr(JDY_BASE,0x0c,4);
    if(event==JDY_M_CLEAR){wr(MOTOR_BASE,0x08,1);wr(JDY_BASE,0x0c,1);}
}
/* 功能：合并两外设保护状态；ctx不用可空；返回bit0联合有效、bit1故障、bit2通信有效。 */
static uint32_t state(void *ctx)
{
    (void)ctx;uint32_t communication=rd(JDY_BASE,8),drive=rd(MOTOR_BASE,4);
    return ((communication&drive)&1U)|((communication|drive)&2U)|((communication&1U)<<2);
}
/* 功能：停机设置驱动配置；ctx不用可空，mask为1..15、invert为0..15；回读相符返回true。 */
static bool configure(void *ctx,uint32_t mask,uint32_t invert)
{
    (void)ctx;
    if(mask<1||mask>15||invert>15||(rd(JDY_BASE,8)&1U)||(rd(MOTOR_BASE,4)&1U))return false;
    uint32_t config=mask|(invert<<4);
    wr(MOTOR_BASE,0x0c,config);
    return rd(MOTOR_BASE,0x0c)==config&&(rd(MOTOR_BASE,4)&4U)!=0;
}
/* 功能：提交四轮有符号驱动力并续租；ctx可空，seq非零递增，key为1..10，level为1..3；实际接收返回true。 */
static bool drive(void *ctx,uint32_t seq,uint32_t key,uint32_t level)
{
    (void)ctx;
    mecanum_output_t output;
    /* 用户实测通道1右后、2右前、3左后、4左前；物理位顺序仍为3/2/4/1。 */
    static const uint8_t physical_to_wheel[4]={MECANUM_RL,MECANUM_FR,MECANUM_FL,MECANUM_RR};
    if(!seq||(rd(JDY_BASE,8)&3U)!=1U||!mecanum_manual_mix(key,level,&output))return false;
    for(unsigned i=0;i<4;i++)wr(MOTOR_BASE,0x50+i*4,(uint32_t)(int32_t)output.effort[physical_to_wheel[i]]);
    wr(MOTOR_BASE,0x10,seq);wr(MOTOR_BASE,0x14,UINT32_C(0x4d4f5633));
    if((rd(MOTOR_BASE,4)&3U)!=1U||rd(MOTOR_BASE,0x44)!=seq)return false;
    /* 仅成功的运动提交能够刷新通信保护；不存在普通hb刷新动力的旁路。 */
    wr(JDY_BASE,0x10,seq);
    return (state(0)&3U)==1U;
}
/* 功能：运行裸机完整低输出遥控；无参数，正常不返回；上电不配置、不启动、不自动恢复。 */
int main(void)
{
    const jdy_motor_io_t io={0,enqueue,guard,state,configure,drive};
    if(rd(JDY_BASE,0)!=UINT32_C(0x45535001)||rd(MOTOR_BASE,0)!=UINT32_C(0x4d465703))for(;;){}
    jdy_motor_init(&motor,&io,rd(JDY_BASE,4));
    perf_monitor_init(enqueue);
    for(;;){
        uint32_t now=rd(JDY_BASE,4);
        if(rd(JDY_BASE,0x1c))jdy_motor_transport_fault(&motor);
        jdy_motor_poll(&motor,now);
        if(perf_monitor_pending)perf_monitor_service(now);
        uint32_t c=rd(JDY_BASE,0x14);
        if(!(c&0x100U))jdy_motor_rx(&motor,(uint8_t)c,now);
        if(tail!=head&&!(rd(JDY_BASE,0x18)&0x100U)){
            wr(JDY_BASE,0x18,(uint8_t)tx[tail]);tail=(tail+1U)%sizeof(tx);
        }
    }
}
