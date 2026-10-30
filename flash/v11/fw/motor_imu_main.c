#include "jdy31_motor.h"
#include "mecanum_manual.h"
#include "bmi088_fifo.h"
#include "heading_hold.h"
#include <string.h>
#define IMU_BASE UINT32_C(0x20004000)
static bmi088_fifo_t imu;
static heading_hold_t heading;
static uint32_t hardware_seq, last_heading_us;
static int32_t heading_base;
static char imu_line[80];
static unsigned imu_length;
static uint32_t imu_rx_ms;
static bool imu_receiving, imu_dropping;
/* 来源：esp_diag_wb与motor_forward_wb板级外设；BX72无STM32 HAL，仅此文件封装MMIO。 */
#define JDY_BASE UINT32_C(0x20002000)
#define MOTOR_BASE UINT32_C(0x20003000)
static jdy_motor_t motor; /* 静态协议状态；物理位与算法轮序的换算仅在drive内进行。 */
static char tx[512]; /* 有界整包发送队列；避免通信拥塞继续运动。 */
static unsigned head,tail;
/* 外部通道1右后、2右前、3左后、4左前，对应RTL物理位3、1、0、2。 */
static const uint8_t channel_to_bit[4]={3,1,0,2};

/* 功能：读板级MMIO；base为两外设基址，offset为合法4字节对齐偏移；返回32位值。 */
static uint32_t rd(uint32_t base,unsigned offset)
{
#ifdef BMI088_HOST_TEST
    extern uint32_t imu_test_read(uint32_t,unsigned);
    return imu_test_read(base,offset);
#else
    uint32_t v=*(volatile uint32_t *)(uintptr_t)(base+offset);__asm__ volatile("":::"memory");return v;
#endif
}
/* 功能：写板级MMIO；base为两外设基址，offset为合法偏移，v为寄存器值；无返回。 */
static void wr(uint32_t base,unsigned offset,uint32_t v)
{
#ifdef BMI088_HOST_TEST
    extern void imu_test_write(uint32_t,unsigned,uint32_t);
    imu_test_write(base,offset,v);
#else
    __asm__ volatile("":::"memory");*(volatile uint32_t *)(uintptr_t)(base+offset)=v;
#endif
}
/* 当前官方PicoRV32无数据缓存、总线逐笔完成，volatile配合编译器屏障保持MMIO次序；
   不依赖旧核未承诺支持的FENCE指令。迁移至有缓存或多笔并发访问的CPU时须重审。 */
/* 功能：整包入队；ctx不用可空，data不可空，n为字节数；容量足够返回true。 */
static bool enqueue(void *ctx,const char *data,size_t n)
{
    (void)ctx;if(n>(tail+sizeof(tx)-head-1U)%sizeof(tx))return false;
    for(size_t i=0;i<n;i++){tx[head]=data[i];head=(head+1U)%sizeof(tx);}return true;
}
/* 功能：提交保护事件；ctx不用可空，event为JDY_M枚举，seq保留；无返回。 */
static void guard(void *ctx,unsigned event,uint32_t seq)
{
    (void)ctx;(void)seq;
    if(event==JDY_M_ARM){wr(JDY_BASE,0x0c,2);return;}
    /* 停车必须先撤销动力，再改变通信状态；清故障本身不得启动任何轮子。 */
    wr(MOTOR_BASE,0x08,4);wr(JDY_BASE,0x0c,4);
    wr(IMU_BASE,0x60,0);heading.running=false;
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
    if(heading.enabled||mask<1||mask>15||invert>15||(rd(JDY_BASE,8)&1U)||(rd(MOTOR_BASE,4)&1U))return false;
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
    if(heading.enabled||heading.calibrating||!seq||(rd(JDY_BASE,8)&3U)!=1U||!mecanum_manual_mix(key,level,&output))return false;
    for(unsigned i=0;i<4;i++)wr(MOTOR_BASE,0x50+i*4,(uint32_t)(int32_t)output.effort[physical_to_wheel[i]]);
    wr(MOTOR_BASE,0x10,seq);wr(MOTOR_BASE,0x14,UINT32_C(0x4d4f5633));
    if((rd(MOTOR_BASE,4)&3U)!=1U||rd(MOTOR_BASE,0x44)!=seq)return false;
    /* 仅成功的运动提交能够刷新通信保护；不存在普通hb刷新动力的旁路。 */
    wr(JDY_BASE,0x10,seq);
    return (state(0)&3U)==1U;
}
/* 功能：把RTL物理四位掩码转换成通道1..4顺序；value仅低四位有效；返回四位掩码。 */
static uint32_t channel_mask(uint32_t value)
{
    uint32_t result=0;
    for(unsigned c=0;c<4;c++)result|=((value>>channel_to_bit[c])&1U)<<c;
    return result;
}
/* 功能：冻结并读取四路反馈；ctx可空，out不可空；能力、快照编号和周期合法返回true。 */
static bool snapshot(void *ctx,jdy_motor_snapshot_t *out)
{
    (void)ctx;
    if(!out||rd(MOTOR_BASE,0x70)!=UINT32_C(0x434c5031))return false;
    wr(MOTOR_BASE,0x08,8);
    uint32_t sequence=rd(MOTOR_BASE,0xd0),diagnostics=rd(MOTOR_BASE,0xd4);
    uint32_t mask=rd(MOTOR_BASE,0x0c)&15U,ready=rd(MOTOR_BASE,0x7c)&15U;
    out->sequence=sequence;out->mode=rd(MOTOR_BASE,0x74);
    out->diagnostics=diagnostics&UINT32_C(0xff0000ff);
    for(unsigned shift=8;shift<=20;shift+=4)
        out->diagnostics|=channel_mask((diagnostics>>shift)&15U)<<shift;
    out->encoder_invert_mask=channel_mask(rd(MOTOR_BASE,0x78));
    out->tick_ms=rd(MOTOR_BASE,0xd8)/1000U;
    out->ready=mask!=0&&(ready&mask)==mask&&((rd(MOTOR_BASE,0xdc)>>8)&mask)==mask;
    for(unsigned c=0;c<4;c++){
        unsigned bit=channel_to_bit[c],offset=bit*4;
        out->position[c]=(int32_t)rd(MOTOR_BASE,0x20+offset);
        out->illegal[c]=rd(MOTOR_BASE,0x30+offset);
        out->cps[c]=(int32_t)rd(MOTOR_BASE,0xc0+offset);
        out->effort[c]=(int32_t)rd(MOTOR_BASE,0xe0+offset);
        out->target[c]=(int32_t)rd(MOTOR_BASE,0xf0+offset);
        if((mask&(1U<<bit))&&!(rd(MOTOR_BASE,0x80+offset)|rd(MOTOR_BASE,0x90+offset)))out->ready=0;
    }
    return out->mode<=1&&out->tick_ms==10&&sequence==rd(MOTOR_BASE,0xd0);
}
/* 功能：按通道读取实际PI参数；ctx可空、channel为1..4、out不可空；成功true。 */
static bool pid_get(void *ctx,uint32_t channel,jdy_motor_pid_t *out)
{
    (void)ctx;if(channel<1||channel>4||!out)return false;
    unsigned bit=channel_to_bit[channel-1],offset=bit*4;
    out->kp_q16=rd(MOTOR_BASE,0x80+offset);out->ki_tick_q16=rd(MOTOR_BASE,0x90+offset);
    out->kd_q16=0;out->limit=rd(MOTOR_BASE,0xa0+offset);
    out->enc_inv=(rd(MOTOR_BASE,0x78)>>bit)&1U;return true;
}
/* 功能：停机写入一轮PI并核对全部回读；ctx可空、channel为1..4、p不可空；失败false。 */
static bool pid_set(void *ctx,uint32_t channel,const jdy_motor_pid_t *p)
{
    (void)ctx;
    if(heading.enabled||channel<1||channel>4||!p||(rd(JDY_BASE,8)&1U)||(rd(MOTOR_BASE,4)&1U)||
       p->kp_q16>INT32_MAX||p->ki_tick_q16>INT32_MAX||p->kd_q16||p->limit<1||p->limit>16384||p->enc_inv>1)return false;
    unsigned bit=channel_to_bit[channel-1],offset=bit*4;
    uint32_t invert=rd(MOTOR_BASE,0x78);
    invert=(invert&~(1U<<bit))|(p->enc_inv<<bit);
    wr(MOTOR_BASE,0x78,invert);
    wr(MOTOR_BASE,0x80+offset,p->kp_q16);wr(MOTOR_BASE,0x90+offset,p->ki_tick_q16);
    wr(MOTOR_BASE,0xa0+offset,p->limit);
    jdy_motor_pid_t actual;
    return pid_get(0,channel,&actual)&&actual.kp_q16==p->kp_q16&&actual.ki_tick_q16==p->ki_tick_q16&&
        actual.limit==p->limit&&actual.enc_inv==p->enc_inv&&
        (!(p->kp_q16|p->ki_tick_q16)||(rd(MOTOR_BASE,0x7c)&(1U<<bit)));
}
/* 功能：停机切换开环或PI模式；ctx可空、closed为期望模式；回读一致返回true。 */
static bool loop_mode(void *ctx,bool closed)
{
    (void)ctx;
    if(heading.enabled||(rd(JDY_BASE,8)&1U)||(rd(MOTOR_BASE,4)&1U))return false;
    jdy_motor_snapshot_t data;
    if(!snapshot(0,&data)||(closed&&!data.ready))return false;
    wr(MOTOR_BASE,0x74,closed?1U:0U);
    return rd(MOTOR_BASE,0x74)==(closed?1U:0U);
}
/* 功能：仅提交本地电机目标，不刷新蓝牙租约；target四项不可空；成功true。 */
static bool commit_targets(const int32_t target[4])
{
    if(hardware_seq==UINT32_MAX)return false;
    uint32_t seq=++hardware_seq;
    for(unsigned c=0;c<4;c++)wr(MOTOR_BASE,0xb0+channel_to_bit[c]*4,(uint32_t)target[c]);
    wr(MOTOR_BASE,0x10,seq);wr(MOTOR_BASE,0x14,UINT32_C(0x434c5331));
    return (rd(MOTOR_BASE,4)&3U)==1U&&rd(MOTOR_BASE,0x44)==seq;
}
/* 功能：外部合法速度提交；ctx可空，seq新鲜、target四项不可空；成功才续蓝牙租约。 */
static bool velocity(void *ctx,uint32_t seq,const int32_t target[4])
{
    (void)ctx;int32_t adjusted[4];
    if(heading.calibrating||!target||!seq||rd(MOTOR_BASE,0x74)!=1||(rd(JDY_BASE,8)&3U)!=1U)return false;
    for(unsigned c=0;c<4;c++)if(target[c]<-200000||target[c]>200000)return false;
    if(heading.enabled){
        if(!imu.valid)return false;
        for(unsigned c=1;c<4;c++)if(target[c]!=target[0])return false;
        if(!heading.running){
            if(!heading_begin(&heading))return false;
            wr(IMU_BASE,0x60,1);if(rd(IMU_BASE,0x60)!=1)return false;
            heading_base=target[0];last_heading_us=rd(IMU_BASE,8);
        }
        if(!heading_targets(&heading,target[0],adjusted))return false;
        target=adjusted;
    }
    if(!commit_targets(target))return false;
    wr(JDY_BASE,0x10,seq);return (state(0)&3U)==1U;
}
#include "motor_imu_service.inc"

/* 功能：运行裸机完整低输出遥控；无参数，正常不返回；上电不配置、不启动、不自动恢复。 */
int main(void)
{
    const jdy_motor_io_t io={0,enqueue,guard,state,configure,drive};
    const jdy_motor_control_io_t control={0,snapshot,pid_get,pid_set,loop_mode,velocity};
    if(rd(JDY_BASE,0)!=UINT32_C(0x45535001)||rd(MOTOR_BASE,0)!=UINT32_C(0x4d465703))for(;;){}
    jdy_motor_init(&motor,&io,rd(JDY_BASE,4));
    if(rd(MOTOR_BASE,0x70)!=UINT32_C(0x434c5031)||!jdy_motor_set_control_io(&motor,&control))for(;;){}
    imu_start();
    for(;;){
        imu_service();
        uint32_t now=rd(JDY_BASE,4);
        if(rd(JDY_BASE,0x1c))jdy_motor_transport_fault(&motor);
        jdy_motor_poll(&motor,now);
        uint32_t c=rd(JDY_BASE,0x14);
        if(!(c&0x100U))imu_receive((uint8_t)c,now);
        if(tail!=head&&!(rd(JDY_BASE,0x18)&0x100U)){
            wr(JDY_BASE,0x18,(uint8_t)tx[tail]);tail=(tail+1U)%sizeof(tx);
        }
    }
}
