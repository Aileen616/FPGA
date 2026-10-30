/* 依赖：tof5_main.c的MMIO、halt与任务状态；旧手机协议仅绑定蓝牙辅助串口。 */
#include "jdy31_motor.h"
#include "mecanum_manual.h"
static jdy_motor_t legacy; /* 旧手机会话，独立代号和命令缓存，不使用USB解析缓存。 */
static char legacy_tx[512]; /* 9600波特率发送队列，整包入队；满时停车。 */
static unsigned legacy_head,legacy_tail;
#define BT_BASE UINT32_C(0x20008000)

/* 功能：整包加入蓝牙应答队列；ctx可空，s非空，n为字节数；空间足够返回true。 */
static bool legacy_write(void *ctx,const char *s,size_t n)
{
    (void)ctx;if(n>(legacy_tail+sizeof(legacy_tx)-legacy_head-1U)%sizeof(legacy_tx))return false;
    for(size_t i=0;i<n;i++){legacy_tx[legacy_head]=s[i];legacy_head=(legacy_head+1U)%sizeof(legacy_tx);}
    return true;
}
/* 功能：处理手动保护事件；ctx可空，event为JDY_M枚举，seq保留；无返回。 */
static void legacy_guard(void *ctx,unsigned event,uint32_t seq)
{
    (void)ctx;(void)seq;
    if(event==JDY_M_ARM){wr(JDY_BASE,12,2);return;}
    halt(T5_CANCEL);
    if(event==JDY_M_CLEAR){wr(MOTOR_BASE,8,1);wr(JDY_BASE,12,1);}
}
/* 功能：合并实际驱动及通信保护；ctx可空；返回旧协议规定的低三位状态。 */
static uint32_t legacy_state(void *ctx)
{
    (void)ctx;uint32_t c=rd(JDY_BASE,8),m=rd(MOTOR_BASE,4);
    return ((c&m)&1U)|((c|m)&2U)|((c&1U)<<2);
}
/* 功能：停车设置旧App轮位配置并关闭速度PI；ctx可空，mask=1..15、invert=0..15；成功返回true。 */
static bool legacy_configure(void *ctx,uint32_t mask,uint32_t invert)
{
    (void)ctx;
    if(mask<1||mask>15||invert>15||(rd(MOTOR_BASE,4)&1U)||robot_active(&robot)||bypass_active(&planner))return false;
    pi_configured=false;configured=false;
    wr(MOTOR_BASE,0x74,0);wr(MOTOR_BASE,12,mask|(invert<<4));
    return rd(MOTOR_BASE,0x74)==0&&rd(MOTOR_BASE,12)==(mask|(invert<<4))&&(rd(MOTOR_BASE,4)&4U);
}
/* 功能：提交旧App四轮手动输出；ctx可空，seq非零递增、key=1..10、level=1..3；PL接受返回true。 */
static bool legacy_drive(void *ctx,uint32_t seq,uint32_t key,uint32_t level)
{
    (void)ctx;mecanum_output_t output;
    static const uint8_t physical_to_wheel[4]={MECANUM_RL,MECANUM_FR,MECANUM_FL,MECANUM_RR};
    if(robot_active(&robot)||bypass_active(&planner)||!seq||(rd(JDY_BASE,8)&3U)!=1U||
       rd(MOTOR_BASE,0x74)!=0||!mecanum_manual_mix(key,level,&output))return false;
    for(unsigned i=0;i<4;i++)wr(MOTOR_BASE,0x50+i*4,(uint32_t)(int32_t)output.effort[physical_to_wheel[i]]);
    wr(MOTOR_BASE,0x10,seq);wr(MOTOR_BASE,0x14,UINT32_C(0x4d4f5633));
    if((rd(MOTOR_BASE,4)&3U)!=1U||rd(MOTOR_BASE,0x44)!=seq)return false;
    wr(JDY_BASE,0x10,seq);return (legacy_state(0)&3U)==1U;
}
/* 功能：撤销旧代号和配置，要求重新显式配置；无参数和返回，不清硬件故障。 */
static void legacy_invalidate(void)
{
    jdy_motor_transport_fault(&legacy);
    legacy.blocked=false;legacy.configured=false;legacy.wheel_mask=legacy.invert=0;
    legacy.sequence=0;legacy.dropping=true;
}
/* 功能：初始化蓝牙兼容层；无参数和返回，上电保持停车且未配置。 */
static void legacy_init(void)
{
    const jdy_motor_io_t io={0,legacy_write,legacy_guard,legacy_state,legacy_configure,legacy_drive};
    jdy_motor_init(&legacy,&io,rd(JDY_BASE,4));
}
