#include "ps2_control.h"
static void maze_imu_service(void);
static void imu_quality_reset(void);
static uint8_t imu_quality_fault; /* 已确认的采样异常，显式停机初始化前保持锁存。 */
static void maze_link_error(void);
static bool maze_trial; /* 仅本次显式试跑持权；不是动态标定通过。 */
static bool maze_owned; /* 仅明确启动且动态标定通过后取得电机所有权。 */
static void maze_halt(void);
static void maze_record_stop(unsigned reason);
static void maze_service(void);
static bool pad_owner; /* 手柄自主任务占用标志。 */
static uint8_t pad_cal_stage; /* 0空闲、1初始化、2静止标定。 */
static void pad_osd(void);
static void pad_stop(void);
#include "robot_mission.h"
#include "perf_monitor.h"
static robot_mission_t robot;
static void robot_tick(void);
#include "tof5_control.h"
#include "autonomy_policy.h"
#include "fpga_vision.h"
#include "vision_profile.h"
static void vision_service(void);
#include "autonomy_ps2_adapter.h"
static ap_policy_t autonomy;
static bool autonomy_owned,autonomy_initialized;
static void autonomy_halt(tof5_reason_t reason);
static void autonomy_init(void);
static void autonomy_tick(void);
static bool autonomy_begin(uint32_t seq,uint8_t task);
static void legacy_invalidate(void);
#include "bypass_return.h"
#include "bypass_direction_policy.h"
#include "sensor_joint_array.h"
#include <string.h>
#include "bmi088_fifo.h"
#include "heading_calibration.h"
#include "heading_straight.h"
/* BX72官方PicoRV32裸机平台，无STM32 HAL；MMIO只封装在本文件。 */
/* ST ULD模式2为长距；只切模式，保留50ms预算、100ms周期和200ms失鲜保护。 */
#define TOF5_DISTANCE_MODE 2U
#define JDY_BASE UINT32_C(0x20002000)
#define MOTOR_BASE UINT32_C(0x20003000)
#define IMU_BASE UINT32_C(0x20004000)
#define TOF_BASE UINT32_C(0x20005000)
static bmi088_fifo_t imu; /* DRDY完整帧，不读取FIFO积压替代航向采样。 */
static heading_hold_t heading;
static heading_straight_t straight;
static heading_calibration_t calibration; /* 6KB静态原始窗口，标定时只清元数据。 */
static bool imu_started;
static uint32_t right_settled_counts;
static bypass_fsm_t planner; /* 三段任务，只在本地状态机允许时切换方向。 */
static uint32_t sequence, renewed_ms, started_generation, last_motion_ms, forward_counts;
static const bypass_config_t geometry={280,250,350,20,550,200,true,true};
/* 横移仅有1483counts/280mm单点对照，系数仍5268.945；true仅允许受限原型验证，不代表完整标定。 */
static tof5_control_t mission;         /* 本地任务状态，复位默认禁能。 */
static vl53l1x_array_t joint_tof;      /* 槽位0/1/2对应X2前/X3右/X4左。 */
static bool tof_running, configured;
static uint32_t hw_sequence,last_tick,last_tof,rx_stamp;
static uint32_t illegal_start[4],stall_stamp[4],last_position[4];
static const uint8_t channel_to_bit[4]={3,1,0,2}; /* 通道RR/FR/RL/FL到物理位。 */
static char tx[1024],line[96];          /* 固定容量，无堆；命令超长即停车。 */
static unsigned head,tail,length;
static bool dropping;
/* 功能：读取合法MMIO偏移；base为上述基址、off四字节对齐；返回32位值。 */
static uint32_t rd(uint32_t base,unsigned off)
{
#ifdef TOF5_HOST_TEST
    extern uint32_t tof5_test_read(uint32_t,unsigned);
    return tof5_test_read(base,off);
#else
    uint32_t v=*(volatile uint32_t *)(uintptr_t)(base+off);
    __asm__ volatile("":::"memory");return v;
#endif
}
/* 功能：写合法MMIO；base/off范围同rd、v为寄存器值；无返回。 */
static void wr(uint32_t base,unsigned off,uint32_t v)
{
#ifdef TOF5_HOST_TEST
    extern void tof5_test_write(uint32_t,unsigned,uint32_t);
    tof5_test_write(base,off,v);
#else
    __asm__ volatile("":::"memory");*(volatile uint32_t *)(uintptr_t)(base+off)=v;
#endif
}
/* 功能：立即撤销驱动和串口租约；reason有效；无返回，保留首个停车原因。 */
static void halt(tof5_reason_t reason)
{
    bool was_maze=maze_owned;maze_record_stop(reason);maze_halt();
    autonomy_halt(reason);
    if(robot_active(&robot))robot_stop(&robot,R_CANCEL);
    wr(MOTOR_BASE,8,4);wr(JDY_BASE,12,4);if(was_maze){wr(JDY_BASE,12,32);wr(UINT32_C(0x20008000),12,32);}
    wr(IMU_BASE,0x60,0);heading.running=heading.enabled=false;
    if(tof5_active(&mission)||mission.state==T5_IDLE)tof5_stop(&mission,reason);
    if(bypass_active(&planner))bypass_abort(&planner,reason==T5_SENSOR?B_SENSOR:reason==T5_LINK?B_LINK:reason==T5_CANCEL?B_CANCEL:B_SEGMENT_FAULT);
}
#include "imu_display_state.inc"
#include "bypass_return_imu.inc"
/* 功能：核对新鲜已标定轴向；无参数；可用于本次航向保持时true。 */
static bool imu_ready(void)
{
    return imu_started&&imu.valid&&imu.error==SENSOR_OK&&heading.calibrated&&!heading.calibrating&&
        heading.axis_confirmed&&heading.axis==2&&heading.sign==1&&!heading.fault&&heading.feedback_valid&&
        (uint32_t)(rd(IMU_BASE,8)-heading.stamp_us)<20000U;
}
/* 功能：发送NUL结尾文本；s不可空；无返回，缓冲不足立即停车。 */
static void reply(const char *s)
{
    size_t n=strlen(s);
    if(n>(tail+sizeof(tx)-head-1U)%sizeof(tx)){if(maze_owned)maze_link_error();else halt(T5_LINK);return;}
    while(*s){tx[head]=*s++;head=(head+1U)%sizeof(tx);}
}
#include "forward_guard_trace.inc"
/* 功能：取当前段输入与原子计数；v不可空；无返回。 */
static void inputs(tof5_input_t *v)
{
    unsigned index=mission.raw_invert==10?1U:mission.raw_invert==5?2U:0U;
    const tof_sample_t *t=&joint_tof.sample[index];
    v->now_ms=rd(JDY_BASE,4);v->age_us=rd(TOF_BASE,8)-t->timestamp_us;
    v->valid=tof_running&&vl53l1x_array_ready(&joint_tof)&&t->received&&
        t->validity==SENSOR_VALID&&t->error==SENSOR_OK&&t->range_status==0;
    v->distance_mm=t->distance_mm;
    v->drive_ok=(rd(MOTOR_BASE,4)&3U)==1U&&(rd(JDY_BASE,8)&3U)==1U;
    wr(MOTOR_BASE,8,8);
    for(unsigned c=0;c<4;c++)v->position[c]=(int32_t)rd(MOTOR_BASE,0x20+channel_to_bit[c]*4);
}
/* 功能：组装遥测和当前方向准入；v不可空；无返回，仅选中方向的无效/失鲜拒绝。 */
static void planning_inputs(bypass_input_t *v)
{
    memset(v,0,sizeof(*v));v->now_ms=rd(JDY_BASE,4);
    v->lease_valid=(uint32_t)(v->now_ms-renewed_ms)<250U;
    unsigned selected=bypass_tof_channel(planner.phase);
    v->sensors_fresh=selected==3||(tof_running&&vl53l1x_array_ready(&joint_tof));
    uint16_t *dist[3]={&v->front_mm,&v->right_mm,&v->left_mm};
    for(unsigned i=0;i<3;i++){
        const tof_sample_t *t=&joint_tof.sample[i];unsigned offset=i?60U:0U;
        *dist[i]=t->distance_mm>offset?(uint16_t)(t->distance_mm-offset):0;
        if(i==selected&&(!t->received||t->validity!=SENSOR_VALID||t->error!=SENSOR_OK||t->range_status!=0||
           (uint32_t)(rd(TOF_BASE,8)-t->timestamp_us)>=200000U))v->sensors_fresh=false;
    }
    v->imu_ready=imu_ready();v->left_sequence=joint_tof.sample[2].sequence;
    v->front_sequence=joint_tof.sample[0].sequence;
    v->wheels_moving=(uint32_t)(v->now_ms-last_motion_ms)<20U;
    v->result_generation=started_generation;
    v->result=tof5_active(&mission)?B_SEG_RUNNING:mission.reason==T5_DISTANCE?B_SEG_DISTANCE:
        mission.reason==T5_OBSTACLE?B_SEG_OBSTACLE:B_SEG_FAILED;
}
/* 功能：提交平移加航向纠偏；effort为0..19660；成功true，所有轮子绝对输出不超过60%。 */

/* 功能：取本横移段平均有向计数，包含停车后滑行；无参数；返回正向平均counts。 */

/* 功能：将毫米换算为名义编码器计数；mm为0..6000；返回向上取整计数。 */
static uint32_t counts(uint32_t mm)
{return (uint32_t)(((uint64_t)mm*5268945U+999999U)/1000000U);}
/* 功能：执行状态机已批准的新段；无参数；成功true，失败不再试。 */

#include "encoder_fault_trace.inc"
/* 功能：每10ms执行本地保护/静止检查/分段切换；无参数/返回，无I2C调用。 */
static void control_tick(void)
{
    uint32_t now=rd(JDY_BASE,4);
    if((uint32_t)(now-last_tick)<10U)return;
    last_tick=now;if(maze_owned){maze_service();return;}vision_service();tof5_input_t v;inputs(&v);guard_observe();
    for(unsigned c=0;c<4;c++)if((uint32_t)v.position[c]!=last_position[c]){
        last_position[c]=(uint32_t)v.position[c];stall_stamp[c]=last_motion_ms=now;
    }
    robot_tick();
}
/* 该文件由离线工具从已验证的联合测距I2C适配派生，保留X2/X3/X4，原驱动不改。 */
/* 功能：非阻塞发送一个UART字节；无参数和返回值。 */
static void uart_tx_service(void);
#include "tof5_bus.inc"
/* 功能：停机初始化前向ToF；无参数；成功true，失败false；不会启动电机。 */
static uint32_t tof_poll_count[4],tof_poll_max_us[4]; /* 本轮各通道服务次数与最长间隔，单位us。 */
static bool tof_start(void)
{
    if(bypass_active(&planner))return false;
    tof_stop();wr(TOF_BASE,0x10,256);
    const sensor_bus_t bus={.i2c_read=tof_read,.i2c_write=tof_write,.xshut=tof_shutdown};
    const tof_config_t cfg={.count=4,.address7={0x31,0x32,0x33,0x30},.distance_mode=TOF5_DISTANCE_MODE,
        .budget_ms=50,.period_ms=100,.fresh_us=200000,.io_mode=TOF_IO_AVDD};
    memset(tof_poll_count,0,sizeof(tof_poll_count));memset(tof_poll_max_us,0,sizeof(tof_poll_max_us));
    tof_running=vl53l1x_array_start(&joint_tof,&bus,&cfg,rd(TOF_BASE,8))==SENSOR_PENDING;
    return tof_running;
}
#include "ps2_start_gate.inc"
/* 功能：启动一次完整绕障；seq为新的非零序号；成功true。 */
static bool begin(uint32_t seq) { return autonomy_begin(seq,3); }
/* 功能：解析严格十进制uint32；s/v不可空、不允许尾随字符；合法返回true。 */
static bool number(const char *s,uint32_t *v)
{
    uint32_t n=0;if(!*s)return false;
    while(*s){if(*s<'0'||*s>'9')return false;uint32_t d=(unsigned)(*s++-'0');
        if(n>(UINT32_MAX-d)/10U)return false;
        n=n*10U+d;}
    *v=n;return true;
}
/* 功能：追加文本；p/s不可空且p空间足够；返回新末端。 */
static char *text(char *p,const char *s){while(*s)*p++=*s++;return p;}
/* 功能：追加无符号十进制；p至少10字节、n任意；返回新末端。 */
static char *decimal(char *p,uint32_t n)
{char b[10];unsigned k=0;do{b[k++]=(char)('0'+n%10U);n/=10U;}while(n);while(k)*p++=b[--k];return p;}
/* 功能：追加键值；p/key不可空、n任意；返回新末端。 */
static char *field(char *p,const char *key,uint32_t n){return decimal(text(p,key),n);}
/* 功能：回读任务及三路净距；无参数/返回，不续期。 */
static void status(void)
{
    char b[512],*p=b;bypass_input_t v;planning_inputs(&v);
    p=text(p,"B2 V=1 DM=2 PROFILE=783");p=field(p," PHASE=",planner.phase);p=field(p," REASON=",planner.reason);
    p=field(p," STATE=",mission.state);p=field(p," WHY=",mission.reason);p=field(p," PWM=",mission.effort);
    p=field(p," COUNT=",mission.progress>0?(uint32_t)mission.progress:0);p=field(p," TARGET=",mission.target_counts);
    p=field(p," FRONT=",v.front_mm);p=field(p," RIGHT=",v.right_mm);p=field(p," LEFT=",v.left_mm);
    p=field(p," VALID=",v.sensors_fresh);p=field(p," SEQ=",sequence);p=field(p," GEN=",planner.generation);
    p=field(p," RUN=",tof_running);p=field(p," READY=",vl53l1x_array_ready(&joint_tof));p=field(p," ACTIVE=",rd(MOTOR_BASE,4)&1U);
    p=field(p," RETURNCOUNT=",right_settled_counts);p=field(p," IMUOK=",imu_ready());
    p=field(p," COMPLETE=",planner.phase==B_DONE&&planner.returned);
    static const char *const keys[3]={" SF="," SR="," SL="};
    static const char *const ages[3]={" AF="," AR="," AL="};
    static const char *const errors[3]={" EF="," ER="," EL="};
    for(unsigned i=0;i<3;i++){
        const tof_sample_t *t=&joint_tof.sample[i];p=field(p,keys[i],t->sequence);
        p=field(p,ages[i],t->received?rd(TOF_BASE,8)-t->timestamp_us:UINT32_MAX);
        p=field(p,errors[i],t->range_status);
    }
    p=text(p,"\r\n");*p=0;reply(b);
}
#include "forward_guard_reply.inc"
#include "bypass_return_commands.inc"
#include "competition_service.inc"
#include "vision_service.inc"
/* 功能：新增视觉片段的轻量IMU让出点；无参数/返回，有新事件才调用原完整读取校验。
 * 只读快照不更新消费序号；原主循环和I2C前后仍完整服务，失鲜保护不变。 */
static void endpoint_imu_service(void)
{
    if(!irq_armed){imu_service();return;}
    /* 无新事件时省去完整SPI服务的重复快照，但仍每个采样周期兜底检查失鲜/故障。 */
    uint32_t now=rd(IMU_BASE,8);
    if(!imu.valid||imu.error<0||heading.fault||irq_missed_acc||irq_missed_gyro||irq_races||imu.rejected_frames||
       (uint32_t)(now-imu.acc_us)>=5000U||(uint32_t)(now-imu.gyro_us)>=5000U){imu_service();return;}
    wr(IMU_BASE,0x40,1);
    uint32_t acc=rd(IMU_BASE,0x48),gyro=rd(IMU_BASE,0x4c);
    if(acc!=irq_last_acc||gyro!=irq_last_gyro)imu_service();
    else imu.new_gyro=false; /* 与完整服务无事件分支一致，不让上轮新样本标志重复消费。 */
}
#include "endpoint_service.inc"
#include "autonomy_service.inc"
#include "tof4_telemetry.inc"
#include "maze_board_static.inc"
#include "legacy_bluetooth.inc"
#include "imu_display_service.inc"
#include "ps2_task3_service.inc"
#include "encoder_fault_reply.inc"
#include "encoder_live_reply.inc"
#include "encoder_edge_reply.inc"
/* 功能：只接受绕障命令白名单；s为非空NUL结尾字符串；无返回。 */
static void command(const char *s)
{
    uint32_t n;
    if(maze_map_command(s))return;
    if(maze_command(s))return;
    /* 运动所有权期间只读诊断和停止允许，配置及其他任务不得争抢驱动。 */
    if(maze_owned&&strcmp(s,"stop")&&strcmp(s,"robot")&&strcmp(s,"robot state")&&strcmp(s,"imu")&&
       strncmp(s,"tof read ",9)&&strcmp(s,"endpoint status")&&strcmp(s,"endpoint timing")&&strcmp(s,"endpoint work")&&
       strncmp(s,"enc live ",9)){reply("MZ BUSY\r\n");return;}
    if(endpoint_command(s))return;
    if(vision_command(s))return;
    if(!strcmp(s,"auto status")){autonomy_reply();return;}
    if(!strncmp(s,"enc edge ",9)&&number(s+9,&n)&&n<4U){encoder_edge_reply(n);return;}
    if(!strncmp(s,"enc live ",9)&&number(s+9,&n)&&n<4U){encoder_live_reply(n);return;}
    if(!strncmp(s,"enc fault ",10)&&number(s+10,&n)&&n<4U){encoder_fault_reply(n);return;}
    if(!strncmp(s,"ps2 last ",9)&&number(s+9,&n)&&n<4U){pad_history_reply(n);return;}
    if(!strcmp(s,"stop")){legacy_invalidate();pad_stop();reply("B2 STOP=1\r\n");return;}
    if((pad_owner||pad_cal_stage||(autonomy_owned&&auto_task==3))&&strcmp(s,"robot")&&strcmp(s,"robot state")&&strcmp(s,"robot video")&&strcmp(s,"robot perf")&&
       strcmp(s,"imu")&&strcmp(s,"imu health")&&strcmp(s,"imu recovery")&&strcmp(s,"imu calstats")&&strcmp(s,"tof")&&strcmp(s,"bypass")&&
       strcmp(s,"diag")&&strcmp(s,"guard")&&strncmp(s,"tof read ",9)){reply("PS2 BUSY\r\n");return;}
    /* USB只读查询不影响蓝牙；其他USB请求先撤销旧手势，避免共用驱动配置。 */
    if(!autonomy_owned&&(legacy.configured||legacy.active||legacy.length||legacy.blocked)&&strcmp(s,"robot")&&strcmp(s,"robot state")&&strcmp(s,"robot video")&&strcmp(s,"robot perf")&&
       strcmp(s,"imu")&&strcmp(s,"imu health")&&strcmp(s,"imu recovery")&&strcmp(s,"tof")&&strcmp(s,"diag")&&strcmp(s,"guard")&&strncmp(s,"tof read ",9))
        legacy_invalidate();
    if(!strncmp(s,"tof read ",9)&&number(s+9,&n)&&n<4){tof4_read_reply(n);return;}
    if(!strcmp(s,"imu recovery")){imu_display_reply();return;}
    if(autonomy_owned&&strcmp(s,"robot")&&strcmp(s,"robot state")&&strcmp(s,"robot video")&&strcmp(s,"robot perf")&&
       strcmp(s,"imu")&&strcmp(s,"imu health")&&strcmp(s,"imu recovery")&&strcmp(s,"tof")&&strcmp(s,"bypass")&&
       strcmp(s,"diag")&&strcmp(s,"guard")&&strncmp(s,"tof read ",9)&&strncmp(s,"robot target ",13)&&
       strncmp(s,"robot keep ",11)&&strncmp(s,"bypass keep ",12)){reply("AUTO BUSY\r\n");return;}
    if(autonomy_owned&&(!strncmp(s,"bypass keep ",12)||!strncmp(s,"robot keep ",11)))return; /* 旧外部续期不影响自主任务。 */
    if(robot_command(s))return;
    if(robot_active(&robot)&&strcmp(s,"imu")&&strcmp(s,"imu health")&&strcmp(s,"imu recovery")&&strcmp(s,"tof")&&strcmp(s,"bypass")&&strcmp(s,"stop")&&strcmp(s,"diag")&&strcmp(s,"guard")){
        halt(T5_CANCEL);reply("ROBOT REJECTED ACTIVE\r\n");return;
    }
    if(!strncmp(s,"imu",3)){imu_command(s);return;}
    if(!strcmp(s,"bypass")||!strcmp(s,"tof")){status();return;}
    if(!strcmp(s,"guard")){guard_reply();return;}
    if(!strcmp(s,"guard clear")){if(!bypass_active(&planner)){guard_reset();reply("G783 CLEAR=1\r\n");}return;}
    if(!strcmp(s,"diag")){
        char b[512],*p=b;p=text(p,"D783");
        p=field(p," STATE=",joint_tof.state);p=field(p," INDEX=",joint_tof.index);p=field(p," ADDR=",tof_error_addr);p=text(p," ERR=");p=signed_number(p,joint_tof.error);
        p=field(p," REG=",tof_error_reg);p=field(p," STAGE=",tof_error_stage);p=field(p," STATUS=",tof_error_status);
        for(unsigned i=0;i<4;i++){
            p=field(p," WAIT=",joint_tof.wait_ready_low[i]);p=field(p," CLEAR=",joint_tof.clear_confirmed[i]);p=field(p," PENDING=",joint_tof.clear_still_ready[i]);
        }
        p=text(p,"\r\n");*p=0;reply(b);return;
    }
    if(!strcmp(s,"stop")){halt(T5_CANCEL);reply("B2 STOP=1\r\n");return;}
    if(!strcmp(s,"tof stop")){halt(T5_SENSOR);tof_stop();reply("TOF STOP=1\r\n");return;}
    if(!strcmp(s,"tof start")){reply(tof_start()?"TOF START=1\r\n":"TOF START=0\r\n");return;}
    if(!strcmp(s,"bypass arm")){
        if(bypass_active(&planner)){reply("B2 ARM=0\r\n");return;}
        halt(T5_CANCEL);wr(MOTOR_BASE,12,0x5f);wr(MOTOR_BASE,0x74,0);
        configured=rd(MOTOR_BASE,12)==0x5f&&rd(MOTOR_BASE,0x74)==0;
        reply(configured?"B2 ARM=1\r\n":"B2 ARM=0\r\n");return;
    }
    if(!strncmp(s,"bypass start ",13)&&number(s+13,&n)){
        reply(begin(n)?"B2 START=1\r\n":"B2 START=0\r\n");return;
    }
    if(!strncmp(s,"bypass keep ",12)&&number(s+12,&n)){
        if(autonomy_owned)return; /* 自主续期只由本地有效控制步完成。 */
        control_tick();
        if(bypass_active(&planner)&&n>sequence&&n){
            if((uint32_t)(rd(JDY_BASE,4)-renewed_ms)>=250U){halt(T5_LINK);return;}
            sequence=n;renewed_ms=rd(JDY_BASE,4);wr(JDY_BASE,16,n);
        }
        return;
    }
    halt(T5_CANCEL);reply("B2 REJECTED\r\n");
}
/* 功能：处理UART字节；c为0..255、now毫秒；无返回，波浪号可中断任何半包。 */
static void receive(uint8_t c,uint32_t now)
{
    rx_stamp=now;
    if(c=='~'){pad_stop();length=0;dropping=false;return;}
    if(c=='\r')return;
    if(c=='\n'){if(!dropping&&length){line[length]=0;command(line);}length=0;dropping=false;return;}
    if(c<32||c>126||length>=sizeof(line)-1){if(maze_owned){if(!dropping)maze_link_error();}else halt(T5_LINK);length=0;dropping=true;return;}
    if(!dropping)line[length++]=(char)c;
}
#include "competition_uart.inc"
#include "tof_direction_priority.inc"
/* 功能：裸机主循环；无参数、正常不返回；上电初始化传感器供显示；姿态零偏和运动仍需显式命令。 */
int main(void)
{
    fv_init(&task3_vision);task3_vision.enabled=true;autonomy_init();autonomy_initialized=true;
    ps2_control_init(&pad);pad.task=pad.selected=3;
    robot_init(&robot);heading_init(&heading);bypass_init(&planner);tof5_init(&mission);halt(T5_CANCEL);wr(TOF_BASE,12,0);
    if(rd(JDY_BASE,0)!=UINT32_C(0x45535001)||rd(MOTOR_BASE,0)!=UINT32_C(0x4d465703)||
       rd(MOTOR_BASE,0x70)!=UINT32_C(0x434c5031))for(;;){}
    legacy_init();
    perf_monitor_init(perf_enqueue);wr(IMU_BASE,0x14,21);imu_start();imu_display_manual_reset();
    /* 每次上电仅提交一次四路非阻塞初始化，后续由主循环推进；失败保持停机，不自动重试。 */
    if(!tof_start())halt(T5_SENSOR);
    for(;;){
        imu_service();imu_display_service();control_tick();uint32_t now=rd(JDY_BASE,4),us=rd(TOF_BASE,8);
        jdy_motor_poll(&legacy,now);
        if(length&&(uint32_t)(now-rx_stamp)>=1000U){if(maze_owned)maze_link_error();else halt(T5_LINK);length=0;dropping=true;}
        if(tof_running&&(uint32_t)(us-last_tof)>=6000U){
            last_tof=us;tof_select_priority(us);maze_tof_step(us);
            if(joint_tof.error<0){halt(T5_SENSOR);tof_stop();}
        }
        pad_service();uart_rx_service();imu_service();imu_display_service();control_tick();uart_tx_service();display_service();
        /* 每个识别片段之间处理原控制与手柄，识别错误只影响自身结果，不撤销原驱动许可。 */
        for(unsigned endpoint_round=0;endpoint_round<4;endpoint_round++){
            endpoint_service();uart_rx_service();endpoint_imu_service();control_tick();pad_service();
            jdy_motor_poll(&legacy,rd(JDY_BASE,4));uart_tx_service();
            uint32_t endpoint_us=rd(TOF_BASE,8);
            if(tof_running&&(uint32_t)(endpoint_us-last_tof)>=6000U){
                last_tof=endpoint_us;tof_select_priority(endpoint_us);maze_tof_step(endpoint_us);
                if(joint_tof.error<0){halt(T5_SENSOR);tof_stop();}
            }
        }
        maze_service();
        perf_monitor_queue((head+sizeof(tx)-tail)%sizeof(tx));if(perf_monitor_pending)perf_monitor_service(now);
    }
}
