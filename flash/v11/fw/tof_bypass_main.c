#include "tof5_control.h"
#include "tof_bypass_fsm.h"
#include "sensor_joint_array.h"
#include <string.h>
/* BX72官方PicoRV32裸机平台，无STM32 HAL；MMIO只封装在本文件。 */
/* ST ULD模式2为长距；只切模式，保留50ms预算、100ms周期和200ms失鲜保护。 */
#define TOF5_DISTANCE_MODE 2U
#define JDY_BASE UINT32_C(0x20002000)
#define MOTOR_BASE UINT32_C(0x20003000)
#define TOF_BASE UINT32_C(0x20005000)
static bypass_fsm_t planner; /* 三段任务，只在本地状态机允许时切换方向。 */
static uint32_t sequence, renewed_ms, started_generation, last_motion_ms, forward_counts;
static const bypass_config_t geometry={270,255,210,55,650,400,true,true};
/* 横移仅有1483counts/280mm单点对照，系数仍5268.945；true仅允许受限原型验证，不代表完整标定。 */
static tof5_control_t mission;         /* 本地任务状态，复位默认禁能。 */
static vl53l1x_array_t joint_tof;      /* 槽位0/1/2对应X2前/X3右/X4左。 */
static bool tof_running, configured;
static uint32_t hw_sequence,last_tick,last_tof,rx_stamp;
static uint32_t illegal_start[4],stall_stamp[4],last_position[4];
static const uint8_t channel_to_bit[4]={3,1,0,2}; /* 通道RR/FR/RL/FL到物理位。 */
static char tx[1024],line[48];          /* 固定容量，无堆；命令超长即停车。 */
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
    wr(MOTOR_BASE,8,4);wr(JDY_BASE,12,4);
    if(tof5_active(&mission)||mission.state==T5_IDLE)tof5_stop(&mission,reason);
    if(bypass_active(&planner))bypass_abort(&planner,reason==T5_SENSOR?B_SENSOR:reason==T5_LINK?B_LINK:reason==T5_CANCEL?B_CANCEL:B_SEGMENT_FAULT);
}
/* 功能：发送NUL结尾文本；s不可空；无返回，缓冲不足立即停车。 */
static void reply(const char *s)
{
    size_t n=strlen(s);
    if(n>(tail+sizeof(tx)-head-1U)%sizeof(tx)){halt(T5_LINK);return;}
    while(*s){tx[head]=*s++;head=(head+1U)%sizeof(tx);}
}
/* 功能：取当前段输入与原子计数；v不可空；无返回。 */
static void inputs(tof5_input_t *v)
{
    unsigned index=mission.raw_invert==10?1U:0U;
    const tof_sample_t *t=&joint_tof.sample[index];
    v->now_ms=rd(JDY_BASE,4);v->age_us=rd(TOF_BASE,8)-t->timestamp_us;
    v->valid=tof_running&&vl53l1x_array_ready(&joint_tof)&&t->received&&
        t->validity==SENSOR_VALID&&t->error==SENSOR_OK&&t->range_status==0;
    v->distance_mm=t->distance_mm;
    v->drive_ok=(rd(MOTOR_BASE,4)&3U)==1U&&(rd(JDY_BASE,8)&3U)==1U;
    wr(MOTOR_BASE,8,8);
    for(unsigned c=0;c<4;c++)v->position[c]=(int32_t)rd(MOTOR_BASE,0x20+channel_to_bit[c]*4);
}
/* 功能：组装三路净距和租约；v不可空；无返回，任何无效/失鲜均拒绝。 */
static void planning_inputs(bypass_input_t *v)
{
    memset(v,0,sizeof(*v));v->now_ms=rd(JDY_BASE,4);
    v->lease_valid=(uint32_t)(v->now_ms-renewed_ms)<250U;
    v->sensors_fresh=tof_running&&vl53l1x_array_ready(&joint_tof);
    uint16_t *dist[3]={&v->front_mm,&v->right_mm,&v->left_mm};
    for(unsigned i=0;i<3;i++){
        const tof_sample_t *t=&joint_tof.sample[i];unsigned offset=i?60U:0U;
        *dist[i]=t->distance_mm>offset?(uint16_t)(t->distance_mm-offset):0;
        if(!t->received||t->validity!=SENSOR_VALID||t->error!=SENSOR_OK||t->range_status!=0||
           (uint32_t)(rd(TOF_BASE,8)-t->timestamp_us)>=200000U)v->sensors_fresh=false;
    }
    v->front_sequence=joint_tof.sample[0].sequence;
    v->wheels_moving=(uint32_t)(v->now_ms-last_motion_ms)<20U;
    v->result_generation=started_generation;
    v->result=tof5_active(&mission)?B_SEG_RUNNING:mission.reason==T5_DISTANCE?B_SEG_DISTANCE:
        mission.reason==T5_OBSTACLE?B_SEG_OBSTACLE:B_SEG_FAILED;
}
/* 功能：提交本段有符号输出；effort为0..8192；握手成功true。 */
static bool commit(int32_t effort)
{
    if(effort<0||effort>8192||hw_sequence==UINT32_MAX)return false;
    for(unsigned i=0;i<4;i++){
        int32_t value=effort;
        if(mission.raw_invert==10&&i<2)value=-value;
        wr(MOTOR_BASE,0x50+i*4,(uint32_t)value);
    }
    wr(MOTOR_BASE,0x10,++hw_sequence);wr(MOTOR_BASE,0x14,UINT32_C(0x4d4f5633));
    return (rd(MOTOR_BASE,4)&3U)==1U&&rd(MOTOR_BASE,0x44)==hw_sequence;
}
/* 功能：将毫米换算为名义编码器计数；mm为0..6000；返回向上取整计数。 */
static uint32_t counts(uint32_t mm)
{return (uint32_t)(((uint64_t)mm*5268945U+999999U)/1000000U);}
/* 功能：执行状态机已批准的新段；无参数；成功true，失败不再试。 */
static bool start_segment(void)
{
    uint32_t target=planner.phase==B_APPROACH?counts(6000):
        planner.phase==B_SHIFT_RIGHT?counts(geometry.shift_mm):counts(planner.pass_mm);
    if(planner.phase==B_PASS&&forward_counts+target>counts(6000))return false;
    tof5_init(&mission);mission.raw_invert=planner.phase==B_SHIFT_RIGHT?10:12;
    mission.target_counts=target;
    if(!tof5_offset(&mission,mission.raw_invert==10?60:0))return false;
    tof5_input_t v;inputs(&v);
    if(!tof5_begin(&mission,&v,sequence))return false;
    started_generation=planner.generation;
    for(unsigned c=0;c<4;c++)stall_stamp[c]=v.now_ms;
    return true;
}
/* 功能：每10ms执行本地保护/静止检查/分段切换；无参数/返回，无I2C调用。 */
static void control_tick(void)
{
    uint32_t now=rd(JDY_BASE,4);
    if((uint32_t)(now-last_tick)<10U)return;
    last_tick=now;tof5_input_t v;inputs(&v);
    for(unsigned c=0;c<4;c++){
        if((uint32_t)v.position[c]!=last_position[c]){
            last_position[c]=(uint32_t)v.position[c];stall_stamp[c]=last_motion_ms=now;
        }
    }
    if(!bypass_active(&planner))return;
    for(unsigned c=0;c<4;c++){
        if(rd(MOTOR_BASE,0x30+channel_to_bit[c]*4)!=illegal_start[c]||
           (mission.effort>0&&(uint32_t)(now-stall_stamp[c])>=1000U)){
            halt(T5_ENCODER);return;
        }
    }
    if(!v.drive_ok){halt(T5_DRIVE);return;}
    if(tof5_active(&mission)){
        mission.renewed_ms=renewed_ms;tof5_step(&mission,&v);
        if(!tof5_active(&mission)&&mission.raw_invert==12)forward_counts+=(uint32_t)(mission.progress>0?mission.progress:0);
    }
    bypass_input_t in;planning_inputs(&in);bypass_step(&planner,&in);
    if(!bypass_active(&planner)){halt(planner.phase==B_DONE?T5_DISTANCE:T5_DRIVE);return;}
    if(planner.generation!=started_generation){
        /* 换向只能发生在FSM已经验证连续静止之后；段开始先保持零输出。 */
        if(!start_segment()){halt(T5_DRIVE);return;}
    }
    if(!commit(tof5_active(&mission)?mission.effort:0))halt(T5_DRIVE);
}
/* 该文件由离线工具从已验证的联合测距I2C适配派生，保留X2/X3/X4，原驱动不改。 */
#include "tof5_bus.inc"
/* 功能：停机初始化前向ToF；无参数；成功true，失败false；不会启动电机。 */
static bool tof_start(void)
{
    if(bypass_active(&planner))return false;
    tof_stop();wr(TOF_BASE,0x10,256);
    const sensor_bus_t bus={.i2c_read=tof_read,.i2c_write=tof_write,.xshut=tof_shutdown};
    const tof_config_t cfg={.count=3,.address7={0x31,0x32,0x33},.distance_mode=TOF5_DISTANCE_MODE,
        .budget_ms=50,.period_ms=100,.fresh_us=200000,.io_mode=TOF_IO_AVDD};
    tof_running=vl53l1x_array_start(&joint_tof,&bus,&cfg,rd(TOF_BASE,8))==SENSOR_PENDING;
    return tof_running;
}
/* 功能：启动一次完整绕障；seq为新的非零序号；成功true。 */
static bool begin(uint32_t seq)
{
    uint32_t now=rd(JDY_BASE,4);
    if(bypass_active(&planner)||!configured||!seq||seq<=sequence||
       (uint32_t)(now-last_motion_ms)<300U)return false;
    bypass_input_t in;planning_inputs(&in);in.lease_valid=true;
    if(!bypass_begin(&planner,&geometry,&in))return false;
    sequence=seq;renewed_ms=now;forward_counts=0;
    tof5_init(&mission);
    for(unsigned c=0;c<4;c++)illegal_start[c]=rd(MOTOR_BASE,0x30+channel_to_bit[c]*4);
    wr(MOTOR_BASE,8,1);wr(JDY_BASE,12,1);wr(JDY_BASE,12,2);
    if(!commit(0)){halt(T5_DRIVE);return false;}
    wr(JDY_BASE,16,seq);
    if(!start_segment()){halt(T5_DRIVE);return false;}
    return true;
}
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
    p=text(p,"B1 V=1 DM=2 PROFILE=741");p=field(p," PHASE=",planner.phase);p=field(p," REASON=",planner.reason);
    p=field(p," STATE=",mission.state);p=field(p," WHY=",mission.reason);p=field(p," PWM=",mission.effort);
    p=field(p," COUNT=",mission.progress>0?(uint32_t)mission.progress:0);p=field(p," TARGET=",mission.target_counts);
    p=field(p," FRONT=",v.front_mm);p=field(p," RIGHT=",v.right_mm);p=field(p," LEFT=",v.left_mm);
    p=field(p," VALID=",v.sensors_fresh);p=field(p," SEQ=",sequence);p=field(p," GEN=",planner.generation);
    p=field(p," RUN=",tof_running);p=field(p," READY=",vl53l1x_array_ready(&joint_tof));p=field(p," ACTIVE=",rd(MOTOR_BASE,4)&1U);
    p=field(p," COMPLETE=",planner.phase==B_DONE&&planner.pass_mm>0);
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
/* 功能：只接受绕障命令白名单；s为非空NUL结尾字符串；无返回。 */
static void command(const char *s)
{
    uint32_t n;
    if(!strcmp(s,"bypass")||!strcmp(s,"tof")){status();return;}
    if(!strcmp(s,"stop")){halt(T5_CANCEL);reply("B1 STOP=1\r\n");return;}
    if(!strcmp(s,"tof stop")){halt(T5_SENSOR);tof_stop();reply("TOF STOP=1\r\n");return;}
    if(!strcmp(s,"tof start")){reply(tof_start()?"TOF START=1\r\n":"TOF START=0\r\n");return;}
    if(!strcmp(s,"bypass arm")){
        if(bypass_active(&planner)){reply("B1 ARM=0\r\n");return;}
        halt(T5_CANCEL);wr(MOTOR_BASE,12,0x5f);wr(MOTOR_BASE,0x74,0);
        configured=rd(MOTOR_BASE,12)==0x5f&&rd(MOTOR_BASE,0x74)==0;
        reply(configured?"B1 ARM=1\r\n":"B1 ARM=0\r\n");return;
    }
    if(!strncmp(s,"bypass start ",13)&&number(s+13,&n)){
        reply(begin(n)?"B1 START=1\r\n":"B1 START=0\r\n");return;
    }
    if(!strncmp(s,"bypass keep ",12)&&number(s+12,&n)){
        control_tick();
        if(bypass_active(&planner)&&n>sequence&&n){
            if((uint32_t)(rd(JDY_BASE,4)-renewed_ms)>=250U){halt(T5_LINK);return;}
            sequence=n;renewed_ms=rd(JDY_BASE,4);wr(JDY_BASE,16,n);
        }
        return;
    }
    halt(T5_CANCEL);reply("B1 REJECTED\r\n");
}
/* 功能：处理UART字节；c为0..255、now毫秒；无返回，波浪号可中断任何半包。 */
static void receive(uint8_t c,uint32_t now)
{
    rx_stamp=now;
    if(c=='~'){halt(T5_CANCEL);length=0;dropping=false;return;}
    if(c=='\r')return;
    if(c=='\n'){if(!dropping&&length){line[length]=0;command(line);}length=0;dropping=false;return;}
    if(c<32||c>126||length>=sizeof(line)-1){halt(T5_LINK);length=0;dropping=true;return;}
    if(!dropping)line[length++]=(char)c;
}
/* 功能：裸机主循环；无参数、正常不返回；上电不初始化传感器、不启动运动。 */
int main(void)
{
    bypass_init(&planner);tof5_init(&mission);halt(T5_CANCEL);wr(TOF_BASE,12,0);
    if(rd(JDY_BASE,0)!=UINT32_C(0x45535001)||rd(MOTOR_BASE,0)!=UINT32_C(0x4d465703)||
       rd(MOTOR_BASE,0x70)!=UINT32_C(0x434c5031))for(;;){}
    for(;;){
        control_tick();uint32_t now=rd(JDY_BASE,4),us=rd(TOF_BASE,8);
        if(rd(JDY_BASE,0x1c))halt(T5_LINK);
        if(length&&(uint32_t)(now-rx_stamp)>=1000U){halt(T5_LINK);length=0;dropping=true;}
        if(tof_running&&(uint32_t)(us-last_tof)>=1000U){
            last_tof=us;(void)vl53l1x_array_step(&joint_tof,us);
            if(joint_tof.error<0){halt(T5_SENSOR);tof_stop();}
        }
        control_tick();uint32_t c=rd(JDY_BASE,0x14);
        if(!(c&0x100U))receive((uint8_t)c,rd(JDY_BASE,4));
        if(tail!=head&&!(rd(JDY_BASE,0x18)&0x100U)){
            wr(JDY_BASE,0x18,(uint8_t)tx[tail]);tail=(tail+1U)%sizeof(tx);
        }
    }
}
