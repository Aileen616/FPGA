#include <stdio.h>
#include <stdlib.h>
#define TOF5_HOST_TEST
#define main firmware_entry
#include "tof5_main.c"
#undef main
static uint32_t mem[4][256]; /* 模拟四页MMIO，测试时独占。 */
static unsigned checks,commits;
#define CHECK(x) do { ++checks;if(!(x)){printf("FAIL %u %s\n",(unsigned)__LINE__,#x);exit(1);} }while(0)
/* 功能：读取模拟寄存器；base/off为合法地址；返回32位值。 */
uint32_t tof5_test_read(uint32_t base,unsigned off){return mem[(base-JDY_BASE)/4096][off/4];}
/* 功能：模拟握手；base/off/v为合法写入；无返回。 */
void tof5_test_write(uint32_t base,unsigned off,uint32_t v)
{
    unsigned page=(base-JDY_BASE)/4096;mem[page][off/4]=v;
    if(base==MOTOR_BASE&&off==8&&(v==4||v==1))mem[1][1]=0;
    if(base==JDY_BASE&&off==12)mem[0][2]=v==2?1U:0U;
    if(base==MOTOR_BASE&&off==0x14){commits++;mem[1][1]=1;mem[1][0x44/4]=mem[1][4];}
}
/* 功能：注入三路健康帧；now为毫秒；无返回。 */
static void healthy(uint32_t now)
{
    mem[2][2]=now*1000U;imu_started=true;imu.valid=true;imu.error=SENSOR_OK;
    heading.stamp_us=now*1000U;heading.feedback_valid=true;
    mem[0][1]=now;mem[3][2]=now*1000U;tof_running=true;joint_tof.state=8;
    for(unsigned i=0;i<3;i++){
        tof_sample_t *t=&joint_tof.sample[i];t->received=true;t->validity=SENSOR_VALID;
        t->error=SENSOR_OK;t->range_status=0;t->raw_status=9;
        t->distance_mm=2000;t->timestamp_us=now*1000U;t->sequence=now/100;
    }
}
/* 功能：注入串口命令；s非空；无返回，调用真实解析器。 */
static void send(const char *s){while(*s)receive((uint8_t)*s++,mem[0][1]);}
/* 功能：重新初始化模拟平台；无参数/返回。 */
static void setup(void)
{
    memset(mem,0,sizeof(mem));memset(&imu,0,sizeof(imu));heading_init(&heading);
    heading.calibrated=true;heading.axis_confirmed=true;heading.axis=2;heading.sign=1;
    irq_missed_acc=irq_missed_gyro=irq_races=0;memset(&joint_tof,0,sizeof(joint_tof));
    memset(last_position,0,sizeof(last_position));memset(stall_stamp,0,sizeof(stall_stamp));
    bypass_init(&planner);tof5_init(&mission);configured=tof_running=false;
    sequence=renewed_ms=started_generation=last_motion_ms=forward_counts=0;
    hw_sequence=last_tick=head=tail=length=0;dropping=false;halt(T5_CANCEL);
    send("bypass start 1\n");CHECK(!bypass_active(&planner));
    send("bypass arm\n");CHECK(configured);healthy(500);control_tick();
    send("bypass start 1\n");CHECK(planner.phase==B_APPROACH&&tof5_active(&mission));
    head=tail=0;
}
/* 功能：推进编码器与定时续期；ms当前时间、move选择是否推进四轮；无返回。 */
static void tick(uint32_t ms,bool move)
{
    healthy(ms);
    if(planner.phase==B_APPROACH&&ms>=1500)joint_tof.sample[0].distance_mm=650;
    if(planner.phase==B_WAIT_FRONT)joint_tof.sample[0].distance_mm=600;
    unsigned selected=bypass_tof_channel(planner.phase);
    for(unsigned i=0;i<3;i++)if(i!=selected){joint_tof.sample[i].validity=SENSOR_INVALID;joint_tof.sample[i].distance_mm=100;}
    if(move&&tof5_active(&mission))for(unsigned c=0;c<4;c++){
        unsigned reg=(0x20+channel_to_bit[c]*4)/4;
        mem[1][reg]+=(mission.raw_invert&(1U<<c))?0U-20U:20U;
    }
    if(ms%100==0){char cmd[48];snprintf(cmd,sizeof(cmd),"bypass keep %u\n",sequence+1);send(cmd);}
    control_tick();head=tail=0;
}
/* 功能：核对换向门禁、重复帧和三方向限幅；无参数/返回，失败终止。 */
static void fsm_tests(void)
{
    bypass_fsm_t f; bypass_input_t v={0};bypass_init(&f);
    v.lease_valid=v.sensors_fresh=v.imu_ready=true;v.front_mm=v.right_mm=v.left_mm=2000;
    CHECK(bypass_begin(&f,&geometry,&v));
    v.result_generation=f.generation;v.result=B_SEG_OBSTACLE;v.now_ms=100;v.front_mm=600;
    bypass_step(&f,&v);CHECK(f.phase==B_WAIT_FRONT);
    v.now_ms=299;bypass_step(&f,&v);CHECK(f.phase==B_WAIT_FRONT);
    v.now_ms=300;bypass_step(&f,&v);CHECK(f.phase==B_SHIFT_RIGHT);
    v.now_ms=400;v.result_generation=f.generation;v.result=B_SEG_DISTANCE;v.front_mm=2000;
    bypass_step(&f,&v);CHECK(f.phase==B_WAIT_SHIFT);
    for(unsigned t=410;t<600;t+=10){v.now_ms=t;++v.front_sequence;bypass_step(&f,&v);}
    CHECK(f.clear_samples==0);v.now_ms=600;bypass_step(&f,&v);CHECK(f.phase==B_WAIT_SHIFT);
    v.now_ms=610;++v.front_sequence;bypass_step(&f,&v);CHECK(f.clear_samples==1);
    v.now_ms=620;bypass_step(&f,&v);CHECK(f.clear_samples==1);
    v.now_ms=630;v.wheels_moving=true;++v.front_sequence;bypass_step(&f,&v);CHECK(f.clear_samples==0);
    v.wheels_moving=false;v.now_ms=830;bypass_step(&f,&v);CHECK(f.clear_samples==0);
    for(unsigned i=0;i<3;i++){v.now_ms+=100;++v.front_sequence;bypass_step(&f,&v);}
    CHECK(f.phase==B_PASS);
    v.result_generation=f.generation-1;bypass_step(&f,&v);CHECK(f.phase==B_PASS);
    v.result_generation=f.generation;bypass_step(&f,&v);CHECK(f.phase==B_WAIT_PASS);
    v.now_ms+=200;bypass_step(&f,&v);
    for(unsigned i=0;i<3;i++){v.now_ms+=100;++v.left_sequence;bypass_step(&f,&v);}
    CHECK(f.phase==B_RETURN_LEFT);v.result_generation=f.generation;bypass_step(&f,&v);
    CHECK(f.phase==B_WAIT_END);v.now_ms+=200;bypass_step(&f,&v);CHECK(f.phase==B_DONE&&f.returned);
    for(unsigned d=0;d<3;d++)for(int ratio=-100;ratio<=100;ratio++)for(int e=0;e<=19660;e+=1966){
        int32_t out[4];CHECK(bypass_mix(d,e,ratio,out));
        for(unsigned c=0;c<4;c++)CHECK(abs(out[c])<=19660);
        int32_t yaw=out[0]+out[1]-out[2]-out[3];CHECK(ratio==0?yaw==0:ratio>0?yaw>=0:yaw<=0);
    }
    for(unsigned phase=B_APPROACH;phase<B_DONE;phase++){
        bypass_init(&f);CHECK(bypass_begin(&f,&geometry,&v));f.phase=(bypass_phase_t)phase;
        v.imu_ready=false;bypass_step(&f,&v);CHECK(f.phase==B_FAULT&&f.reason==B_IMU);v.imu_ready=true;
    }
}
#include "test_direction_policy.inc"
/* 功能：验证真实执行器/状态机/MMIO连接；无参数；全部通过0，否则退出1。 */
int original_platform_tests(void)
{
    setup();CHECK(tof_shutdown(0,0,true)==0&&mem[3][3]==2);
    CHECK(tof_shutdown(0,1,true)==0&&mem[3][3]==6);
    CHECK(tof_shutdown(0,2,true)==0&&mem[3][3]==14);CHECK(tof_shutdown(0,3,true)<0);
    unsigned seen=0;
    for(unsigned t=510;t<20000&&bypass_active(&planner);t+=10){
        tick(t,true);seen|=1U<<planner.phase;
        if(planner.phase==B_SHIFT_RIGHT&&mission.effort){
            CHECK((int32_t)mem[1][0x50/4]<0&&(int32_t)mem[1][0x54/4]<0);
            CHECK((int32_t)mem[1][0x58/4]>0&&(int32_t)mem[1][0x5c/4]>0);
        }
        if(planner.phase==B_RETURN_LEFT&&mission.effort){
            CHECK((int32_t)mem[1][0x50/4]>0&&(int32_t)mem[1][0x54/4]>0);
            CHECK((int32_t)mem[1][0x58/4]<0&&(int32_t)mem[1][0x5c/4]<0);
        }
        CHECK(heading.reference_md==0);
        for(unsigned i=0;i<4;i++)CHECK(abs((int32_t)mem[1][(0x50+i*4)/4])<=19660);
        if((planner.phase==B_APPROACH||planner.phase==B_PASS)&&mission.effort)
            for(unsigned i=0;i<4;i++)CHECK((int32_t)mem[1][(0x50+i*4)/4]>0);
        if(planner.phase==B_WAIT_FRONT||planner.phase==B_WAIT_SHIFT||planner.phase==B_WAIT_PASS||planner.phase==B_WAIT_END)
            for(unsigned i=0;i<4;i++)CHECK(mem[1][(0x50+i*4)/4]==0);
    }
    CHECK(planner.phase==B_DONE&&planner.pass_mm>0&&planner.returned);CHECK(right_settled_counts>=counts(450));CHECK(!mem[1][1]&&!mem[0][2]);
    CHECK(seen&(1U<<B_RETURN_LEFT));
    CHECK(!heading.enabled&&!heading.running&&!mem[2][0x60/4]);
    CHECK((seen&((1U<<B_SHIFT_RIGHT)|(1U<<B_PASS)))==((1U<<B_SHIFT_RIGHT)|(1U<<B_PASS)));
    for(unsigned i=0;i<3;i++){
        setup();healthy(510);joint_tof.sample[i].validity=SENSOR_INVALID;control_tick();
        CHECK(i==0?(planner.phase==B_FAULT&&!mem[1][1]&&!mem[0][2]):planner.phase==B_APPROACH);
    }
    setup();healthy(750);send("bypass keep 2\n");CHECK(planner.phase==B_FAULT&&!mem[1][1]);
    setup();send("~");CHECK(planner.reason==B_CANCEL&&!mem[1][1]);
    setup();send("tfr5 start 50\n");CHECK(planner.phase==B_FAULT&&!mem[1][1]);
    setup();for(unsigned t=510;t<=1600&&bypass_active(&planner);t+=10)tick(t,false);
    CHECK(planner.phase==B_FAULT&&mission.reason==T5_ENCODER&&!mem[1][1]);
    setup();healthy(510);mem[1][0x30/4]=1;control_tick();CHECK(mission.reason==T5_ENCODER&&!mem[1][1]);
    setup();healthy(510);joint_tof.sample[2].timestamp_us=310000;control_tick();CHECK(planner.phase==B_APPROACH);
    setup();healthy(510);heading.stamp_us-=20000;control_tick();CHECK(planner.reason==B_IMU&&!mem[1][1]);
    setup();healthy(510);irq_races=1;control_tick();CHECK(planner.reason==B_IMU&&!mem[1][1]);
    setup();healthy(510);heading.yaw_q16_md=21000LL*65536;control_tick();CHECK(planner.reason==B_IMU&&!mem[1][1]);
    setup();send("imu start\n");CHECK(planner.phase==B_APPROACH); /* 活动时拒绝重置。 */
    fsm_tests();direction_policy_tests();
    printf("HOST_TEST_PASS tof_bypass_platform checks=%u\n",checks);return 0;
}

#include "test_route_priority.inc"
