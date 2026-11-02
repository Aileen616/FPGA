#include "maze_tof.h"
static mt_t maze_shadow; /* 静态与运动共用的导航上下文，不隐式切换模式。 */
static mt_input_t maze_input;
static uint32_t maze_tick_ms,maze_ticks,maze_max_us;
static bool maze_initialized;
static uint8_t maze_driver_reason; /* 原控制层停车原因，独立于导航首个故障。 */
#include "maze_drive.inc"
#include "maze_robust.inc"
#include "maze_mapping_board.inc"
/* 核心token与捕获token分离，防止新任务接受旧任务的在途图像。 */
static uint32_t maze_token,maze_core_token,maze_capture_token,maze_request_ms,maze_vision_seq;
static uint32_t maze_consumed,maze_rejected,maze_first_capture;
static uint8_t maze_probe,maze_probe_value,maze_probe_count,maze_probe_result,maze_coverage;
static int32_t maze_probe_front,maze_probe_yaw;
static uint32_t maze_probe_stationary;
/* 功能：注销观察请求；无参数；无返回，不修改已发布图像时间。 */
static void maze_vision_cancel(void)
{endpoint_request_token=0;maze_core_token=maze_capture_token=0;maze_probe=0;}
/* 功能：保存驱动撤销原因；reason为原控制枚举；无返回，不覆盖已锁存原因。 */
static void maze_record_stop(unsigned reason)
{if(maze_owned&&!maze_driver_reason)maze_driver_reason=(uint8_t)reason;}
/* 功能：通用halt的撤销钩子；无参数；无返回，不递归写停止寄存器。 */
static void maze_halt(void)
{
    if(maze_owned||maze_probe){
        if(maze_shadow.state<MT_FAULT)mt_cancel(&maze_shadow);
        memset(&maze_shadow.output,0,sizeof(maze_shadow.output));maze_vision_cancel();
    }
    maze_owned=maze_trial=false;memset(maze_wheel,0,sizeof(maze_wheel));
}
/* 功能：建立600mm静态观察配置；无参数；返回初始化结果，不提交驱动。 */
static bool maze_reset(void)
{
    mt_config_t c;mt_defaults(&c);c.corridor_mm=600;c.goal_mm=170;c.front_goal_only=1;c.mapping_enabled=1;++maze_map_session;
    /* 未完成动态标定，不能将这些静态预算直接用于实车许可。 */
    c.fresh_ms=450;c.confirm_ms=180;c.error_mm=50;c.wait_ms=30000;c.offline_only=1;c.dynamics_verified=0;
    maze_vision_cancel();maze_probe_result=MT_VISION_UNKNOWN;maze_probe_count=0;
    memset(&maze_input,0,sizeof(maze_input));maze_ticks=maze_max_us=0;
    maze_driver_reason=0;maze_health_fault=0;maze_initialized=mt_init(&maze_shadow,&c);return maze_initialized;
}
/* 功能：保守映射微秒帧龄；now为毫秒、age_us为无符号年龄；返回采集毫秒。 */
static uint32_t maze_stamp(uint32_t now,uint32_t age_us)
{return now-age_us/1000U-(age_us%1000U!=0);}
/* 功能：建立停车视觉请求；core为导航token或探测编号；无返回，不复用旧发布。 */
static void maze_vision_begin(uint32_t core)
{
    ++maze_token;if(!maze_token)++maze_token;
    maze_core_token=core;maze_capture_token=maze_token;maze_request_ms=rd(JDY_BASE,4);
    maze_vision_seq=endpoint_sequence;maze_probe_count=0;maze_probe_value=MT_VISION_UNKNOWN;
    endpoint_request_token=maze_capture_token;
}
/* 功能：消费本请求的新鲜图像；now毫秒、coverage为充分视野；返回分类或UNKNOWN。 */
static uint8_t maze_vision_feed(uint32_t now,bool coverage)
{
    maze_input.vision_result=MT_VISION_UNKNOWN;maze_input.vision_coverage=0;
    if(!maze_capture_token||(int32_t)(endpoint_sequence-maze_vision_seq)<=0)return MT_VISION_UNKNOWN;
    maze_vision_seq=endpoint_sequence;
    if(!coverage||!endpoint_fresh(now)||endpoint_result_token!=maze_capture_token||
       (int32_t)(endpoint_stamp-maze_request_ms)<0||endpoint_last.count>1){++maze_rejected;return MT_VISION_UNKNOWN;}
    uint8_t value=endpoint_last.count==1?MT_VISION_POSITIVE:MT_VISION_NEGATIVE;
    maze_input.vision_seq=endpoint_sequence;maze_input.vision_capture_ms=endpoint_stamp;
    maze_input.vision_token=maze_core_token;maze_input.vision_coverage=1;maze_input.vision_result=value;
    ++maze_consumed;return value;
}
/* 功能：核对本次观察位置和姿态；无参数；静态一致且必要数据有效返回true。 */
static bool maze_probe_still(void)
{
    const mt_range_input_t *r=&maze_input.range[MT_FRONT];
    return r->valid&&(uint32_t)(maze_input.now_ms-r->stamp_ms)<200U&&maze_input.imu_valid&&
        (uint32_t)(maze_input.now_ms-maze_input.imu_stamp_ms)<50U&&
        auto_abs((int32_t)r->raw_mm-maze_probe_front)<=20U&&
        auto_abs(maze_input.yaw_md-maze_probe_yaw)<=1000U&&auto_abs(maze_input.yaw_rate_md_s)<=1000U;
}
/* 功能：无驱动验证终点接口；now为毫秒；无返回，不将探测成功伪装成导航到达。 */
static void maze_probe_tick(uint32_t now)
{
    if((rd(MOTOR_BASE,4)&1U)||robot_active(&robot)||autonomy_owned||pad_owner||
       (uint32_t)(now-maze_probe_stationary)>35000U){maze_probe_result=3;maze_vision_cancel();return;}
    if(!maze_probe_still()){
        /* 短暂失鲜或位置不一致只撤销本次取像，重新静止确认；不将单坏帧锁故障。
         * 旧在途图像携带旧token，恢复后必须重新获取，35秒总期限不重置。 */
        if(maze_probe==2){maze_vision_cancel();maze_probe=1;maze_probe_count=0;}
        maze_probe_front=maze_input.range[MT_FRONT].raw_mm;maze_probe_yaw=maze_input.yaw_md;
        maze_request_ms=now;return;
    }
    if(maze_probe==1){if((uint32_t)(now-maze_request_ms)<500U)return;maze_vision_begin(1);maze_probe=2;}
    uint8_t v=maze_vision_feed(now,maze_coverage!=0);if(v==MT_VISION_UNKNOWN)return;
    if(v!=maze_probe_value){maze_probe_value=v;maze_probe_count=0;maze_first_capture=endpoint_stamp;}
    if(maze_probe_count<255)++maze_probe_count;
    if(maze_probe_count>=2&&(uint32_t)(endpoint_stamp-maze_first_capture)>=180U){maze_probe_result=v;maze_vision_cancel();}
}
/* 功能：推进采样、导航、视觉和获准执行；无参数；无返回，静态分支不写电机。 */
static void maze_service(void)
{
    uint32_t now=rd(JDY_BASE,4),us=rd(TOF_BASE,8);
    if((uint32_t)(now-maze_tick_ms)<(maze_owned?10U:50U))return;
    maze_tick_ms=now;if(!maze_initialized&&!maze_reset())return;maze_input.now_ms=now;
    static const uint8_t slot[4]={0,1,3,2};
    for(unsigned d=0;d<4;d++){
        const tof_sample_t *s=&joint_tof.sample[slot[d]];mt_range_input_t *r=&maze_input.range[d];
        if(s->received&&s->sequence!=r->seq){
            r->seq=s->sequence;r->stamp_ms=maze_stamp(now,us-s->timestamp_us);r->raw_mm=s->distance_mm;
            r->valid=tof_running&&vl53l1x_array_ready(&joint_tof)&&s->validity==SENSOR_VALID&&s->error==SENSOR_OK&&s->range_status==0;
        }
    }
    if(imu.gyro_seq!=maze_input.imu_seq){
        maze_input.imu_seq=imu.gyro_seq;maze_input.imu_stamp_ms=maze_stamp(now,rd(IMU_BASE,8)-imu.gyro_us);
        maze_input.yaw_md=heading.reference_md-(int32_t)(heading.yaw_q16_md/65536);maze_input.yaw_rate_md_s=-heading.rate_mdps;
        maze_input.imu_valid=imu_started&&imu.valid&&imu.error==SENSOR_OK&&heading.calibrated&&!heading.calibrating&&
            heading.axis_confirmed&&heading.axis==2&&heading.sign==1&&!heading.fault&&heading.feedback_valid;
    }
    maze_encoder_sample(now);
    if(maze_owned&&maze_health_check())return;
    if(maze_probe)maze_probe_tick(now);
    else{
        if(maze_shadow.state==MT_GOAL){
            if(maze_core_token!=maze_shadow.token||!maze_capture_token)maze_vision_begin(maze_shadow.token);
            (void)maze_vision_feed(now,(maze_shadow.cfg.mapping_enabled||maze_drive_config.coverage_verified||endpoint_last.count==1)&&maze_shadow.dir==MT_FRONT);
        }else{
            maze_input.vision_result=MT_VISION_UNKNOWN;maze_input.vision_coverage=0;
            if(maze_capture_token)maze_vision_cancel();
            /* 后台有有效标志才触发核心确认；无标志或取像未知都不阻塞ToF探索。 */
            if(maze_shadow.cfg.mapping_enabled&&endpoint_fresh(now)&&endpoint_last.count<=1){
                maze_input.vision_seq=endpoint_sequence;maze_input.vision_capture_ms=endpoint_stamp;
                maze_input.vision_token=0;maze_input.vision_coverage=1;
                maze_input.vision_result=endpoint_last.count==1?MT_VISION_POSITIVE:MT_VISION_NEGATIVE;
            }
        }
        const mt_output_t *o=mt_tick(&maze_shadow,&maze_input);
        if(maze_owned){
            if(!maze_trial&&(uint32_t)(now-maze_started_ms)>=120000U&&o->state<MT_FAULT){
                maze_shadow.state=MT_FAULT;maze_shadow.reason=MT_TIMEOUT;halt(T5_TIMEOUT);
            }else if(o->state>=MT_FAULT)halt(T5_CANCEL);
            else{
                if(o->hardware_permitted||maze_trial)maze_mix(o);else memset(maze_wheel,0,sizeof(maze_wheel));
                if(!maze_commit())halt(T5_DRIVE);
            }
        }
    }
    ++maze_ticks;uint32_t elapsed=rd(TOF_BASE,8)-us;if(elapsed>maze_max_us)maze_max_us=elapsed;
}
/* 功能：显式申请运动；seq非零递增；成功true，未标定拒绝且不写外设。 */
static bool maze_begin(uint32_t seq,bool trial)
{
    if((!trial&&!maze_drive_ready())||imu_lease_seq==UINT32_MAX||!irq_armed||!imu.phase||!bmi088_ready(&imu.core)||!seq||seq<=sequence||maze_owned||maze_probe||pad_owner||pad_cal_stage||autonomy_owned||
       robot_active(&robot)||bypass_active(&planner)||(rd(MOTOR_BASE,4)&1U)||!imu_ready())return false;
    /* 当前候选无串口校准解锁；只有离线夹具配置已验证预算才可覆盖执行路径。 */
    if(!trial&&(!maze_shadow.cfg.dynamics_verified||maze_shadow.cfg.offline_only))return false;
    legacy_invalidate();maze_health_reset();
    /* 停机启用UART持续确认并核对新RTL能力；旧码流无法通过此门禁。 */
    wr(JDY_BASE,12,16);wr(UINT32_C(0x20008000),12,16);
    if(!(rd(JDY_BASE,8)&4U)){wr(JDY_BASE,12,32);wr(UINT32_C(0x20008000),12,32);return false;}
    uint8_t previous_state=maze_shadow.state,previous_phase=maze_shadow.map_phase;
    int16_t previous_target=maze_shadow.target_node,previous_travel=maze_shadow.travel_target;
    if(maze_requested_target>=0&&!mt_request_target(&maze_shadow,(uint16_t)maze_requested_target)){
        wr(JDY_BASE,12,32);wr(UINT32_C(0x20008000),12,32);return false;
    }
    if(!mt_start(&maze_shadow,MT_FRONT,rd(JDY_BASE,4))){maze_shadow.state=previous_state;maze_shadow.map_phase=previous_phase;maze_shadow.target_node=previous_target;maze_shadow.travel_target=previous_travel;wr(JDY_BASE,12,32);wr(UINT32_C(0x20008000),12,32);return false;}
    wr(IMU_BASE,0x64,++imu_lease_seq);wr(IMU_BASE,0x60,1);
    sequence=seq;maze_started_ms=rd(JDY_BASE,4);maze_owned=true;maze_trial=trial;maze_map_trial_mode=trial;pi_configured=false;
    memset(maze_wheel,0,sizeof(maze_wheel));wr(MOTOR_BASE,0x74,0);wr(MOTOR_BASE,12,0x5f);
    wr(MOTOR_BASE,8,1);wr(JDY_BASE,12,1);wr(JDY_BASE,12,2);local_lease=0;
    if(!maze_commit()){halt(T5_DRIVE);return false;}return true;
}
/* 功能：保留已标定生产入口；seq非零递增；未标定时仍拒绝。 */
static bool maze_start(uint32_t seq) {return maze_begin(seq,false);}
/* 功能：处理迷宫诊断和显式请求；s不可空；返回是否命中，不提供动态许可解锁。 */
static bool maze_command(const char *s)
{
    if(strncmp(s,"maze",4))return false;
    if(!maze_initialized)(void)maze_reset();
    uint32_t d;char b[512],*p;
    if(!strcmp(s,"maze stop")){
        if(maze_owned)halt(T5_CANCEL);
        mt_cancel(&maze_shadow);maze_vision_cancel();reply("MZ STOP=1 MOTOR=0\r\n");return true;
    }
    if(!strncmp(s,"maze trial ",11)&&number(s+11,&d)){
        reply(maze_begin(d,true)?"MZ TRIAL=1\r\n":"MZ TRIAL=0\r\n");return true;
    }
    if(!strncmp(s,"maze start ",11)&&number(s+11,&d)){reply(maze_start(d)?"MZ START=1\r\n":"MZ START=0\r\n");return true;}
    if(!strcmp(s,"maze observe")||!strncmp(s,"maze goal ",10)){
        bool safe=!maze_owned&&!maze_probe&&!pad_owner&&!pad_cal_stage&&!autonomy_owned&&!robot_active(&robot)&&!(rd(MOTOR_BASE,4)&1U);
        if(!strcmp(s,"maze observe")){
            bool ok=safe&&maze_reset()&&mt_start(&maze_shadow,MT_FRONT,rd(JDY_BASE,4));
            reply(ok?"MZ OBSERVE=1 MOTOR=0\r\n":"MZ OBSERVE=0 MOTOR=0\r\n");return true;
        }
        if(safe&&number(s+10,&d)&&d<=1&&imu_ready()&&endpoint_run){
            mt_cancel(&maze_shadow);maze_vision_cancel();maze_coverage=(uint8_t)d;maze_probe_result=MT_VISION_UNKNOWN;maze_probe_count=0;
            maze_probe_front=joint_tof.sample[0].distance_mm;maze_probe_yaw=heading.reference_md-(int32_t)(heading.yaw_q16_md/65536);
            maze_request_ms=maze_probe_stationary=rd(JDY_BASE,4);maze_probe=1;reply("MZ GOAL=1 MOTOR=0\r\n");return true;
        }
        reply("MZ GOAL=0 MOTOR=0\r\n");return true;
    }
    if(!strcmp(s,"maze ref")){
        p=field(text(b,"MZREF V=1 FIX=2")," DIR=",maze_shadow.dir);
        p=field(p," A=",maze_shadow.anchor_dir);p=field(p," AE=",maze_shadow.anchor_epoch);
        p=field(p," E=",maze_shadow.sensor[maze_shadow.anchor_dir].epoch);
        p=text(p," MM=");p=signed_number(p,maze_shadow.sensor[maze_shadow.anchor_dir].mm);
        p=text(p," EDGE=");p=signed_number(p,maze_shadow.edge_anchor);
        p=text(p," TARGET=");p=signed_number(p,maze_shadow.align_target);
        p=field(p," PM=",maze_shadow.pending_mask);p=field(p," CE=",maze_shadow.cross_epoch);
        p=field(p," CX=",maze_shadow.sensor[maze_shadow.cross_dir].epoch);
        p=text(p,"\r\n");*p=0;reply(b);return true;
    }
    if(!strcmp(s,"maze io")){
        p=field(text(b,"MZIO V=1")," OWN=",maze_owned);p=field(p," VERIFIED=",maze_drive_ready());p=field(p," COMMITS=",maze_commits);
        p=field(p," STOPWHY=",maze_driver_reason);p=field(p," HEALTH=",maze_health_fault);p=field(p," IQFAULT=",imu_quality_fault);p=field(p," IBAD=",maze_imu_bad);p=field(p," LINKBAD=",maze_link_bad);p=field(p," EXCLUDED=",maze_shadow.excluded_deadends);p=field(p," EXDIRS=",maze_shadow.excluded_mask);p=field(p," TRIAL=",maze_trial);p=field(p," TLIMIT=",0);p=field(p," FRONTGOAL=",maze_shadow.cfg.front_goal_only);p=field(p," PROBE=",maze_probe);p=field(p," RESULT=",maze_probe_result);p=field(p," TOKEN=",maze_capture_token);
        p=field(p," CAPTOKEN=",endpoint_result_token);p=field(p," CONSUMED=",maze_consumed);p=field(p," REJECTED=",maze_rejected);p=field(p," FRAMES=",maze_probe_count);
    }else if(!strcmp(s,"maze status")){
        p=field(text(b,"MZ V=15")," STATIC=",!maze_owned);p=text(p," WIDTH=600 LENGTH=260 BODYW=250");p=field(p," MOTOR=",rd(MOTOR_BASE,4)&1U);
        p=field(p," STATE=",maze_shadow.state);p=field(p," WHY=",maze_shadow.reason);p=field(p," TICKS=",maze_ticks);p=field(p," MAXUS=",maze_max_us);
        p=field(p," PERMIT=",maze_shadow.output.hardware_permitted);p=field(p," IMU=",maze_input.imu_valid);p=field(p," ISEQ=",maze_input.imu_seq);p=field(p," IAGE=",maze_input.now_ms-maze_input.imu_stamp_ms);
        p=text(p," FX=");p=signed_number(p,maze_shadow.output.forward_mm_s);p=text(p," RY=");p=signed_number(p,maze_shadow.output.right_mm_s);
        p=field(p," FRESH=",maze_shadow.cfg.fresh_ms);p=field(p," CONFIRM=",maze_shadow.cfg.confirm_ms);
    }else if(!strncmp(s,"maze read ",10)&&number(s+10,&d)&&d<4){
        const mt_range_input_t *r=&maze_input.range[d];const mt_sensor_t *v=&maze_shadow.sensor[d];
        p=field(text(b,"MZR D="),"",d);p=field(p," RAW=",r->raw_mm);p=field(p," OFFSET=",maze_shadow.cfg.inset_mm[d]);p=text(p," NET=");p=signed_number(p,v->mm);
        p=field(p," VALID=",r->valid);p=field(p," SEQ=",r->seq);p=field(p," AGE=",maze_input.now_ms-r->stamp_ms);p=field(p," ACCEPT=",v->accepted);p=field(p," AAGE=",maze_input.now_ms-v->stamp_ms);
        p=field(p," PENDING=",v->pending_count);p=field(p," EPOCH=",v->epoch);
    }else{reply("MZ REJECT MOTOR=0\r\n");return true;}
    p=text(p,"\r\n");*p=0;reply(b);return true;
}
