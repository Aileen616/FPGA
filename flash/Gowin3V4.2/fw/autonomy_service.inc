/* 整机接入层；依赖tof5_main的MMIO、传感器、robot PI、故障记录及序号。
 * 速度通过真实编码器cps估计；输出只走原CLS1四轮速度PI，不把PWM换算为车速。
 */
#include "autonomy_board_config.h"
static robot_mission_t auto_output; /* 规范FL/FR/RL/RR速度目标，counts/s。 */
static uint32_t auto_leg_base[4],auto_leg_goal,auto_chunk_end,auto_phase_ms,auto_quiet_ms;
static uint32_t auto_shift_counts,auto_progress,auto_started_ms,auto_illegal[4];
static uint32_t auto_last_pos[4],auto_stall_ms[4];
static uint8_t auto_task;
static bool auto_leg_wait,auto_switch_wait,auto_rotation_verified;
static int32_t auto_reference_md;
static uint32_t auto_clear_sequence;
static uint8_t auto_clear_frames;

/* 功能：初始化生产配置；无参数/返回，不写外设、不标定、不启动。 */
static void autonomy_init(void)
{
    ap_config_t c;ap_defaults(&c);
    c.brake_model_verified=AUTO_BRAKE_VERIFIED!=0;c.brake_decel_mm_s2=AUTO_BRAKE_DECEL_MM_S2;
    c.offset_mm[0]=0;c.offset_mm[1]=60;c.offset_mm[2]=60;c.offset_mm[3]=0;
    c.offset_known_mask=AUTO_OFFSET_KNOWN_MASK;
    (void)ap_init(&autonomy,&c);autonomy_owned=false;
    auto_rotation_verified=AUTO_ROTATION_ENVELOPE_VERIFIED!=0;
}

/* 功能：撤销自主所有权；reason来自通用停机原因；无返回，不递归调用halt。 */
static void autonomy_halt(tof5_reason_t reason)
{
    if(!autonomy_owned)return;
    if(autonomy.state!=AP_FAULT&&autonomy.state!=AP_DONE){
        autonomy.state=reason==T5_DISTANCE?AP_DONE:AP_FAULT;
        autonomy.reason=reason==T5_DISTANCE?AP_OK:reason==T5_CANCEL?AP_CANCEL:reason==T5_ENCODER?AP_ENCODER:AP_DRIVE;
    }
    autonomy.speed_limit_mm_s=0;autonomy_owned=false;
    for(unsigned i=0;i<4;i++)auto_output.wheel_cps[i]=0;
}

/* 功能：取安全绝对值；v任意int32；返回uint32幅值，处理INT32_MIN。 */
static uint32_t auto_abs(int32_t v){return v<0?0U-(uint32_t)v:(uint32_t)v;}

/* 功能：编码器cps向上转换速度；cps为幅值；返回饱和到2001的mm/s供超速判定。 */
static uint16_t auto_mmps(uint32_t cps)
{
    uint64_t v=((uint64_t)cps*1000000U+5268944U)/5268945U;
    return v>2000U?2001U:(uint16_t)v;
}

/* 功能：构造策略真实输入；v不可空、mask为方向位、starting表示尚未使能；无返回。 */
static void autonomy_inputs(ap_input_t *v,uint8_t mask,bool starting)
{
    memset(v,0,sizeof(*v));v->now_ms=rd(JDY_BASE,4);v->direction_mask=mask;
    uint32_t us=rd(TOF_BASE,8);
    for(unsigned i=0;i<4;i++){
        const tof_sample_t *t=&joint_tof.sample[i];uint32_t age=us-t->timestamp_us;
        v->range[i]=(ap_range_t){.sequence=t->sequence,.timestamp_ms=v->now_ms-(age/1000U+(age%1000U!=0)),
            .raw_mm=t->distance_mm,.valid=tof_running&&vl53l1x_array_ready(&joint_tof)&&t->received&&
            t->validity==SENSOR_VALID&&t->error==SENSOR_OK&&t->range_status==0&&age<200000U};
    }
    uint32_t age=rd(IMU_BASE,8)-heading.stamp_us;
    v->imu_sequence=imu.gyro_seq;v->imu_timestamp_ms=v->now_ms-(age/1000U+(age%1000U!=0));
    v->imu_valid=imu_started&&imu.valid&&imu.error==SENSOR_OK&&heading.feedback_valid&&age<20000U;
    v->calibrated=heading.calibrated&&!heading.calibrating;
    v->axis_confirmed=heading.axis_confirmed&&heading.axis==2&&heading.sign==1;
    v->imu_hard_fault=heading.fault||imu.error<0;
    v->driver_ok=starting||((rd(MOTOR_BASE,4)&3U)==1U&&(rd(JDY_BASE,8)&3U)==1U&&rd(MOTOR_BASE,0x74)==1);
    v->encoder_reliable=true;
    wr(MOTOR_BASE,8,8); /* 只锁存快照，不清计数、不刷新驱动租约。 */
    uint32_t fastest=0;
    for(unsigned i=0;i<4;i++){
        unsigned bit=canonical_bit[i];uint32_t speed=auto_abs((int32_t)rd(MOTOR_BASE,0xc0+4*bit));
        if(speed>fastest)fastest=speed;
        if(!starting&&rd(MOTOR_BASE,0x30+4*bit)!=auto_illegal[i])v->encoder_event=true;
    }
    v->speed_mm_s=auto_mmps(fastest);
    /* 未有真实毛刺分类证据，任何新增未知非法AB仍报告给公共策略。 */
    v->encoder_event_classified_glitch=false;
}

/* 功能：返回可信缓存净距；i为0..3；返回mm，无效或过期返回0，不读取显示缓存。 */
static uint16_t auto_clearance(unsigned i)
{
    const ap_range_t *r=&autonomy.accepted[i];uint16_t offset=autonomy.config.offset_mm[i];
    if(!r->valid||!(autonomy.config.offset_known_mask&(1U<<i))||
       (uint32_t)(rd(JDY_BASE,4)-r->timestamp_ms)>=autonomy.config.range_age_ms)return 0;
    return r->raw_mm>offset?(uint16_t)(r->raw_mm-offset):0;
}

/* 功能：把车体平移/转向请求映射为需检查方向；f/l/t为counts/s；返回方向位。 */
static uint8_t auto_mask(int32_t f,int32_t l,int32_t t)
{
    if(t&&!f&&!l)return AP_ALL;
    return (uint8_t)((f>0?AP_FRONT:f<0?AP_REAR:0)|(l>0?AP_LEFT:l<0?AP_RIGHT:0));
}

/* 功能：按策略速度上限生成四轮闭环目标；f/l/t单位counts/s；无返回，未授权时全零。 */
static void auto_mix(int32_t f,int32_t l,int32_t t)
{
    int32_t w[4]={f-l-t,f+l+t,f+l-t,f-l+t};uint32_t peak=0;
    uint32_t cap=(uint32_t)((uint64_t)autonomy.speed_limit_mm_s*5268945U/1000000U);
    for(unsigned i=0;i<4;i++)if(auto_abs(w[i])>peak)peak=auto_abs(w[i]);
    for(unsigned i=0;i<4;i++)auto_output.wheel_cps[i]=!cap?0:peak>cap?(int32_t)((int64_t)w[i]*cap/peak):w[i];
}

/* 功能：提交本地健康控制帧；无参数；返回硬件是否接受，只有本函数续自主租约。 */
static bool auto_commit(void)
{
    if(!autonomy_owned)return false;
    if(!robot_commit(&auto_output))return false;
    renewed_ms=rd(JDY_BASE,4);return true;
}

/* 功能：记录新段起点；phase为任务三阶段、target为计数；无返回，累计真实编码器位置。 */
static void auto_leg(bypass_phase_t phase,uint32_t target)
{
    if(phase==B_SHIFT_RIGHT&&task3_vision.enabled){task3_vision.release_frames=0;task3_vision.block_latched=true;}
    planner.phase=phase;planner.generation++;started_generation=planner.generation;
    auto_leg_goal=target;auto_chunk_end=auto_progress=0;auto_leg_wait=true;auto_phase_ms=rd(JDY_BASE,4);
    auto_quiet_ms=auto_phase_ms;auto_clear_frames=0;auto_clear_sequence=joint_tof.sample[0].sequence;
    mission.raw_invert=phase==B_SHIFT_RIGHT?10:phase==B_RETURN_LEFT?5:12;
    mission.progress=0;mission.target_counts=target;mission.started_ms=auto_phase_ms;
    for(unsigned c=0;c<4;c++){
        mission.baseline[c]=rd(MOTOR_BASE,0x20+4*channel_to_bit[c]);
        unsigned bit=canonical_bit[c];uint32_t raw=rd(MOTOR_BASE,0x20+4*bit);
        auto_leg_base[c]=(5U&(1U<<bit))?0U-raw:raw;
    }
}

/* 功能：得到任务三本腿正向行程；无参数；返回是否计数一致，失败冻结原诊断。 */
static bool auto_leg_progress(void)
{
    int64_t sum=0;int32_t lo=INT32_MAX,hi=INT32_MIN;
    static const int8_t right[4]={1,-1,-1,1};
    for(unsigned i=0;i<4;i++){
        unsigned bit=canonical_bit[i];uint32_t n=rd(MOTOR_BASE,0x20+4*bit);
        if(5U&(1U<<bit))n=0U-n;
        uint32_t d=n-auto_leg_base[i];
        int sign=planner.phase==B_SHIFT_RIGHT?right[i]:planner.phase==B_RETURN_LEFT?-right[i]:1;
        if(sign<0)d=0U-d;
        int32_t step=(int32_t)d;sum+=step;if(step<lo)lo=step;if(step>hi)hi=step;
    }
    if(lo< -100||hi>40000||(int64_t)hi-lo>790){
        tof5_input_t trace;inputs(&trace);encoder_trace_count_fault(&trace,0);return false;
    }
    auto_progress=sum>0?(uint32_t)(sum/4):0;mission.progress=(int32_t)auto_progress;
    return true;
}

/* 功能：统一停机并保留策略原因；reason为策略枚举；无返回。 */
static void auto_fail(ap_reason_t reason)
{
    autonomy.state=AP_FAULT;autonomy.reason=reason;autonomy.speed_limit_mm_s=0;
    halt(reason==AP_ENCODER?T5_ENCODER:reason==AP_CANCEL||reason==AP_ESTOP?T5_CANCEL:T5_SENSOR);
}

/* 功能：准备新自主任务；seq新鲜非零、task为3..6；返回起步成功，拒绝不提交电机使能。 */
static bool autonomy_begin(uint32_t seq,uint8_t task)
{
    if(maze_owned||autonomy_owned||robot_active(&robot)||bypass_active(&planner)||!seq||seq<=sequence||
       (uint32_t)(rd(JDY_BASE,4)-last_motion_ms)<200U){start_reject_mask=SG_BUSY;return false;}
    if(task<3||task>6||(task==4&&!robot.point_count)){start_reject_mask=SG_CONFIG;return false;}
    if(!autonomy_initialized){autonomy_init();autonomy_initialized=true;}
    if(task==3&&task3_vision.enabled){
        vision_service();
        if(!vision_profile_verified||!vision_geometry_verified||!vision_depth_mm||
           !fv_fresh(&task3_vision,rd(JDY_BASE,4))||!task3_vision.target_visible){
            autonomy.reason=AP_TARGET;start_reject_mask=SG_CONFIG;return false;
        }
        task3_vision.seen_target=true;
    }
    uint8_t mask=AP_FRONT;
    if(task==4){robot_point_t p=robot.points[0];mask=p.x_mm>25?AP_FRONT:p.x_mm< -25?AP_REAR:p.y_mm>25?AP_LEFT:p.y_mm< -25?AP_RIGHT:AP_FRONT;}
    ap_input_t in;autonomy_inputs(&in,mask,true);
    if(!ap_begin(&autonomy,task,seq,&in)){
        start_reject_mask=autonomy.reason==AP_IMU?SG_IMU:autonomy.reason==AP_RANGE?SG_FRONT_INVALID:
            autonomy.reason==AP_BLOCKED||autonomy.reason==AP_NEAR?SG_FRONT_NEAR:SG_CONFIG;return false;
    }
    legacy_invalidate(); /* 接管前撤销旧蓝牙半包与手势，防止其超时回调误停自主任务。 */
    if(!robot_pi(4096)){autonomy.state=AP_FAULT;autonomy.reason=AP_DRIVE;start_reject_mask=SG_CONFIG;return false;}
    configured=true;sequence=seq;auto_task=task;auto_started_ms=in.now_ms;
    auto_reference_md=(int32_t)(heading.yaw_q16_md/65536);
    auto_rotation_verified=AUTO_ROTATION_ENVELOPE_VERIFIED!=0;
    auto_shift_counts=auto_progress=0;auto_switch_wait=false;forward_counts=right_settled_counts=0;
    memset(&auto_output,0,sizeof(auto_output));
    if(task!=3){
        robot_input_t r;robot_inputs(&r);
        robot.configured=true;memcpy(robot.offsets,autonomy.config.offset_mm,sizeof(robot.offsets));
        robot.auto_slow_mm=autonomy.config.slow_mm;
        robot.auto_clear_mm=(uint16_t)(autonomy.config.start_mm+autonomy.config.hysteresis_mm);
        if(!robot_begin(&robot,&r,task==4?R_ROUTE:R_SEEK,seq)){autonomy.state=AP_FAULT;autonomy.reason=AP_COMMAND;return false;}
    }
    for(unsigned i=0;i<4;i++){
        unsigned bit=canonical_bit[i];auto_illegal[i]=rd(MOTOR_BASE,0x30+4*bit);
        auto_last_pos[i]=rd(MOTOR_BASE,0x20+4*bit);auto_stall_ms[i]=in.now_ms;
        illegal_start[i]=rd(MOTOR_BASE,0x30+4*channel_to_bit[i]);
    }
    heading.enabled=true;wr(IMU_BASE,0x60,1);wr(MOTOR_BASE,8,1);wr(JDY_BASE,12,1);wr(JDY_BASE,12,2);
    local_lease=0;autonomy_owned=true;renewed_ms=in.now_ms;
    if(task==3){planner.reason=B_OK;planner.returned=false;planner.approach_stop_mm=planner.pass_mm=0;
        planner.started_ms=in.now_ms;auto_leg(B_APPROACH,counts(6000));}
    if(!auto_commit()){auto_fail(AP_DRIVE);return false;}
    start_reject_mask=0;return true;
}

/* 功能：推进任务三分段规划；in为本轮真实反馈，f/l为输出请求；返回任务是否仍活动。 */
static bool auto_task3(ap_input_t *in,int32_t *f,int32_t *l)
{
    if(!auto_leg_progress())return false;
    /* 视觉只附加阻挡/新鲜度检查；解除阻挡后仍由ap_step独立核对ToF和制动条件。 */
    if(task3_vision.enabled){
        in->target_required=true;in->target_timestamp_ms=task3_vision.stamp_ms;
        in->target_valid=fv_fresh(&task3_vision,in->now_ms)&&task3_vision.seen_target&&
            (planner.phase!=B_PASS||fv_released(&task3_vision,in->now_ms));
        if(!in->target_valid){in->planner_wait=true;return true;}
    }
    if(in->speed_mm_s)auto_quiet_ms=in->now_ms;
    bool settled=(uint32_t)(in->now_ms-auto_quiet_ms)>=200U;
    unsigned channel=planner.phase==B_SHIFT_RIGHT?1:planner.phase==B_RETURN_LEFT?2:0;
    uint8_t mask=(uint8_t)(1U<<channel);
    /* 前进在200mm处转入绕行准备，80mm仍是危险停车，不等到危险距离才规划。 */
    bool reached=auto_progress>=auto_leg_goal;
    if(planner.phase==B_APPROACH&&auto_clearance(0)>0&&auto_clearance(0)<=autonomy.config.slow_mm){
        planner.approach_stop_mm=auto_clearance(0);reached=true;
    }
    if(reached){
        auto_leg_wait=true;in->planner_wait=true;
        if(!settled||autonomy.speed_limit_mm_s)return true;
        if(planner.phase==B_APPROACH){
            if(!planner.approach_stop_mm){in->complete=true;planner.phase=B_DONE;planner.returned=false;return true;}
            uint32_t width=task3_vision.enabled?VISION_OBSTACLE_WIDTH_MM:geometry.obstacle_width_mm;
            uint32_t shift=(autonomy.config.body_width_mm+width+1U)/2U+geometry.clearance_mm;
            if(shift>(task3_vision.enabled?VISION_SHIFT_MAX_MM:geometry.shift_mm)){auto_fail(AP_CONFIG);return false;}
            auto_leg(B_SHIFT_RIGHT,counts(shift));mask=AP_RIGHT;
        }else if(planner.phase==B_SHIFT_RIGHT){
            if(task3_vision.enabled&&!fv_released(&task3_vision,in->now_ms)){
                in->target_required=true;in->target_valid=false;return true;
            }
            auto_shift_counts=auto_progress;right_settled_counts=auto_shift_counts;
            uint32_t depth=task3_vision.enabled?vision_depth_mm:geometry.obstacle_depth_mm;
            uint32_t pass=planner.approach_stop_mm+depth+autonomy.config.body_length_mm+geometry.clearance_mm;
            if(pass>(task3_vision.enabled?VISION_PASS_MAX_MM:900U)){auto_fail(AP_CONFIG);return false;}
            planner.pass_mm=(uint16_t)pass;auto_leg(B_PASS,counts(pass));mask=AP_FRONT;
        }else if(planner.phase==B_PASS){auto_leg(B_RETURN_LEFT,auto_shift_counts);mask=AP_LEFT;}
        else {planner.phase=B_DONE;planner.returned=true;in->complete=true;return true;}
        in->direction_mask=mask;in->planner_wait=false;return true;
    }
    in->direction_mask=mask;
    if(auto_progress>=auto_chunk_end){
        if(!auto_leg_wait){auto_leg_wait=true;auto_quiet_ms=in->now_ms;}
        in->planner_wait=!settled;
        if(autonomy.state==AP_RUN&&settled){
            uint32_t remain=auto_leg_goal-auto_progress;
            uint16_t mm=(uint16_t)((uint64_t)remain*1000000U/5268945U);
            uint16_t distance=ap_segment_limit(&autonomy,mm?mm:1,auto_clearance(channel));
            if(distance){uint32_t step=counts(distance);if(step>remain)step=remain;auto_chunk_end=auto_progress+step;auto_leg_wait=false;}
        }
        if(auto_leg_wait)return true;
    }
    if(channel==0)*f=526;else *l=channel==1?-526:526;
    return true;
}

/* 功能：每10ms执行所有自主任务的统一安全链；无参数/返回，任何失败直接撤销输出。 */
static void autonomy_tick(void)
{
    if(!autonomy_owned)return;
    ap_input_t in;autonomy_inputs(&in,autonomy.direction_mask,false);
    ap_refresh(&autonomy,&in); /* 发布真实新帧供规划器使用；不推进状态、不续租。 */
    int32_t f=0,l=0,t=0;
    if(auto_task==3){
        if(!auto_task3(&in,&f,&l))return;
        int32_t yaw=(int32_t)(heading.yaw_q16_md/65536)-auto_reference_md;
        if(yaw>15000||yaw< -15000){auto_fail(AP_IMU);return;}
        if(f||l){t=(-yaw*4-heading.rate_mdps)/1000;if(t>70)t=70;if(t< -70)t=-70;}
    }else{
        robot_input_t r;robot_inputs(&r);
        r.imu_valid=in.calibrated&&in.axis_confirmed&&!in.imu_hard_fault&&autonomy.imu_received&&
            (uint32_t)(in.now_ms-autonomy.imu_timestamp_ms)<autonomy.config.imu_age_ms;
        if(!r.imu_valid){auto_fail(AP_IMU);return;}
        for(unsigned i=0;i<4;i++){
            const ap_range_t *sample=&autonomy.accepted[i];
            r.range_valid[i]=sample->valid&&(autonomy.config.offset_known_mask&(1U<<i))&&
                (uint32_t)(in.now_ms-sample->timestamp_ms)<autonomy.config.range_age_ms;
            r.range_mm[i]=sample->raw_mm;r.range_seq[i]=sample->sequence;
        }
        robot_step(&robot,&r);
        f=robot.forward;l=robot.left;t=robot.turn;
        if(!robot_active(&robot)){if(robot.fault!=R_OK){auto_fail(AP_COMMAND);return;}in.complete=true;}
        uint8_t requested=auto_mask(f,l,t);
        if(requested&&requested!=autonomy.direction_mask){
            if(in.speed_mm_s||autonomy.speed_limit_mm_s){in.planner_wait=true;f=l=t=0;auto_switch_wait=true;}
            else {in.direction_mask=requested;auto_switch_wait=false;}
        }
        if(!requested&&!in.complete)in.planner_wait=true;
        if(auto_task>=5){in.target_required=true;in.target_valid=robot.target_valid;in.target_timestamp_ms=robot.target_ms;}
        in.rotation=in.direction_mask==AP_ALL;in.rotation_envelope_verified=auto_rotation_verified;
    }
    ap_step(&autonomy,&in);
    if(autonomy.state==AP_FAULT){
        if(autonomy.reason==AP_ENCODER){tof5_input_t v;inputs(&v);encoder_stop_trace(EF_ILLEGAL,15,&v,0);}
        auto_fail(autonomy.reason);return;
    }
    if(autonomy.state==AP_DONE){halt(T5_DISTANCE);return;}
    if(autonomy.state!=AP_RUN)f=l=t=0;
    auto_mix(f,l,t);
    for(unsigned i=0;i<4;i++){
        unsigned bit=canonical_bit[i];uint32_t position=rd(MOTOR_BASE,0x20+4*bit);
        if(position!=auto_last_pos[i]||auto_output.wheel_cps[i]==0)auto_stall_ms[i]=in.now_ms;
        auto_last_pos[i]=position;
        if(auto_output.wheel_cps[i]&&(uint32_t)(in.now_ms-auto_stall_ms[i])>=1000U){auto_fail(AP_ENCODER);return;}
    }
    mission.effort=0;
    for(unsigned i=0;i<4;i++){
        uint32_t actual=auto_abs((int32_t)rd(MOTOR_BASE,0xe0+4*i));
        if(actual>(uint32_t)mission.effort)mission.effort=(int32_t)(actual>32767U?32767U:actual);
    }
    mission.state=autonomy.state==AP_RUN?T5_RUN:T5_STOP;mission.reason=T5_NONE;
    if(!auto_commit())auto_fail(AP_DRIVE);
}

/* 功能：只读显示新策略身份和部署门禁；无参数/返回，不续租、不修改状态。 */
static void autonomy_reply(void)
{
    char b[320],*p=text(b,"AUTO V=1");p=field(p," TASK=",auto_task);p=field(p," OWN=",autonomy_owned);
    p=field(p," STATE=",autonomy.state);p=field(p," WHY=",autonomy.reason);
    p=field(p," BRAKE=",autonomy.config.brake_model_verified);p=field(p," OFFSETS=",autonomy.config.offset_known_mask);
    p=field(p," START=",autonomy.config.start_mm);p=field(p," STOP=",autonomy.config.emergency_mm);
    p=field(p," DYNAMIC=",autonomy.stop_mm);p=field(p," LIMIT=",autonomy.speed_limit_mm_s);
    p=field(p," LENGTH=",autonomy.config.body_length_mm);p=field(p," WIDTH=",autonomy.config.body_width_mm);
    p=text(p,"\r\n");*p=0;reply(b);
}
