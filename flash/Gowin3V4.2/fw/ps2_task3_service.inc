/* 依赖：tof5_main.c的既有绕障、IMU、PS2事件和蓝牙接口；仅用于任务3独立候选。 */
#define PS2_BASE UINT32_C(0x20009000)
static ps2_control_t pad; /* 按键边沿与120ms失联检测，固定任务3。 */
static uint32_t pad_cal_since; /* 当前标定阶段开始时刻，单位ms。 */
enum { PAD_IDLE, PAD_RUNNING, PAD_DONE, PAD_NOT_READY, PAD_CAL_FAILED, PAD_STOPPED };
static uint8_t pad_result; /* 上述枚举结果；未通过不能自动补启动。 */
#include "ps2_start_history.inc"

/* 功能：结束静止标定；ok为本轮是否通过原IMU准入；无返回，不启动任务。 */
static void pad_cal_finish(bool ok)
{
    pad_cal_stage=0;pad_owner=false;
    if(!ok){heading.calibrating=false;heading.calibrated=false;}
    halt(T5_CANCEL);pad_result=ok?PAD_IDLE:PAD_CAL_FAILED;
}

/* 功能：取消手柄任务及标定；无参数/返回，保留任务终态和原停车保护。 */
static void pad_stop(void)
{
    if(pad_cal_stage)pad_cal_finish(false);
    pad_owner=false;halt(T5_CANCEL);pad_result=PAD_STOPPED;
}

/* 功能：无阻塞推进标定；无参数/返回；每阶段最多8秒，失败不重试。 */
static void pad_cal_service(void)
{
    uint32_t elapsed=rd(JDY_BASE,4)-pad_cal_since;
    if(heading.fault||imu.error<0||elapsed>=8000U){pad_cal_finish(false);return;}
    if(pad_cal_stage==1){
        if(elapsed<500U||!imu.valid)return;
        imu_command("imu bias");
        if(!heading.calibrating){pad_cal_finish(false);return;}
        pad_cal_stage=2;pad_cal_since=rd(JDY_BASE,4);
    }else if(!heading.calibrating){
        if(heading.calibrated)imu_command("imu axis 2 1"); /* 复用已核对的Z2/+1。 */
        pad_cal_finish(imu_ready());
    }
}

/* 功能：消费手柄快照并触发一次原有右绕；无参数/返回，只有新有效帧续租。 */
static void pad_service(void)
{
    uint32_t seq=rd(PS2_BASE,4),valid=rd(PS2_BASE,8),buttons=rd(PS2_BASE,12),axes=rd(PS2_BASE,16);
    if(seq!=rd(PS2_BASE,4))return; /* 发布期间不拼接不同帧。 */
    bool busy=maze_owned||bypass_active(&planner)||robot_active(&robot)||legacy.active||
        (rd(MOTOR_BASE,4)&1U)||pad_cal_stage||heading.calibrating;
    uint16_t previous_buttons=pad.previous;
    bool fresh=ps2_control_update(&pad,seq,rd(JDY_BASE,4),valid!=0,(uint16_t)~buttons,axes,busy);
    pad.task=pad.selected=3; /* 此候选按START直接执行任务3，菜单不切换任务。 */
    if(pad.stop&&((!autonomy_owned&&!maze_owned)||(fresh&&pad.linked))){pad_stop();return;}
    if(pad_cal_stage){pad_cal_service();return;}
    if(pad_owner&&!bypass_active(&planner)){
        pad_owner=false;
        pad_result=planner.phase==B_DONE&&planner.returned?PAD_DONE:PAD_STOPPED;
        return; /* 本帧启动边沿已消费，终止后必须重新按键。 */
    }
    if(!fresh||!pad.linked)return;
    if(!busy&&((uint16_t)~buttons&PS_SQUARE)&&!(previous_buttons&PS_SQUARE)){
        (void)vision_command(task3_vision.enabled?"vision mode 0":"vision mode 1");
        return; /* 同帧方块与START只切模式，不启动或排队。 */
    }
    if(pad.confirm&&!busy){
        legacy_invalidate();length=0;dropping=true;
        imu_command("imu start");pad_cal_stage=1;pad_cal_since=rd(JDY_BASE,4);pad_result=PAD_IDLE;
        return; /* 同帧○与START只标定，不排队运动。 */
    }
    if(pad.start&&!busy){
        legacy_invalidate();length=0;dropping=true;
        /* 与原bypass arm完全相同，修复旧PS2入口依赖电脑预配置的问题。 */
        /* 统一入口先检查策略，再配置PI；拒绝不使能电机。 */
        pad_start_record_t *attempt=pad_record_start();
        pad_owner=begin(sequence==UINT32_MAX?0U:sequence+1U);
        attempt->accepted=pad_owner;attempt->mask=start_reject_mask;
        if(!pad_owner&&!attempt->mask)attempt->mask=SG_INTERNAL;
        start_reject_mask=attempt->mask;
        pad_result=pad_owner?PAD_RUNNING:PAD_NOT_READY;
        return; /* 不等待传感器变好后自动启动；拒绝后需要重新按START。 */
    }
    /* 自主续租移至本地控制步，普通PS2断链不取消已启动任务。 */
}

/* 功能：显示任务3入口、阶段和拒绝原因；无参数/返回，保留四路距离与姿态行。 */
static void pad_osd(void)
{
    char b[64],*p;
    if(autonomy_owned){p=decimal(text(b,"AUTO T"),auto_task);p=text(p," PHASE ");}
    else p=text(b,pad.linked?"PS2 T3 PHASE ":"PS2 OFF PHASE ");
    p=decimal(p,planner.phase);p=field(p," WHY ",planner.reason);*p=0;osd_line(0,b);
    if(task3_vision.enabled&&(!autonomy_owned||auto_task==3)){
        p=decimal(text(b,"T3 P"),planner.phase);p=text(p,!vision_profile_verified?" V:CAL":
            !fv_fresh(&task3_vision,rd(JDY_BASE,4))?" V:INVALID":
            !fv_released(&task3_vision,rd(JDY_BASE,4))?" V:BLOCK":
            !task3_vision.target_visible?" V:UNKNOWN":
            task3_vision.position==FV_LEFT?" V:LEFT":task3_vision.position==FV_RIGHT?" V:RIGHT":" V:CENTER");*p=0;osd_line(0,b);
    }
    const char *message;
    if(pad_cal_stage)message="IMU CAL / KEEP STILL";
    else if(autonomy_owned)message=autonomy.state==AP_WAIT?(autonomy.reason==AP_TARGET?"VISION WAIT / CROSS STOP":"AUTO WAIT / CHECK CLEARANCE"):"AUTO RUN / CROSS STOP";
    else if(pad_result==PAD_CAL_FAILED)message="CAL FAILED / CIRCLE RETRY";
    else if(pad_result==PAD_DONE)message="T3 DONE / START AGAIN";
    else if(pad_result==PAD_NOT_READY)message=autonomy.reason==AP_CONFIG?"AUTO CONFIG / HW PENDING":pad_reject_text();
    else if(mission.reason==T5_ENCODER&&encoder_trace.valid)message="ENC FAULT / READ TRACE";
    else if(!imu_ready())message="KEEP STILL / CIRCLE CAL";
    else if(pad_result==PAD_STOPPED)message="STOPPED / START RETRY";
    else message="T3 READY / START RUN";
    osd_line(12,message);
}
