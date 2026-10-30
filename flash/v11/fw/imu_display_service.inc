/* 功能：显式停机初始化后重新观察；无参数/返回，保留错误证据及本次上电自动恢复预算。 */
static void imu_display_manual_reset(void)
{
    imu_display.phase=IDR_WARM;imu_display.ready=false;imu_display.control_fault=false;
    imu_display.good_pairs=0;imu_display.last_acc=imu_display.last_gyro=0;
    imu_display.since_ms=rd(JDY_BASE,4);
}
/* 功能：保存首末故障并撤销旧航向；now为毫秒；无返回，不清除通信连接。 */
static void imu_display_fault(uint32_t now)
{
    imu_display.ready=false;imu_display.good_pairs=0;imu_display.control_fault=true;
    if(!imu_display.errors){
        imu_display.first_ms=now;imu_display.first_seq=imu.gyro_seq;imu_display.first_error=imu.error;
        imu_display.first_acc_id=imu.acc_id;imu_display.first_gyro_id=imu.gyro_id;
    }
    if(imu_display.errors!=UINT32_MAX)++imu_display.errors;
    imu_display.last_ms=now;imu_display.last_seq=imu.gyro_seq;imu_display.last_error=imu.error;
    imu_display.last_acc_id=imu.acc_id;imu_display.last_gyro_id=imu.gyro_id;imu_display.last_stage=imu.fault_stage;
    /* 故障时只撤销已活动的手势；静止的手机会话与配置继续保留。 */
    if(legacy.active||(rd(MOTOR_BASE,4)&1U))legacy_invalidate();
    halt(T5_SENSOR);heading.fault=true;heading.calibrated=false;heading.calibrating=false;
    imu_display.since_ms=now;
    imu_display.phase=imu.error==SENSOR_E_ID&&imu_display.attempts<3?IDR_WAIT:IDR_LOCKED;
}
/* 功能：非阻塞显示恢复；无参数/返回，每轮最多推进一次，错误样本绝不显示为有效。 */
static void imu_display_service(void)
{
    uint32_t now=rd(JDY_BASE,4);
    if(!imu_started)return;
    if(imu_display.control_fault){
        heading.fault=true;heading.calibrated=false;heading.calibrating=false;
        heading.enabled=heading.running=false;
    }
    if(imu_display.phase==IDR_LOCKED)return;
    if(imu_display.phase==IDR_WAIT){
        if((uint32_t)(now-imu_display.since_ms)<250U||(rd(MOTOR_BASE,4)&1U)||
           legacy.active||robot_active(&robot)||bypass_active(&planner)||tof5_active(&mission))return;
        ++imu_display.attempts;
        /* 复用带等待和配置读回的初始化；只启动状态机，不在这里延时或循环重试。 */
        imu_start();heading.fault=true;
        imu_display.phase=IDR_WARM;imu_display.since_ms=now;
        imu_display.last_acc=imu_display.last_gyro=imu_display.good_pairs=0;
        return;
    }
    if(imu.error<0){imu_display_fault(now);return;}
    bool fresh=imu.valid&&imu.error==SENSOR_OK&&
        (uint32_t)(rd(IMU_BASE,8)-imu.acc_us)<20000U&&(uint32_t)(rd(IMU_BASE,8)-imu.gyro_us)<20000U;
    if(!fresh){imu_display.ready=false;imu_display.good_pairs=0;}
    else if(imu.acc_seq!=imu_display.last_acc&&imu.gyro_seq!=imu_display.last_gyro){
        imu_display.last_acc=imu.acc_seq;imu_display.last_gyro=imu.gyro_seq;
        if(imu_display.good_pairs<20U)++imu_display.good_pairs;
        if(imu_display.good_pairs==20U){
            if(imu_display.phase==IDR_WARM&&imu_display.attempts)++imu_display.recovered;
            imu_display.ready=true;imu_display.phase=IDR_WATCH;
        }
    }
    /* 初始化/无DRDY超时不无限重试，也不伪造姿态。 */
    if(imu_display.phase==IDR_WARM&&(uint32_t)(now-imu_display.since_ms)>=1500U){
        imu.error=SENSOR_E_TIMEOUT;imu.valid=false;imu_display_fault(now);
    }
}
/* 功能：输出显示恢复和身份故障证据；无参数/返回，只读，不撤销手机配置。 */
static void imu_display_reply(void)
{
    char b[512],*p=text(b,"IDR V=1");
    p=field(p," STATE=",imu_display.phase);p=field(p," READY=",imu_display.ready);
    p=field(p," TRY=",imu_display.attempts);p=field(p," ERRORS=",imu_display.errors);
    p=field(p," RECOVERED=",imu_display.recovered);p=field(p," GOOD=",imu_display.good_pairs);
    p=field(p," LOCK=",imu_display.control_fault);
    p=text(p," FIRSTERR=");p=signed_number(p,imu_display.first_error);
    p=field(p," FIRSTA=",imu_display.first_acc_id);p=field(p," FIRSTG=",imu_display.first_gyro_id);
    p=field(p," FIRSTN=",imu_display.first_seq);p=field(p," FIRSTMS=",imu_display.first_ms);
    p=text(p," LASTERR=");p=signed_number(p,imu_display.last_error);
    p=field(p," LASTA=",imu_display.last_acc_id);p=field(p," LASTG=",imu_display.last_gyro_id);
    p=field(p," LASTN=",imu_display.last_seq);p=field(p," LASTMS=",imu_display.last_ms);
    p=field(p," STAGE=",imu_display.last_stage);p=text(p,"\r\n");*p=0;reply(b);
}
