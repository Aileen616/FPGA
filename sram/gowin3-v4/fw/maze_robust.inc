/* IMU在初始化后统一使用；ToF及通信策略仍仅在迷宫持权时接入。 */
typedef struct {
    uint32_t event_us,first_bad_us,last_bad_us; /* 独立事件及当前确认窗口首末异常时间，微秒。 */
    uint8_t count,good,seen; /* 饱和异常计数、连续恢复计数和事件去重标志。 */
} maze_quality_t;
static maze_quality_t maze_iq[2],maze_tq[4],maze_lq;
static uint8_t maze_health_fault; /* 0正常，否则锁存本任务的MT原因。 */
static uint32_t maze_imu_bad,maze_link_bad; /* 本任务拒绝的独立事件数，仅诊断。 */

/* 功能：加入独立事件；q不可空，bad为失败，us为真实时间，window为正微秒窗口；无返回。 */
static void maze_quality_event(maze_quality_t *q,bool bad,uint32_t us,uint32_t window)
{
    if(q->seen&&(int32_t)(us-q->event_us)<=0)return;
    q->seen=1;q->event_us=us;
    if(q->count&&(uint32_t)(us-q->first_bad_us)>window)q->count=0;
    if(bad){
        /* 高频事务也保留首错时间，不能用固定队列挤掉持续时间证据。 */
        if(!q->count)q->first_bad_us=us;
        if(q->count<255)++q->count;
        q->last_bad_us=us;q->good=0;
    }else{
        if(q->good<3)++q->good;
        if(q->good>=3)q->count=0; /* 一两帧偶然成功不能无限推迟持续故障。 */
    }
}
/* 功能：检查窗口内持续异常；q不可空，span/window/now均微秒；返回是否确认。 */
static bool maze_quality_bad(const maze_quality_t *q,uint32_t span,uint32_t window,uint32_t now)
{
    return q->count>=3&&(uint32_t)(now-q->first_bad_us)<=window&&
           (uint32_t)(q->last_bad_us-q->first_bad_us)>=span;
}
/* 功能：锁存迷宫故障并停机；why为MT原因；无返回，不刷新任何反馈。 */
static void maze_health_stop(uint8_t why)
{
    if(!maze_owned)return;
    if(!maze_health_fault)maze_health_fault=why;
    maze_shadow.state=MT_FAULT;maze_shadow.reason=maze_health_fault;
    halt(why==MT_LINK?T5_LINK:T5_SENSOR);
}
/* 功能：显式初始化或标定时清当前确认窗口；无参数/返回，不清硬件累计诊断。 */
static void imu_quality_reset(void)
{memset(maze_iq,0,sizeof(maze_iq));imu_quality_fault=0;maze_imu_bad=0;}
/* 功能：确认IMU持续异常或真实失鲜；无参数；返回已确认故障，不伪造新样本。 */
static bool imu_quality_check(void)
{
    uint32_t now=rd(IMU_BASE,8);
    if(!imu_quality_fault&&imu.acc_seq&&imu.gyro_seq&&
       ((uint32_t)(now-imu.acc_us)>=20000U||(uint32_t)(now-imu.gyro_us)>=20000U||
        maze_quality_bad(&maze_iq[0],10000,40000,now)||maze_quality_bad(&maze_iq[1],10000,40000,now))){
        imu_quality_fault=1;
        if(heading.calibrating){calibration.reason|=HCAL_TIME;calibration.axis_mask|=7;}
        heading.fault=true;heading.calibrated=heading.calibrating=false;
        if(maze_owned)maze_health_stop(MT_IMU);else halt(T5_SENSOR);
    }
    return imu_quality_fault!=0;
}
/* 功能：开始新任务的独立确认窗口；无参数/返回，不清历史硬件计数。 */
static void maze_health_reset(void)
{
    imu_quality_reset();memset(maze_tq,0,sizeof(maze_tq));memset(&maze_lq,0,sizeof(maze_lq));
    maze_health_fault=0;maze_imu_bad=maze_link_bad=0;
}
/* 功能：按当前方向检查底层证据；无参数；返回本次是否已停止。 */
static bool maze_health_check(void)
{
    if(!maze_owned)return false;
    uint32_t now=rd(IMU_BASE,8);uint8_t why=0,d=maze_shadow.dir;
    if((uint32_t)(now-heading.stamp_us)>=20000U||(uint32_t)(now-imu.acc_us)>=20000U||
       maze_quality_bad(&maze_iq[0],10000,40000,now)||maze_quality_bad(&maze_iq[1],10000,40000,now))why=MT_IMU;
    bool q[4];static const uint8_t slot[4]={0,1,3,2};
    for(unsigned i=0;i<4;i++)q[i]=maze_quality_bad(&maze_tq[slot[i]],50000,400000,now);
    if(!why&&(q[d]||(q[(d+1)&3]&&q[(d+3)&3])))why=MT_RANGE;
    if(!why&&maze_quality_bad(&maze_lq,50000,200000,now))why=MT_LINK;
    if(why){maze_health_stop(why);return true;}return false;
}
/* 功能：记录新发生的坏包/队列拒绝；无参数/返回，同一次坏包只调用一次。 */
static void maze_link_error(void)
{
    if(!maze_owned)return;
    ++maze_link_bad;maze_quality_event(&maze_lq,true,rd(IMU_BASE,8),200000);(void)maze_health_check();
}
/* 功能：轮询一个真实ToF事务；us为轮询开始时间；无返回，不重启或重放失败事务。 */
static void maze_tof_step(uint32_t us)
{
    bool running=vl53l1x_array_ready(&joint_tof);unsigned i=joint_tof.poll_index;
    uint32_t seq=joint_tof.sample[i].sequence;
    sensor_result_t rc=vl53l1x_array_step(&joint_tof,us);
    if(maze_owned&&running&&(rc<0||joint_tof.sample[i].sequence!=seq))
        maze_quality_event(&maze_tq[i],rc<0||joint_tof.sample[i].validity!=SENSOR_VALID,us,400000);
}
/* 功能：以新DRDY事务隔离瞬态坏读，保持真实时间；无参数/返回，不重置IMU或标定。 */
static void maze_imu_service(void)
{
    (void)imu_quality_check();if(maze_health_check())return;
    uint32_t begin=rd(IMU_BASE,8);imu.new_gyro=false;
    for(unsigned chip=0;chip<2;chip++){
        imu_irq_snapshot_t before,after;imu_irq_snapshot(&before);
        uint32_t count=chip?before.gyro:before.acc,*last=chip?&irq_last_gyro:&irq_last_acc;
        uint32_t delta=count-*last;if(!delta)continue;
        *last=count;
        if(delta>1){if(chip)irq_missed_gyro+=delta-1;else irq_missed_acc+=delta-1;}
        bmi088_events_t ev={.mask=(uint8_t)(1U<<chip),.acc_us=before.at,.gyro_us=before.gt};
        irq_candidate=imu;sensor_result_t rc=bmi088_irq_step(&irq_candidate,&ev,before.now);
        imu_irq_snapshot(&after);bool raced=(chip?after.gyro:after.acc)!=count;
        bool updated=chip?irq_candidate.gyro_seq!=imu.gyro_seq:irq_candidate.acc_seq!=imu.acc_seq;
        bool bad=rc<0||raced||!updated;
        if(raced)++irq_races;
        if(irq_candidate.rejected_frames!=imu.rejected_frames)imu.rejected_frames=irq_candidate.rejected_frames;
        maze_quality_event(&maze_iq[chip],bad||delta>1,chip?before.gt:before.at,40000);
        if(bad){++maze_imu_bad;continue;} /* 保留旧可信值与原时间，绝不发布半帧。 */
        imu=irq_candidate;
        if(chip&&imu.valid){
            /* 20ms内缺一帧仍沿用原积分规则；超过20ms由独立年龄检查锁停，不补时间。 */
            heading_calibration_sample(&calibration,&heading,imu.acc,imu.gyro,true,imu.gyro_us);
            if(imu_lease_seq!=UINT32_MAX)wr(IMU_BASE,0x64,++imu_lease_seq);
            if((uint32_t)(imu.gyro_us-imu_rate_stamp)>=1000000U){
                imu_rate_millihz=(uint32_t)((uint64_t)(imu.gyro_seq-imu_rate_seq)*1000000000U/(uint32_t)(imu.gyro_us-imu_rate_stamp));
                imu_rate_stamp=imu.gyro_us;imu_rate_seq=imu.gyro_seq;
            }
        }
    }
    uint32_t now=rd(IMU_BASE,8);imu.valid=(uint32_t)(now-imu.gyro_us)<20000U&&(uint32_t)(now-imu.acc_us)<20000U;
    uint32_t elapsed=now-begin;if(elapsed>imu_max_service_us)imu_max_service_us=elapsed;
    (void)imu_quality_check();(void)maze_health_check();
}
