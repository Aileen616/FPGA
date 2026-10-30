/* 本文件仅由motor_imu_main.c包含：五线SPI适配和BMI088命令，不改变旧MOTOR3协议。 */
static uint32_t imu_lease_seq; /* 微控制器复位前严格递增，不随传感器重新初始化归零。 */
static uint32_t imu_max_service_us; /* 本次初始化后单次采样服务最坏耗时，单位us。 */
static uint32_t imu_rate_stamp,imu_rate_seq,imu_rate_millihz; /* 最近约1秒有效读出频率，单位mHz。 */
static bool imu_rate_started; /* 首帧仅建立统计起点，不把上电等待时间算入帧率。 */
static unsigned imu_gyro_odr=200; /* 诊断可选100/200Hz，复位默认200Hz；加速度仍200Hz。 */
static unsigned imu_odr_phase; /* 0待写、1等待配置稳定、2已读回；在FIFO启用前完成。 */
static uint32_t imu_odr_stamp; /* 陀螺ODR写入时刻us，用于非阻塞5ms等待。 */
typedef struct {uint32_t acc,gyro,at,gt,now,levels;} imu_irq_snapshot_t;
static uint32_t irq_last_acc,irq_last_gyro,irq_missed_acc,irq_missed_gyro,irq_races;
static bool irq_armed; /* 初始化成功后才启用边沿捕获。 */
static bmi088_fifo_t irq_candidate; /* 静态暂存，读期间出现新边沿时不发布时间戳不匹配的数据。 */
/* 功能：取得PL原子快照；s不可空；无返回，不清除真实边沿计数。 */
static void imu_irq_snapshot(imu_irq_snapshot_t *s)
{
    wr(IMU_BASE,0x40,1);s->acc=rd(IMU_BASE,0x48);s->gyro=rd(IMU_BASE,0x4c);
    s->at=rd(IMU_BASE,0x50);s->gt=rd(IMU_BASE,0x54);s->now=rd(IMU_BASE,0x58);s->levels=rd(IMU_BASE,0x44);
}
/* 功能：执行有超时的SPI事务；ctx可空，其余参数遵循sensor_bus_t；成功0，失败-1。 */
static int imu_spi(void *ctx,uint8_t chip,const uint8_t *data,uint8_t *reply,size_t n,uint32_t timeout)
{
    (void)ctx;
    if(!data||!reply||chip>1||n<1||n>16||!timeout)return -1;
    if(rd(IMU_BASE,0x0c)&1U){wr(IMU_BASE,0x10,0x100);return -1;}
    /* 只搬运有效字节；固定移位避免无桶形移位器软核反复执行可变移位。 */
    for(unsigned i=0;i<n;i+=4){
        uint32_t word=data[i];
        if(i+1<n)word|=(uint32_t)data[i+1]<<8;
        if(i+2<n)word|=(uint32_t)data[i+2]<<16;
        if(i+3<n)word|=(uint32_t)data[i+3]<<24;
        wr(IMU_BASE,0x20+i,word);
    }
    uint32_t since=rd(IMU_BASE,8);
    wr(IMU_BASE,0x10,1U|((uint32_t)chip<<1)|((uint32_t)(n-1)<<4));
    for(;;){
        uint32_t status=rd(IMU_BASE,0x0c);
        if(status&4U||(uint32_t)(rd(IMU_BASE,8)-since)>=timeout){wr(IMU_BASE,0x10,0x100);return -1;}
        if((status&3U)==2U)break;
    }
    for(unsigned i=0;i<n;i+=4){
        uint32_t word=rd(IMU_BASE,0x30+i);reply[i]=(uint8_t)word;
        if(i+1<n)reply[i+1]=(uint8_t)(word>>8);
        if(i+2<n)reply[i+2]=(uint8_t)(word>>16);
        if(i+3<n)reply[i+3]=(uint8_t)(word>>24);
    }
    return 0;
}
/* 功能：在停机时开始传感器初始化；无参数/返回，失败保持输出禁止。 */
static void imu_start(void)
{
    imu_started=true;
    sensor_bus_t bus={0};bus.spi=imu_spi;heading_init(&heading);imu_max_service_us=0;
    imu_rate_stamp=imu_rate_seq=imu_rate_millihz=0;imu_rate_started=false;
    imu_odr_phase=0;
    wr(IMU_BASE,0x40,0);irq_armed=false;
    irq_last_acc=irq_last_gyro=irq_missed_acc=irq_missed_gyro=irq_races=0;
    if(rd(IMU_BASE,0)!=UINT32_C(0x424d4902)){imu.error=SENSOR_E_ID;return;}
    bmi088_fifo_start(&imu,&bus,rd(IMU_BASE,8));
}
/* 功能：按DRDY服务BMI088；无参数/返回，失鲜和漏帧锁存，禁止自动恢复。 */
static void imu_service(void)
{
    if(!imu_started)return;
    uint32_t now=rd(IMU_BASE,8);
    // 仅测试100Hz时覆盖原初始化的200Hz配置；不改变公共驱动或初始化等待规则。
    if(imu_gyro_odr==100&&bmi088_ready(&imu.core)&&!imu.phase&&imu.error>=0){
        uint8_t tx_odr[2]={0x10,0x05},rx_odr[2];
        if(!imu_odr_phase){
            if(imu_spi(0,1,tx_odr,rx_odr,2,2000)){imu.error=SENSOR_E_IO;return;}
            imu_odr_stamp=rd(IMU_BASE,8);imu_odr_phase=1;return;
        }
        if(imu_odr_phase==1){
            if((uint32_t)(now-imu_odr_stamp)<5000)return;
            tx_odr[0]=0x90;tx_odr[1]=0;
            if(imu_spi(0,1,tx_odr,rx_odr,2,2000)){imu.error=SENSOR_E_IO;return;}
            if((rx_odr[1]&7)!=5){imu.error=SENSOR_E_CONFIG;return;}
            imu_odr_phase=2;
        }
    }
    bmi088_events_t events={0};
    if(!irq_armed){
        bmi088_irq_step(&imu,&events,now);
        if(imu.phase&&imu.error>=0){wr(IMU_BASE,0x40,2);irq_armed=true;}
    }else{
        /* 无事件时仍更新失鲜状态；每轮只允许本轮新陀螺样本进入积分。 */
        bmi088_irq_step(&imu,&events,now);
        for(unsigned chip=0;chip<2&&imu.error>=0;chip++){
            imu_irq_snapshot_t before,after;imu_irq_snapshot(&before);
            uint32_t count=chip?before.gyro:before.acc;
            uint32_t *last=chip?&irq_last_gyro:&irq_last_acc;
            uint32_t delta=count-*last;
            if(!delta)continue;
            if(delta>1){if(chip)irq_missed_gyro+=delta-1;else irq_missed_acc+=delta-1;}
            *last=count;events.mask=(uint8_t)(1U<<chip);
            events.acc_us=before.at;events.gyro_us=before.gt;
            irq_candidate=imu;bmi088_irq_step(&irq_candidate,&events,before.now);
            imu_irq_snapshot(&after);
            /* 两个芯片异步采样：只把当前芯片实际读区间内的新边沿判为竞态。
             * 已读完芯片的新边沿不消费，下轮继续读取；真实跨帧仍立即锁故障。 */
            if(irq_candidate.error>=0&&(chip?after.gyro:after.acc)!=count){
                ++irq_races;imu.new_gyro=false;imu.valid=false;break;
            }
            imu=irq_candidate;
        }
    }
    uint32_t elapsed=rd(IMU_BASE,8)-now;
    if(elapsed>imu_max_service_us)imu_max_service_us=elapsed;
    if(imu.new_gyro&&imu.valid){
        if(imu_lease_seq!=UINT32_MAX)wr(IMU_BASE,0x64,++imu_lease_seq);
        if(!imu_rate_started){imu_rate_stamp=imu.gyro_us;imu_rate_seq=imu.gyro_seq;imu_rate_started=true;}
        else if((uint32_t)(imu.gyro_us-imu_rate_stamp)>=1000000){
            imu_rate_millihz=(uint32_t)((uint64_t)(imu.gyro_seq-imu_rate_seq)*1000000000/(uint32_t)(imu.gyro_us-imu_rate_stamp));
            imu_rate_stamp=imu.gyro_us;imu_rate_seq=imu.gyro_seq;
        }
        /* 多帧积压时读出间隔不代表采样间隔，保留原始诊断，但禁止用于航向积分。 */
        heading_calibration_sample(&calibration,&heading,imu.acc,imu.gyro,true,imu.gyro_us);
    }
    if(!imu.valid&&(heading.calibrating||heading.enabled)){
        heading.fault=true;heading.calibrated=false;heading.calibrating=false;
    }
    if(irq_missed_acc||irq_missed_gyro||irq_races||imu.rejected_frames) {
        if(heading.calibrating){calibration.reason|=HCAL_TIME;calibration.axis_mask|=7;}
        heading.fault=true;heading.calibrated=false;heading.calibrating=false;
    }
    if(bypass_active(&planner)&&(!imu.valid||heading.fault)){
        bypass_abort(&planner,B_IMU);halt(T5_SENSOR);
    }
}
