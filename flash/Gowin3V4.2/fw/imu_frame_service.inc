/* 功能：初始化期间复用原SPI配置，运行期间只消费硬件完整帧。
 * 依赖同编译单元imu/heading/calibration、rd/wr、halt等；无RTOS、无动态分配。
 */
static uint32_t imu_lease_seq,imu_max_service_us,imu_rate_stamp,imu_rate_seq,imu_rate_millihz;
static uint32_t irq_missed_acc,irq_missed_gyro,irq_races;
static bool imu_rate_started,irq_armed;

/* 功能：仅初始化时执行SPI事务；参数遵循sensor_bus；返回0成功/-1失败。 */
static int imu_spi(void *ctx,uint8_t chip,const uint8_t *data,uint8_t *reply,size_t n,uint32_t timeout)
{
    (void)ctx;
    if(!data||!reply||chip>1||n<1||n>16||!timeout||rd(IMU_BASE,0x80))return -1;
    if(rd(IMU_BASE,0x0c)&1U)return -1;
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

/* 功能：停机后初始化IMU并清旧软件状态；无参数/返回，尚不启动周期读取。 */
static void imu_start(void)
{
    wr(IMU_BASE,0x80,0);wr(IMU_BASE,0x40,0);
    imu_started=true;heading_init(&heading);imu_max_service_us=0;
    imu_rate_stamp=imu_rate_seq=imu_rate_millihz=0;imu_rate_started=false;irq_armed=false;
    irq_missed_acc=irq_missed_gyro=irq_races=0;
    sensor_bus_t bus={0};bus.spi=imu_spi;
    bmi088_fifo_start(&imu,&bus,rd(IMU_BASE,8));
    if(rd(IMU_BASE,0)!=UINT32_C(0x424d4902))imu.error=SENSOR_E_ID;
}

/* 功能：消费最多一个硬件帧并调用原标定/积分；无参数/返回，不发起周期SPI读取。 */
static void imu_service(void)
{
    if(!imu_started)return;
    uint32_t begin_us=rd(IMU_BASE,8);imu.new_gyro=false;
    if(!irq_armed){
        bmi088_events_t events={0};bmi088_irq_step(&imu,&events,begin_us);
        if(imu.phase&&imu.error>=0){wr(IMU_BASE,0x40,2);wr(IMU_BASE,0x80,1);irq_armed=true;}
        return;
    }
    uint32_t status=rd(IMU_BASE,0x84),fault=rd(IMU_BASE,0x88);
    if((status>>31)||fault){
        imu.error=SENSOR_E_IO;imu.valid=false;heading.fault=true;
        heading.calibrated=false;heading.calibrating=false;
        if(fault&4U)imu.rejected_frames=1;
        if(fault&8U)irq_races=1;
    }else if(status&15U){
        uint32_t w[7];for(unsigned i=0;i<7;i++)w[i]=rd(IMU_BASE,0xa0+4*i);
        wr(IMU_BASE,0x8c,1); /* 完整头帧读完才出队，生产者不能改写读取中的帧。 */
        uint32_t now=rd(IMU_BASE,8);
        bool fresh=w[3]==1&&w[0]!=0&&(!imu.gyro_seq||w[0]==imu.gyro_seq+1)&&
            (uint32_t)(now-w[1])<20000&&(uint32_t)(now-w[2])<20000;
        if(!fresh){imu.valid=false;heading.fault=true;heading.calibrated=false;heading.calibrating=false;}
        else{
            imu.acc[0]=(int16_t)w[4];imu.acc[1]=(int16_t)(w[4]>>16);imu.acc[2]=(int16_t)w[5];
            imu.gyro[0]=(int16_t)(w[5]>>16);imu.gyro[1]=(int16_t)w[6];imu.gyro[2]=(int16_t)(w[6]>>16);
            imu.acc_us=w[2];imu.gyro_us=w[1];imu.acc_seq=imu.gyro_seq=w[0];imu.valid=true;imu.new_gyro=true;
            if(imu_lease_seq!=UINT32_MAX)wr(IMU_BASE,0x64,++imu_lease_seq);
            if(!imu_rate_started){imu_rate_stamp=w[1];imu_rate_seq=w[0];imu_rate_started=true;}
            else if((uint32_t)(w[1]-imu_rate_stamp)>=1000000){
                imu_rate_millihz=(uint32_t)((uint64_t)(w[0]-imu_rate_seq)*1000000000/(uint32_t)(w[1]-imu_rate_stamp));
                imu_rate_stamp=w[1];imu_rate_seq=w[0];
            }
            heading_calibration_sample(&calibration,&heading,imu.acc,imu.gyro,true,imu.gyro_us);
        }
    }else if(!imu.gyro_seq||(uint32_t)(begin_us-imu.gyro_us)>=20000)imu.valid=false;
    uint32_t elapsed=rd(IMU_BASE,8)-begin_us;if(elapsed>imu_max_service_us)imu_max_service_us=elapsed;
    if(!imu.valid&&(heading.calibrating||heading.enabled)){
        heading.fault=true;heading.calibrated=false;heading.calibrating=false;
    }
    if(bypass_active(&planner)&&(!imu.valid||heading.fault)){bypass_abort(&planner,B_IMU);halt(T5_SENSOR);}
}
