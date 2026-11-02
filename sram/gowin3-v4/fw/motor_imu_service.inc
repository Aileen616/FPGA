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
    sensor_bus_t bus={0};bus.spi=imu_spi;heading_init(&heading);imu_max_service_us=0;
    imu_rate_stamp=imu_rate_seq=imu_rate_millihz=0;imu_rate_started=false;
    imu_odr_phase=0;
    wr(IMU_BASE,0x40,0);irq_armed=false;
    irq_last_acc=irq_last_gyro=irq_missed_acc=irq_missed_gyro=irq_races=0;
    if(rd(IMU_BASE,0)!=UINT32_C(0x424d4902)){imu.error=SENSOR_E_ID;return;}
    bmi088_fifo_start(&imu,&bus,rd(IMU_BASE,8));
}
/* 功能：向充足的缓冲区追加文本；p/s不可空；返回末尾指针。 */
static char *imu_text(char *p,const char *s){while(*s)*p++=*s++;return p;}
/* 功能：追加signed32十进制；p至少12字节，v任意；返回末尾指针。 */
static char *imu_number(char *p,int32_t v)
{
    char b[10];unsigned n=0;uint32_t u=(uint32_t)v;
    if(v<0){*p++='-';u=0U-u;}
    do{b[n++]=(char)('0'+u%10);u/=10;}while(u);
    while(n)*p++=b[--n];
    return p;
}
/* 功能：追加键值；p/key不可空，v任意32位；返回末尾指针。 */
static char *imu_field(char *p,const char *key,int32_t v)
{return imu_number(imu_text(p,key),v);}
/* 功能：整包发送扩展回复；p指向buffer尾端，buffer不可空；失败锁存通信停机，无返回。 */
static void imu_send(char *buffer,char *p)
{p=imu_text(p,"\r\n");if(!enqueue(0,buffer,(size_t)(p-buffer)))jdy_motor_transport_fault(&motor);}
/* 功能：输出拒绝原因；why非空常量；无返回，立即撤销运动。 */
static void imu_reject(const char *why)
{char b[96],*p=b;jdy_motor_rx(&motor,'~',rd(JDY_BASE,4));p=imu_text(p,"IMU ERR=");p=imu_text(p,why);imu_send(b,p);}
/* 功能：处理完整IMU命令；line不可空且以NUL结尾；无返回，只在明确的命令中改变状态。 */
static void imu_command(const char *line)
{
    char b[300],*p=b;bool idle=!(rd(MOTOR_BASE,4)&1U)&&!motor.active;
    if(!strcmp(line,"imu irq")){
        imu_irq_snapshot_t s;imu_irq_snapshot(&s);
        p=imu_text(p,"IIRQ");p=imu_field(p," EN=",irq_armed);p=imu_field(p," A=",(int32_t)s.acc);
        p=imu_field(p," G=",(int32_t)s.gyro);p=imu_field(p," AT=",(int32_t)s.at);p=imu_field(p," GT=",(int32_t)s.gt);
        p=imu_field(p," T=",(int32_t)s.now);p=imu_field(p," LEVEL=",(int32_t)s.levels);
        p=imu_field(p," AMISS=",(int32_t)irq_missed_acc);p=imu_field(p," GMISS=",(int32_t)irq_missed_gyro);
        p=imu_field(p," RACE=",(int32_t)irq_races);imu_send(b,p);return;
    }
    if(!strcmp(line,"imu init")){
        p=imu_text(p,"IINIT");p=imu_field(p," STATE=",imu.core.state);p=imu_field(p," INDEX=",imu.core.config_index);
        p=imu_field(p," CHIP=",imu.core.diag_chip);p=imu_field(p," REG=",imu.core.diag_reg);
        p=imu_field(p," WANT=",imu.core.diag_expected);p=imu_field(p," GOT=",imu.core.diag_actual);
        p=imu_field(p," MASK=",imu.core.diag_mask);p=imu_field(p," AID=",imu.core.acc_id);
        p=imu_field(p," GID=",imu.core.gyro_id);imu_send(b,p);return;
    }
    if(!strcmp(line,"imu clock")){
        uint32_t clk=rd(IMU_BASE,4),div=rd(IMU_BASE,0x14);
        p=imu_text(p,"ICLOCK");p=imu_field(p," CLK=",(int32_t)clk);p=imu_field(p," DIV=",(int32_t)div);
        p=imu_field(p," HZ=",div?(int32_t)(clk/(2*div)):0);p=imu_field(p," ODR=",(int32_t)imu_gyro_odr);
        imu_send(b,p);return;
    }
    if(!strcmp(line,"imu feedback")){
        /* 裸机同一主循环内一次快照；时间戳与XYZ来自同一有效帧，发送期间不重新取值。 */
        bool good=imu.valid&&heading.feedback_valid&&(uint32_t)(rd(IMU_BASE,8)-heading.stamp_us)<20000;
        p=imu_text(p,"IFB V=1");p=imu_field(p," N=",(int32_t)imu.gyro_seq);
        p=imu_field(p," T=",(int32_t)heading.stamp_us);p=imu_field(p," DT=",(int32_t)heading.dt_us);
        p=imu_field(p," HZM=",good?(int32_t)imu_rate_millihz:0);p=imu_field(p," OK=",good);
        p=imu_field(p," CAL=",heading.calibrated);p=imu_field(p," DROP=",(int32_t)imu.rejected_frames);
        p=imu_field(p," GX=",heading.gyro_mdps[0]);
        p=imu_field(p," GY=",heading.gyro_mdps[1]);p=imu_field(p," GZ=",heading.gyro_mdps[2]);
        imu_send(b,p);return;
    }
    if(!strcmp(line,"imu trace")){
        p=imu_text(p,"ITRACE");p=imu_field(p," BEFORE=",imu.fifo_before);
        p=imu_field(p," AFTER=",imu.fifo_after);p=imu_field(p," FC0=",imu.config0);
        p=imu_field(p," FC1=",imu.config1);p=imu_field(p," GAP=",(int32_t)imu.poll_gap_us);
        p=imu_field(p," MAXGAP=",(int32_t)imu.max_poll_gap_us);p=imu_field(p," STAGE=",imu.fault_stage);
        p=imu_field(p," N=",(int32_t)imu.gyro_seq);imu_send(b,p);return;
    }
    if(!strcmp(line,"imu diag")){
        p=imu_text(p,"IDIAG");p=imu_field(p," FIFO=",imu.fifo_status);
        p=imu_field(p," MAXFIFO=",imu.max_fifo);p=imu_field(p," MAXUS=",(int32_t)imu_max_service_us);
        p=imu_field(p," T=",(int32_t)rd(IMU_BASE,8));imu_send(b,p);return;
    }
    if(!strcmp(line,"imu")){
        p=imu_text(p,"IMU V=1");p=imu_field(p," AID=",imu.acc_id);p=imu_field(p," GID=",imu.gyro_id);
        p=imu_field(p," OK=",imu.valid);p=imu_field(p," ERR=",imu.error);p=imu_field(p," N=",(int32_t)imu.gyro_seq);
        p=imu_field(p," AN=",(int32_t)imu.acc_seq);p=imu_field(p," CAL=",heading.calibrated);
        p=imu_field(p," COUNT=",(int32_t)heading.samples);p=imu_field(p," BUSY=",heading.calibrating);
        p=imu_field(p," FAULT=",heading.fault);p=imu_field(p," AXIS=",heading.axis_confirmed?heading.axis:-1);
        p=imu_field(p," SIGN=",heading.sign);p=imu_field(p," HOLD=",heading.enabled);imu_send(b,p);return;
    }
    if(!strcmp(line,"imu raw")){
        p=imu_text(p,"RAW");p=imu_field(p," N=",(int32_t)imu.gyro_seq);p=imu_field(p," OK=",imu.valid);
        p=imu_field(p," AX=",imu.acc[0]);p=imu_field(p," AY=",imu.acc[1]);p=imu_field(p," AZ=",imu.acc[2]);
        p=imu_field(p," GX=",imu.gyro[0]);p=imu_field(p," GY=",imu.gyro[1]);p=imu_field(p," GZ=",imu.gyro[2]);
        p=imu_field(p," BX=",heading.bias_q16[0]);p=imu_field(p," BY=",heading.bias_q16[1]);
        p=imu_field(p," BZ=",heading.bias_q16[2]);imu_send(b,p);return;
    }
    if(!strcmp(line,"imu yaw")){
        p=imu_text(p,"YAW");p=imu_field(p," N=",(int32_t)imu.gyro_seq);
        p=imu_field(p," MD=",(int32_t)(heading.yaw_q16_md/65536));p=imu_field(p," RATE=",heading.rate_mdps);
        p=imu_field(p," REF=",heading.reference_md);p=imu_field(p," U=",heading.correction);
        p=imu_field(p," RUN=",heading.running);p=imu_field(p," FAULT=",heading.fault);imu_send(b,p);return;
    }
    if(!strcmp(line,"imu hold 0")){
        jdy_motor_rx(&motor,'~',rd(JDY_BASE,4));heading.enabled=false;
        p=imu_text(p,"IMU HOLD=0");imu_send(b,p);return;
    }
    if(!idle){imu_reject("ACTIVE");return;}
    if(!strncmp(line,"imu spi ",8)||!strncmp(line,"imu odr ",8)){
        if(heading.enabled||heading.calibrating){imu_reject("BUSY");return;}
        if(line[4]=='s'){
            static const char *const names[]={"50000","100000","250000","500000","1000000","2000000"};
            static const uint32_t hz[]={50000,100000,250000,500000,1000000,2000000};
            unsigned i;for(i=0;i<6;i++)if(!strcmp(line+8,names[i]))break;
            if(i==6){imu_reject("FREQUENCY");return;}
            uint32_t div=(rd(IMU_BASE,4)+2*hz[i]-1)/(2*hz[i]);
            jdy_motor_rx(&motor,'~',rd(JDY_BASE,4));
            wr(IMU_BASE,0x10,0x100);wr(IMU_BASE,0x14,div);
            if(rd(IMU_BASE,0x14)!=div){imu_reject("DIVIDER");return;}
            imu_start();p=imu_field(p,"IMU SPI=",(int32_t)(rd(IMU_BASE,4)/(2*div)));
        }else{
            if(strcmp(line+8,"100")&&strcmp(line+8,"200")){imu_reject("ODR");return;}
            imu_gyro_odr=line[8]=='1'?100U:200U;
            jdy_motor_rx(&motor,'~',rd(JDY_BASE,4));imu_start();p=imu_field(p,"IMU ODR=",(int32_t)imu_gyro_odr);
        }
        imu_send(b,p);return;
    }
    if(!strcmp(line,"imu regs")){
        /* 仅停机诊断时读取关键配置，不清FIFO或改传感器状态。 */
        static const uint8_t addresses[7]={0x0e,0x0f,0x10,0x11,0x3d,0x3e,0x0a};
        static const char *const labels[7]={" FIFO="," RANGE="," BW="," POWER="," FC0="," FC1="," INT="};
        p=imu_text(p,"IREGS");
        for(unsigned i=0;i<7;i++){
            uint8_t tx_reg[2]={(uint8_t)(addresses[i]|0x80),0},rx_reg[2];
            if(imu_spi(0,1,tx_reg,rx_reg,2,2000)){imu_reject("SPI");return;}
            p=imu_field(p,labels[i],rx_reg[1]);
        }
        imu_send(b,p);return;
    }
    if(!strcmp(line,"imu reset")){
        jdy_motor_rx(&motor,'~',rd(JDY_BASE,4));imu_start();p=imu_text(p,"IMU INIT=1");imu_send(b,p);return;
    }
    if(!strcmp(line,"imu bias")){
        if(!imu.valid){imu_reject("STALE");return;}
        jdy_motor_rx(&motor,'~',rd(JDY_BASE,4));heading_calibrate(&heading);
        p=imu_text(p,"IMU CALIBRATING=1000");imu_send(b,p);return;
    }
    if(!strncmp(line,"imu axis ",9)){
        if(!heading.calibrated||heading.enabled||strlen(line)<12||line[9]<'0'||line[9]>'2'||line[10]!=' '||
           (strcmp(line+11,"1")&&strcmp(line+11,"-1"))){imu_reject("AXIS");return;}
        heading.axis=(uint8_t)(line[9]-'0');heading.sign=line[11]=='-'?-1:1;heading.axis_confirmed=true;
        heading.have_yaw_previous=false;
        heading.yaw_q16_md=0;p=imu_text(p,"IMU AXIS_OK=1");imu_send(b,p);return;
    }
    if(!strcmp(line,"imu hold 1")){
        if(!imu.valid||!heading.calibrated||!heading.axis_confirmed||heading.fault||heading.calibrating||
           !motor.closed_loop||rd(MOTOR_BASE,0x0c)!=0x5f){imu_reject("NOT_READY");return;}
        for(unsigned c=1;c<=4;c++){
            jdy_motor_pid_t v;if(!pid_get(0,c,&v)||v.limit>4096||!(v.kp_q16|v.ki_tick_q16)||v.enc_inv!=(c>=3?1U:0U)){
                imu_reject("PID_LIMIT");return;
            }
        }
        heading.enabled=true;p=imu_text(p,"IMU HOLD=1");imu_send(b,p);return;
    }
    imu_reject("SYNTAX");
}
/* 功能：主循环采样和100Hz航向目标更新；无参数/返回，不刷新外部蓝牙租约。 */
static void imu_service(void)
{
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
        imu_irq_snapshot_t before,after;imu_irq_snapshot(&before);
        uint32_t da=before.acc-irq_last_acc,dg=before.gyro-irq_last_gyro;
        if(da){events.mask|=1;events.acc_us=before.at;if(da>1)irq_missed_acc+=da-1;}
        if(dg){events.mask|=2;events.gyro_us=before.gt;if(dg>1)irq_missed_gyro+=dg-1;}
        irq_last_acc=before.acc;irq_last_gyro=before.gyro;
        irq_candidate=imu;bmi088_irq_step(&irq_candidate,&events,before.now);
        imu_irq_snapshot(&after);
        if(irq_candidate.error>=0&&(((events.mask&1)&&after.acc!=before.acc)||((events.mask&2)&&after.gyro!=before.gyro))){
            ++irq_races;imu.new_gyro=false;imu.valid=false;
            // 保留旧有效样本，新事件下次服务；拒绝将旧边沿时间贴到更新后的寄存器数据上。
        }else imu=irq_candidate;
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
        heading_sample(&heading,imu.acc,imu.gyro,true,imu.gyro_us);
    }
    if(!imu.valid&&(heading.calibrating||heading.enabled)){
        heading.fault=true;heading.calibrated=false;heading.calibrating=false;
    }
    if(heading.enabled&&(heading.fault||!imu.valid)){
        jdy_motor_rx(&motor,'~',rd(JDY_BASE,4));heading.enabled=false;
    }
    if(heading.enabled&&heading.running&&motor.active&&(uint32_t)(now-last_heading_us)>=10000){
        int32_t target[4];last_heading_us=now;
        if(!heading_targets(&heading,heading_base,target)||!commit_targets(target)){
            heading.fault=true;jdy_motor_rx(&motor,'~',rd(JDY_BASE,4));heading.enabled=false;
        }
    }
    if(imu_receiving&&(uint32_t)(rd(JDY_BASE,4)-imu_rx_ms)>=1000){
        imu_receiving=false;imu_length=0;imu_dropping=false;imu_reject("TIMEOUT");
    }
}
/* 功能：分流以imu开头的独立ASCII命令；byte任意，now为毫秒；无返回，急停字节优先。 */
static void imu_receive(uint8_t byte,uint32_t now)
{
    if(byte=='~'){
        imu_receiving=false;imu_dropping=false;imu_length=0;jdy_motor_rx(&motor,byte,now);return;
    }
    if(!imu_receiving){
        if(byte=='i'&&motor.length==0&&!motor.dropping){imu_receiving=true;imu_length=0;imu_dropping=false;}
        else{jdy_motor_rx(&motor,byte,now);return;}
    }
    imu_rx_ms=now;
    if(byte=='\r')return;
    if(byte=='\n'){
        if(!imu_dropping){imu_line[imu_length]=0;imu_command(imu_line);}
        imu_receiving=false;imu_length=0;imu_dropping=false;return;
    }
    if(imu_dropping)return;
    if(byte<32||byte>126||imu_length>=sizeof(imu_line)-1){imu_dropping=true;imu_reject("LENGTH");return;}
    imu_line[imu_length++]=(char)byte;
}
