/* 功能：在E8E3主循环中增量接入前/右/左ToF；依赖同编译单元的rd/wr、imu_service和enqueue。
 * 初始化调用tof_start；主循环调用tof_service；UART先交给tof_receive，不消费的字节仍走原协议。
 * 不配置STM32外设；BX72复用官方PicoRV32和既有100kHz开漏I2C硬件，四轮核心不变。
 */
#include "sensor_joint_array.h"
#define TOF_BASE UINT32_C(0x20005000)
static vl53l1x_array_t joint_tof; /* 三个槽位依次物理X2前、X3右、X4左；距离mm，时间us。 */
static uint32_t tof_last_step,tof_max_us,tof_error_reg,tof_error_addr,tof_error_stage,tof_error_status;
static char tof_line[48]; /* 固定长度命令，避免动态内存。 */
static unsigned tof_length;
static bool tof_receiving,tof_dropping;
static uint32_t tof_rx_ms;
static uint32_t tof_last_imu_us; /* I2C字节间最多每1ms服务一次IMU，避免无事件快照重复占满CPU。 */
/* 功能：执行一个I2C字节；cmd为既有命令位，v为发送字节，out可空；返回0成功/-1失败。 */
static int tof_byte(unsigned cmd,uint8_t v,uint8_t *out)
{
    /* I2C处于字节间隙时消费IMU事件，避免17字节ToF读包连续阻塞SPI。 */
    uint32_t imu_now=rd(IMU_BASE,8);
    if((uint32_t)(imu_now-tof_last_imu_us)>=1000U){tof_last_imu_us=imu_now;imu_service();}
    uint32_t t=rd(TOF_BASE,8),s;
    wr(TOF_BASE,0x14,v);wr(TOF_BASE,0x10,cmd);
    do{
        s=rd(TOF_BASE,4);
        if((s&8U)||(uint32_t)(rd(TOF_BASE,8)-t)>2000U){
            tof_error_status=s;wr(TOF_BASE,0x10,256);return -1;
        }
    }while(!(s&2U));
    if(out)*out=(uint8_t)rd(TOF_BASE,0x18);
    if((cmd&8U)&&(s&4U)){tof_error_status=s;return -1;}
    return 0;
}
/* 功能：执行既有16位寄存器I2C事务；p非空，n=1..32，addr为7位；返回0/-1，失败不重放。 */
static int tof_transfer(uint8_t addr,uint16_t reg,uint8_t *p,size_t n,bool read,uint32_t timeout)
{
    if(!p||!n||n>32||addr<8||addr>0x77||timeout<2000)return -1;
    uint32_t t=rd(TOF_BASE,8);
    tof_error_addr=addr;tof_error_reg=reg;tof_error_stage=1;tof_error_status=0;
    if((rd(TOF_BASE,4)&192U)!=192U)goto fail;
    if(tof_byte(9,(uint8_t)(addr<<1),0))goto fail;
    tof_error_stage=2;if(tof_byte(8,(uint8_t)(reg>>8),0))goto fail;
    tof_error_stage=3;if(tof_byte(8,(uint8_t)reg,0))goto fail;
    if(read){tof_error_stage=4;if(tof_byte(9,(uint8_t)((addr<<1)|1),0))goto fail;}
    for(size_t i=0;i<n;i++){
        tof_error_stage=5+(uint32_t)i;
        if((uint32_t)(rd(TOF_BASE,8)-t)>=timeout)goto fail;
        if(tof_byte(read?(i+1==n?22:4):(i+1==n?10:8),read?0:p[i],read?&p[i]:0))goto fail;
    }
    tof_error_stage=0;return 0;
fail:
    (void)tof_byte(2,0,0);wr(TOF_BASE,0x10,256);return -1;
}
/* 功能：适配读回调；ctx不用可空，p非空，其余范围同tof_transfer；返回0/-1。 */
static int tof_read(void *ctx,uint8_t a,uint16_t r,uint8_t *p,size_t n,uint32_t t)
{(void)ctx;return tof_transfer(a,r,p,n,true,t);}
/* 功能：适配写回调；ctx可空，p指向n=1..32字节；返回0/-1，不修改调用方数组。 */
static int tof_write(void *ctx,uint8_t a,uint16_t r,const uint8_t *p,size_t n,uint32_t t)
{uint8_t copy[32];(void)ctx;if(!p||!n||n>32)return -1;memcpy(copy,p,n);return tof_transfer(a,r,copy,n,false,t);}
/* 功能：操作槽位0..2对应X2/X3/X4；ctx可空，release选择释放0/Z；成功0，否则-1。 */
static int tof_shutdown(void *ctx,uint8_t i,bool release)
{(void)ctx;if(i>2)return -1;uint32_t bit=1U<<(i+1),v=rd(TOF_BASE,12);wr(TOF_BASE,12,release?(v|bit)&14U:v&~bit&14U);return 0;}
/* 功能：关闭三路测距并拉低全部XSHUT；无参数/返回，保留故障记录。 */
static void tof_stop(void)
{tof_running=false;if(joint_tof.bus.xshut)(void)vl53l1x_array_stop(&joint_tof);wr(TOF_BASE,12,0);}
/* 功能：仅停止态启动固定掩码14；无参数；返回true已启动，否则false，默认不自动调用。 */
static bool tof_start(void)
{
    if(motor.active||heading.enabled||heading.calibrating||(rd(MOTOR_BASE,4)&1U)||(rd(JDY_BASE,8)&1U))return false;
    tof_stop();wr(TOF_BASE,0x10,256);tof_max_us=0;
    const sensor_bus_t bus={.i2c_read=tof_read,.i2c_write=tof_write,.xshut=tof_shutdown};
    const tof_config_t cfg={.count=3,.address7={0x31,0x32,0x33},.distance_mode=2,
        .budget_ms=50,.period_ms=100,.fresh_us=500000,.io_mode=TOF_IO_AVDD};
    tof_running=vl53l1x_array_start(&joint_tof,&bus,&cfg,rd(TOF_BASE,8))==SENSOR_PENDING;
    return tof_running;
}
/* 功能：处理固定ToF命令；s非空NUL结尾；无返回，拒绝其它掩码和后方X1。 */
static void tof_command(const char *s)
{
    char b[320],*p=b;
    if(strlen(s)==10&&!strncmp(s,"tof regs ",9)&&s[9]>='2'&&s[9]<='4'){
        /* 只读实际GPIO/模式/周期配置；不清中断、不重启测距，便于定位初始化后的停帧。 */
        unsigned i=(unsigned)(s[9]-'2');uint8_t v[4];uint8_t addr=(uint8_t)(0x31+i);
        static const uint16_t regs[7]={0x0030,0x0031,0x0087,0x006c,0x00de,0x005e,0x0061};
        static const uint8_t sizes[7]={1,1,1,4,2,2,2};
        static const char *const names[7]={" CTRL="," GPIO="," MODE="," IMP="," PLL="," TA="," TB="};
        p=imu_text(p,"TCFG");p=imu_field(p," CH=",i+2);
        for(unsigned k=0;k<7;k++){
            if(tof_read(0,addr,regs[k],v,sizes[k],20000)){(void)enqueue(0,"TOF REGERR=1\r\n",14);return;}
            uint32_t value=0;for(unsigned j=0;j<sizes[k];j++)value=(value<<8)|v[j];
            p=imu_field(p,names[k],(int32_t)value);
        }
        p=imu_field(p," CLR=",joint_tof.clear_confirmed[i]);p=imu_field(p," PENDING=",joint_tof.clear_still_ready[i]);
        imu_send(b,p);return;
    }
    if(!strcmp(s,"tof start")||!strcmp(s,"tof start 14")){
        const char *reply=tof_start()?"TOF START=1\r\n":"TOF START=0\r\n";
        (void)enqueue(0,reply,strlen(reply));return;
    }
    if(!strcmp(s,"tof stop")){tof_stop();(void)enqueue(0,"TOF STOP=1\r\n",12);return;}
    if(!strcmp(s,"tof")){
        p=imu_text(p,"TOF V=1");p=imu_field(p," RUN=",tof_running);p=imu_field(p," READY=",vl53l1x_array_ready(&joint_tof));
        p=imu_field(p," STATE=",joint_tof.state);p=imu_field(p," INDEX=",joint_tof.index);p=imu_field(p," ERR=",joint_tof.error);
        p=imu_field(p," X=",rd(TOF_BASE,12));p=imu_field(p," BUS=",rd(TOF_BASE,4));p=imu_field(p," MAXUS=",tof_max_us);
        p=imu_field(p," ADDR=",tof_error_addr);p=imu_field(p," REG=",tof_error_reg);p=imu_field(p," STAGE=",tof_error_stage);
        p=imu_field(p," STATUS=",tof_error_status);p=imu_field(p," MASK=",14);
    }else if(strlen(s)==5&&!strncmp(s,"tof ",4)&&s[4]>='2'&&s[4]<='4'){
        unsigned i=(unsigned)(s[4]-'2');vl53l1x_array_health(&joint_tof,rd(TOF_BASE,8));tof_sample_t *v=&joint_tof.sample[i];
        p=imu_text(p,"RANGE");p=imu_field(p," CH=",i+2);p=imu_field(p," MM=",v->distance_mm);p=imu_field(p," VALID=",v->validity);
        p=imu_field(p," STATUS=",v->range_status);p=imu_field(p," RAW=",v->raw_status);p=imu_field(p," SEQ=",v->sequence);
        p=imu_field(p," STREAM=",v->stream_count);p=imu_field(p," AGE=",v->received?(int32_t)(rd(TOF_BASE,8)-v->timestamp_us):-1);p=imu_field(p," ERR=",v->error);
    }else{(void)enqueue(0,"REJECTED TOF4\r\n",15);return;}
    imu_send(b,p);
}
/* 功能：分流tof字符流；byte任意，now为毫秒；返回true已消费，false交给原BMI/电机协议。 */
static bool tof_receive(uint8_t byte,uint32_t now)
{
    if(byte=='~'){tof_receiving=false;tof_length=0;tof_dropping=false;return false;}
    if(!tof_receiving){
        if(byte!='t'||imu_receiving||motor.length||motor.dropping)return false;
        tof_receiving=true;tof_length=0;tof_dropping=false;
    }
    tof_rx_ms=now;
    if(byte=='\r')return true;
    if(byte=='\n'){
        if(!tof_dropping){tof_line[tof_length]=0;tof_command(tof_line);}
        tof_receiving=false;tof_length=0;tof_dropping=false;return true;
    }
    if(byte<32||byte>126||tof_length>=sizeof(tof_line)-1)tof_dropping=true;
    else if(!tof_dropping)tof_line[tof_length++]=(char)byte;
    return true;
}
/* 功能：每毫秒尝试推进一路ToF状态；无参数/返回，字节之间继续服务IMU，初始化失败不重试。 */
static void tof_service(void)
{
    uint32_t now=rd(TOF_BASE,8);
    if(tof_receiving&&(uint32_t)(rd(JDY_BASE,4)-tof_rx_ms)>=1000){tof_receiving=false;tof_length=0;tof_dropping=false;}
    if(tof_running&&(uint32_t)(now-tof_last_step)>=1000){
        tof_last_step=now;(void)vl53l1x_array_step(&joint_tof,now);
        uint32_t dt=rd(TOF_BASE,8)-now;if(dt>tof_max_us)tof_max_us=dt;
        if(joint_tof.error<0)tof_running=false;
    }
}
