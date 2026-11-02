#include "tof4_array.h"
#include <string.h>
#define UART UINT32_C(0x20002000)
#define TOF UINT32_C(0x20005000)
static vl53l1x_array_t sensors; /* 四路固定次序前、后、左、右；毫米与微秒单位。 */
static bool running;
static uint8_t channel_map[4]={0,1,2,3}; /* 软件槽位到物理X1..X4，静态诊断可选择子集。 */
static uint8_t selected_mask;
static uint32_t last_step,rx_stamp,max_service_us;
static uint32_t bus_error_reg,bus_error_addr,bus_error_stage,bus_error_status;
static char line[48],queue[512]; /* 有界命令和发送队列，静态分配。 */
static unsigned length,head,tail;
static bool dropping;
/* 功能：读取无缓存软核MMIO；b为基址、o为对齐偏移；返回32位值。 */
static uint32_t rd(uint32_t b,unsigned o){return *(volatile uint32_t *)(uintptr_t)(b+o);}
/* 功能：写MMIO；b/o为合法地址、v为值；无返回。仅BSP可直接访问寄存器。 */
static void wr(uint32_t b,unsigned o,uint32_t v){*(volatile uint32_t *)(uintptr_t)(b+o)=v;}
/* 功能：执行一个I2C字节或STOP；cmd是硬件位图，v为发送字节，out可空；返回0/-1。 */
static int byte_io(unsigned cmd,uint8_t v,uint8_t *out)
{
    uint32_t t=rd(TOF,8),s;
    wr(TOF,0x14,v);wr(TOF,0x10,cmd);
    do{
        s=rd(TOF,4);
        if((s&8U)||(uint32_t)(rd(TOF,8)-t)>2000U){bus_error_status=s;wr(TOF,0x10,256);return -1;}
    }while(!(s&2U));
    if(out)*out=(uint8_t)rd(TOF,0x18);
    if((cmd&8U)&&(s&4U)){bus_error_status=s;return -1;}
    return 0;
}
/* 功能：有界多字节I2C事务；addr为7位，reg为16位，p非空，n=1..32，read选择方向；返回0/-1。 */
static int transfer(uint8_t addr,uint16_t reg,uint8_t *p,size_t n,bool read,uint32_t timeout)
{
    uint32_t t=rd(TOF,8);int rc=-1;
    if(!p||!n||n>32||addr<8||addr>0x77||timeout<2000)return -1;
    bus_error_addr=addr;bus_error_reg=reg;bus_error_stage=1;bus_error_status=0;
    if((rd(TOF,4)&192U)!=192U)goto fail;
    if(byte_io(9,(uint8_t)(addr<<1),0))goto fail;
    bus_error_stage=2;if(byte_io(8,(uint8_t)(reg>>8),0))goto fail;
    bus_error_stage=3;if(byte_io(8,(uint8_t)reg,0))goto fail;
    if(read){bus_error_stage=4;if(byte_io(9,(uint8_t)((addr<<1)|1),0))goto fail;}
    for(size_t i=0;i<n;i++){
        bus_error_stage=5+(uint32_t)i;
        if((uint32_t)(rd(TOF,8)-t)>=timeout)goto fail;
        if(byte_io(read?(i+1==n?22:4):(i+1==n?10:8),read?0:p[i],read?&p[i]:0))goto fail;
    }
    bus_error_stage=0;return 0;
fail:
    /* NACK、仲裁或超时后释放总线；不隐式重新发送失败事务。 */
    (void)byte_io(2,0,0);wr(TOF,0x10,256);return rc;
}
/* 功能：适配sensor_bus读取；ctx未用，其余范围同transfer；返回0/-1。 */
static int read_i2c(void *ctx,uint8_t a,uint16_t r,uint8_t *p,size_t n,uint32_t t)
{(void)ctx;return transfer(a,r,p,n,true,t);}
/* 功能：适配sensor_bus写入；ctx未用，p只读不可空；返回0/-1。 */
static int write_i2c(void *ctx,uint8_t a,uint16_t r,const uint8_t *p,size_t n,uint32_t t)
{uint8_t copy[32];(void)ctx;if(!p||n>32)return -1;memcpy(copy,p,n);return transfer(a,r,copy,n,false,t);}
/* 功能：控制四路XSHUT；i=0..3、release为释放0/Z；返回0或参数错误-1。 */
static int xshut(void *ctx,uint8_t i,bool release)
{(void)ctx;if(i>3)return -1;uint32_t bit=1U<<channel_map[i],mask=rd(TOF,12);wr(TOF,12,release?mask|bit:mask&~bit);return 0;}
/* 功能：发送完整单行；s不可空且长度<512；队列不足丢弃整行，无返回。 */
static void send_line(const char *s)
{size_t n=strlen(s);if(n>(tail+sizeof(queue)-head-1)%sizeof(queue))return;while(*s){queue[head]=*s++;head=(head+1)%sizeof(queue);}}
/* 功能：追加文本；p和s非空，调用方提供足够空间；返回尾指针。 */
static char *text(char *p,const char *s){while(*s)*p++=*s++;return p;}
/* 功能：追加十进制字段；p/key非空、v为有符号32位；返回尾指针。 */
static char *field(char *p,const char *key,int32_t v)
{char b[10];unsigned n=0;uint32_t u=(uint32_t)v;p=text(p,key);if(v<0){*p++='-';u=0-u;}do{b[n++]=(char)('0'+u%10);u/=10;}while(u);while(n)*p++=b[--n];return p;}
/* 功能：执行完整诊断命令；s为NUL结尾文本；无返回，从不产生电机命令。 */
static void command(const char *s)
{
    char b[320],*p=b;
    if(!strcmp(s,"ping")){send_line("PONG TOF4 V=1 MOTOR=DISABLED\r\n");return;}
    if(strlen(s)==11&&!strncmp(s,"tof probe ",10)&&s[10]>='1'&&s[10]<='4'){
        /* 独立通道诊断只读ID，等待上限20ms；只在硬连禁能的本诊断固件使用。 */
        unsigned i=(unsigned)(s[10]-'1');uint8_t id[2]={0};running=false;
        if(sensors.bus.xshut)(void)vl53l1x_array_stop(&sensors);
        wr(TOF,12,0);wr(TOF,0x10,256);uint32_t t=rd(TOF,8);
        while((uint32_t)(rd(TOF,8)-t)<2000U){}
        wr(TOF,12,1U<<i);t=rd(TOF,8);
        while((uint32_t)(rd(TOF,8)-t)<20000U){}
        int rc=read_i2c(0,0x29,0x010f,id,2,20000);wr(TOF,12,0);
        p=text(p,"PROBE");p=field(p," CH=",i+1);p=field(p," ERR=",rc);
        p=field(p," ID=",((uint32_t)id[0]<<8)|id[1]);p=field(p," STAGE=",bus_error_stage);
        p=field(p," STATUS=",bus_error_status);p=text(p,"\r\n");*p=0;send_line(b);return;
    }
    if(!strcmp(s,"tof start")||!strncmp(s,"tof start ",10)){
        unsigned mask=15;
        if(s[9]){
            if(s[10]<'1'||s[10]>'9'){send_line("REJECTED TOF4\r\n");return;}
            mask=(unsigned)(s[10]-'0');
            if(s[11]){if(s[11]<'0'||s[11]>'9'||s[12]){send_line("REJECTED TOF4\r\n");return;}mask=mask*10+(unsigned)(s[11]-'0');}
            if(mask>15){send_line("REJECTED TOF4\r\n");return;}
        }
        running=false;wr(TOF,12,0);
        sensor_bus_t bus={0};bus.i2c_read=read_i2c;bus.i2c_write=write_i2c;bus.xshut=xshut;
        tof_config_t cfg={.count=0,.distance_mode=2,
            .budget_ms=50,.period_ms=100,.fresh_us=500000,.io_mode=TOF_IO_AVDD};
        selected_mask=(uint8_t)mask;
        for(unsigned i=0;i<4;i++)if(mask&(1U<<i)){channel_map[cfg.count]=(uint8_t)i;cfg.address7[cfg.count++]=(uint8_t)(0x30+i);}
        wr(TOF,0x10,256);max_service_us=0;
        running=vl53l1x_array_start(&sensors,&bus,&cfg,rd(TOF,8))==SENSOR_PENDING;
        send_line(running?"TOF START=1\r\n":"TOF START=0\r\n");return;
    }
    if(!strcmp(s,"tof stop")){running=false;if(sensors.bus.xshut)(void)vl53l1x_array_stop(&sensors);wr(TOF,12,0);send_line("TOF STOP=1\r\n");return;}
    if(!strcmp(s,"tof")){
        p=text(p,"TOF V=1");p=field(p," RUN=",running);p=field(p," READY=",vl53l1x_array_ready(&sensors));
        p=field(p," STATE=",sensors.state);p=field(p," INDEX=",sensors.index);p=field(p," ERR=",sensors.error);
        p=field(p," X=",rd(TOF,12));p=field(p," BUS=",rd(TOF,4));p=field(p," MAXUS=",max_service_us);
        p=field(p," ADDR=",bus_error_addr);p=field(p," REG=",bus_error_reg);p=field(p," STAGE=",bus_error_stage);p=field(p," STATUS=",bus_error_status);p=field(p," MASK=",selected_mask);
    }else if(strlen(s)==5&&!strncmp(s,"tof ",4)&&s[4]>='1'&&s[4]<='4'){
        unsigned i=(unsigned)(s[4]-'1');tof_sample_t absent={0},*v=&absent;vl53l1x_array_health(&sensors,rd(TOF,8));
        if(selected_mask&(1U<<i))for(unsigned k=0;k<sensors.cfg.count;k++)if(channel_map[k]==i)v=&sensors.sample[k];
        p=text(p,"RANGE");p=field(p," CH=",i+1);p=field(p," MM=",v->distance_mm);p=field(p," VALID=",v->validity);
        p=field(p," STATUS=",v->range_status);p=field(p," RAW=",v->raw_status);p=field(p," SEQ=",v->sequence);
        p=field(p," STREAM=",v->stream_count);p=field(p," AGE=",v->received?(int32_t)(rd(TOF,8)-v->timestamp_us):-1);p=field(p," ERR=",v->error);
    }else{send_line("REJECTED TOF4\r\n");return;}
    p=text(p,"\r\n");*p=0;send_line(b);
}
/* 功能：裸机主循环，每轮推进传感器状态机及有界UART；无参数、不返回。 */
int main(void)
{
    wr(TOF,12,0);
    for(;;){
        uint32_t now=rd(TOF,8),ch=rd(UART,0x14);
        if(ch<256){
            if((uint32_t)(now-rx_stamp)>500000U){length=0;dropping=false;}rx_stamp=now;
            if(ch=='\n'){if(!dropping){line[length]=0;command(line);}length=0;dropping=false;}
            else if(ch!='\r'){if(length+1<sizeof(line))line[length++]=(char)ch;else dropping=true;}
        }
        if(head!=tail&&!(rd(UART,0x18)&256U)){wr(UART,0x18,(uint8_t)queue[tail]);tail=(tail+1)%sizeof(queue);}
        if(running&&(uint32_t)(now-last_step)>=1000){
            uint32_t t=rd(TOF,8);last_step=t;(void)vl53l1x_array_step(&sensors,t);
            uint32_t elapsed=rd(TOF,8)-t;if(elapsed>max_service_us)max_service_us=elapsed;
            if(sensors.error<0)running=false;
        }
    }
}
