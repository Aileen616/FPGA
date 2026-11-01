#include "jdy31_motor.h"
#include "perf_monitor.h"
#include <string.h>

/* 功能：取得字符串字节数；p不可空；返回长度。 */
static size_t text_length(const char *p){size_t n=0;while(p[n])n++;return n;}
/* 功能：比较完整ASCII串；a/b不可空；相同返回0，否则非0。 */
static int text_compare(const char *a,const char *b)
{while(*a&&*a==*b){a++;b++;}return (unsigned char)*a-(unsigned char)*b;}
/* 功能：比较前n字节；a/b不可空，n为字节数；相同返回0，否则1。 */
static int prefix_compare(const char *a,const char *b,size_t n)
{while(n){if(*a!=*b)return 1;if(!*a)return 0;a++;b++;n--;}return 0;}
/* 功能：停止并废弃旧手势；s不可空；无返回，配置保留但不会自动恢复运动。 */
static void stop(jdy_motor_t *s)
{
    s->io.guard(s->io.ctx,JDY_M_STOP,0);
    s->active=false;s->key=0;s->level=0;
    if(++s->epoch==0)s->epoch=1;
}
/* 功能：追加字符串；p/t不可空且空间充分；返回末尾指针。 */
static char *append(char *p,const char *t){while(*t)*p++=*t++;return p;}
/* 功能：格式化无符号数；p至少有10字节可写空间，v为uint32；返回末尾指针。 */
static char *decimal(char *p,uint32_t v)
{char b[10];unsigned n=0;do{b[n++]=(char)('0'+v%10U);v/=10U;}while(v);while(n)*p++=b[--n];return p;}
/* 功能：严格解析十进制uint32；p/v不可空；成功返回true并推进指针，溢出返回false。 */
static bool number(const char **p,uint32_t *v)
{
    uint32_t n=0;unsigned digits=0;
    while(**p>='0'&&**p<='9'){
        unsigned d=(unsigned)(**p-'0');if(n>(UINT32_MAX-d)/10U)return false;
        n=n*10U+d;(*p)++;digits++;
    }
    *v=n;return digits!=0;
}
void jdy_motor_transport_fault(jdy_motor_t *s)
{if(!s->blocked)stop(s);s->blocked=true;s->length=0;}
/* 功能：发送完整应答；s/text不可空；无返回，队列满会锁存通信故障。 */
static void reply(jdy_motor_t *s,const char *text)
{if(!s->io.write(s->io.ctx,text,text_length(text)))jdy_motor_transport_fault(s);}
/* 功能：回读实际租约和配置；s不可空；无返回，ACTIVE不代表实测车轮运动。 */
static void keys(jdy_motor_t *s)
{
    char b[192],*p=b;uint32_t state=s->io.state(s->io.ctx);
    p=append(p,"KEYS MOTOR=3 ACTIVE=");p=decimal(p,state&1U);
    p=append(p," FAULT=");p=decimal(p,(state>>1)&1U);
    p=append(p," EPOCH=");p=decimal(p,s->epoch);
    p=append(p," SEQ=");p=decimal(p,s->sequence);
    p=append(p," KEY=");p=decimal(p,s->key);
    p=append(p," LEVEL=");p=decimal(p,s->level);
    p=append(p," CFG=");p=decimal(p,s->configured?1U:0U);
    p=append(p," MASK=");p=decimal(p,s->wheel_mask);
    p=append(p," INV=");p=decimal(p,s->invert);
    p=append(p,"\r\n");*p=0;reply(s,b);
}
/* 功能：执行完整单行命令；s/t不可空；无返回，仅合法key可提交驱动。 */
static void command(jdy_motor_t *s,const char *t)
{
    const char *p;uint32_t epoch,seq,key,level,mask,invert;
    if(!text_compare(t,"perf")){perf_monitor_request();return;}
    if(!text_compare(t,"ping")){reply(s,"PONG JDY31 MOTOR=3\r\n");return;}
    if(!text_compare(t,"keys")||!text_compare(t,"status")){keys(s);return;}
    if(!text_compare(t,"stop")){stop(s);reply(s,"STOPPED MOTOR=3\r\n");return;}
    if(!prefix_compare(t,"cfg ",4)){
        p=t+4;
        if(!number(&p,&mask)||*p++!=' '||!number(&p,&invert)||*p||
           mask<1||mask>15||invert>15||s->active||(s->io.state(s->io.ctx)&1U)){
            s->rejected++;stop(s);reply(s,"REJECTED MOTOR=3\r\n");return;
        }
        /* 先撤销代号，避免配置前排队的旧帧使用新方向开始运动。 */
        stop(s);s->configured=false;s->wheel_mask=0;s->invert=0;
        if(s->io.configure(s->io.ctx,mask,invert)){
            s->configured=true;s->wheel_mask=mask;s->invert=invert;s->sequence=0;
        }else s->rejected++;
        keys(s);return;
    }
    if(!prefix_compare(t,"key ",4)){
        p=t+4;
        if(!number(&p,&epoch)||*p++!=' '||!number(&p,&seq)||*p++!=' '||
           !number(&p,&key)||*p++!=' '||!number(&p,&level)||*p||key<1||key>10||level<1||level>3){
            s->rejected++;stop(s);return;
        }
        if(!s->configured||epoch!=s->epoch||seq==0||
           (s->active&&seq<=s->sequence)||(!s->active&&seq!=1)){
            s->rejected++;return;
        }
        /* 同一次按住只允许同方向同档续期；换向必须先停车并取得新代号。 */
        if(s->active&&(key!=s->key||level!=s->level)){
            s->rejected++;stop(s);return;
        }
        bool start=!s->active;
        if(start){
            s->io.guard(s->io.ctx,JDY_M_CLEAR,0);s->io.guard(s->io.ctx,JDY_M_ARM,0);
        }
        /* drive同时提交独立运动保护；普通查询和旧hb绝不刷新运动寿命。 */
        uint32_t state=s->io.state(s->io.ctx);
        if((start?(state&7U)==4U:(state&3U)==1U)&&
           s->io.drive(s->io.ctx,seq,key,level)&&(s->io.state(s->io.ctx)&3U)==1U){
            s->active=true;
            s->sequence=seq;s->key=key;s->level=level;s->accepted++;
        }else {s->rejected++;stop(s);}
        return;
    }
    /* 旧arm/hb/clear仅属于无电机诊断；此固件拒绝并停止，不能绕过新按下。 */
    s->rejected++;stop(s);
    if(*t!='+')reply(s,"REJECTED MOTOR=3\r\n");
}
void jdy_motor_init(jdy_motor_t *s,const jdy_motor_io_t *io,uint32_t now)
{memset(s,0,sizeof(*s));s->io=*io;s->epoch=1;s->last_rx=now;s->io.guard(s->io.ctx,JDY_M_STOP,0);}
void jdy_motor_poll(jdy_motor_t *s,uint32_t now)
{
    if(s->blocked)return;
    if(s->active&&(s->io.state(s->io.ctx)&3U)!=1U)stop(s);
    if(s->length&&(uint32_t)(now-s->last_rx)>=1000U){
        s->length=0;s->dropping=true;stop(s);
    }
}
void jdy_motor_rx(jdy_motor_t *s,uint8_t byte,uint32_t now)
{
    jdy_motor_poll(s,now);s->last_rx=now;if(s->blocked)return;
    /* 波浪号单字节立即撤销手势，优先于半包和正常换行解析。 */
    if(byte=='~'){stop(s);s->length=0;s->dropping=false;return;}
    if(byte=='\r')return;
    if(byte=='\n'){
        if(!s->dropping&&s->length){s->line[s->length]=0;command(s,s->line);}
        s->length=0;s->dropping=false;return;
    }
    if(s->dropping)return;
    if(byte<32||byte>126||s->length>=sizeof(s->line)-1U){
        s->rejected++;s->length=0;s->dropping=true;stop(s);return;
    }
    s->line[s->length++]=(char)byte;
}
