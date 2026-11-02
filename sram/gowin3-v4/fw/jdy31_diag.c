#include "jdy31_diag.h"
#include <string.h>
/* 功能：无标准库求字符串长度；p非空；返回字节数。 */
static size_t text_length(const char *p){size_t n=0;while(p[n])n++;return n;}
/* 功能：比较整个ASCII串；a/b不可空；相同返回0。 */
static int text_compare(const char *a,const char *b)
{while(*a&&*a==*b){a++;b++;}return (unsigned char)*a-(unsigned char)*b;}
/* 功能：比较前n字节；a/b不可空，n为字节数；相同返回0。 */
static int prefix_compare(const char *a,const char *b,size_t n)
{while(n){if(*a!=*b)return 1;if(!*a)return 0;a++;b++;n--;}return 0;}
/* 功能：撤销租约并更换代号；s不可空；无返回。 */
static void stop(jdy_diag_t *s)
{s->io.guard(s->io.ctx,JDY_G_STOP,0);s->active=false;s->key=0;s->level=0;if(++s->epoch==0)s->epoch=1;}
/* 功能：追加ASCII字符串；p/t不可空且空间充分；返回末尾指针。 */
static char *append(char *p,const char *t){while(*t)*p++=*t++;return p;}
/* 功能：格式化uint32；p至少10字节，v为数值；返回末尾指针。 */
static char *decimal(char *p,uint32_t v)
{char b[10];unsigned n=0;do{b[n++]=(char)('0'+v%10U);v/=10U;}while(v);while(n)*p++=b[--n];return p;}
/* 功能：严格解析uint32；p/v不可空；成功返回true并推进p。 */
static bool number(const char **p,uint32_t *v)
{
    uint32_t n=0;unsigned digits=0;
    while(**p>='0'&&**p<='9'){
        unsigned d=(unsigned)(**p-'0');if(n>(UINT32_MAX-d)/10U)return false;
        n=n*10U+d;(*p)++;digits++;
    }
    *v=n;return digits!=0;
}
void jdy_diag_transport_fault(jdy_diag_t *s)
{if(!s->blocked)stop(s);s->blocked=true;s->length=0;}
/* 功能：发送完整应答；s/text不可空；无返回，拥塞立即锁存。 */
static void reply(jdy_diag_t *s,const char *text)
{if(!s->io.write(s->io.ctx,text,text_length(text)))jdy_diag_transport_fault(s);}
/* 功能：发送实际保护状态；s不可空；无返回。 */
static void status(jdy_diag_t *s)
{
    char b[160],*p=b;uint32_t state=s->io.state(s->io.ctx);
    p=append(p,"DIAG MOTOR=0 ACTIVE=");p=decimal(p,state&1U);
    p=append(p," FAULT=");p=decimal(p,(state>>1)&1U);
    p=append(p," EPOCH=");p=decimal(p,s->epoch);
    p=append(p," SEQ=");p=decimal(p,s->sequence);
    p=append(p," RX=");p=decimal(p,s->accepted);
    p=append(p," BAD=");p=decimal(p,s->rejected);p=append(p,"\r\n");*p=0;reply(s,b);
}
/* 功能：回读已接收的遥控意图；s不可空；无返回，仍无电机输出。 */
static void keys(jdy_diag_t *s)
{
    char b[160],*p=b;uint32_t state=s->io.state(s->io.ctx);
    p=append(p,"KEYS MOTOR=0 ACTIVE=");p=decimal(p,state&1U);
    p=append(p," FAULT=");p=decimal(p,(state>>1)&1U);
    p=append(p," EPOCH=");p=decimal(p,s->epoch);
    p=append(p," SEQ=");p=decimal(p,s->sequence);
    p=append(p," KEY=");p=decimal(p,s->key);
    p=append(p," LEVEL=");p=decimal(p,s->level);p=append(p,"\r\n");*p=0;reply(s,b);
}
/* 功能：执行单行命令；s/t不可空；无返回，无任何运动控制。 */
static void command(jdy_diag_t *s,const char *t)
{
    const char *p;uint32_t epoch,seq,key,level;
    if(!text_compare(t,"ping")){reply(s,"PONG JDY31 MOTOR=0\r\n");return;}
    if(!text_compare(t,"status")){status(s);return;}
    if(!text_compare(t,"keys")){keys(s);return;}
    if(!text_compare(t,"stop")){stop(s);reply(s,"STOPPED MOTOR=0\r\n");return;}
    if(!text_compare(t,"clear")){stop(s);s->sequence=0;s->io.guard(s->io.ctx,JDY_G_CLEAR,0);status(s);return;}
    if(!prefix_compare(t,"key ",4)){
        p=t+4;
        if(!number(&p,&epoch)||*p++!=' '||!number(&p,&seq)||*p++!=' '||
           !number(&p,&key)||*p++!=' '||!number(&p,&level)||*p||key<1||key>6||level<1||level>3){
            s->rejected++;stop(s);return;
        }
        if(epoch!=s->epoch||seq==0||(s->active&&seq<=s->sequence)||(!s->active&&seq!=1)){
            s->rejected++;return; /* 过期手势不能清故障，只有新按下的当前会话seq=1允许开始。 */
        }
        if(!s->active){
            s->io.guard(s->io.ctx,JDY_G_CLEAR,0);s->io.guard(s->io.ctx,JDY_G_ARM,0);
            s->active=s->io.state(s->io.ctx)==1;
        }
        if(s->active&&s->io.state(s->io.ctx)==1){
            s->sequence=seq;s->key=key;s->level=level;s->accepted++;
            s->io.guard(s->io.ctx,JDY_G_KICK,seq);
        }else stop(s);
        return;
    }
    if(!prefix_compare(t,"arm ",4)){
        p=t+4;
        if(number(&p,&epoch)&&*p==0&&epoch==s->epoch&&!s->active&&s->io.state(s->io.ctx)==0){
            s->sequence=0;s->io.guard(s->io.ctx,JDY_G_ARM,0);
            s->active=s->io.state(s->io.ctx)==1;
            reply(s,s->active?"ARMED TEST_ONLY MOTOR=0\r\n":"FAULT\r\n");return;
        }
    }else if(!prefix_compare(t,"hb ",3)){
        p=t+3;
        if(number(&p,&epoch)&&*p==' '){p++;
            if(number(&p,&seq)&&*p==0&&epoch==s->epoch&&seq>s->sequence&&s->active&&s->io.state(s->io.ctx)==1){
                s->sequence=seq;s->accepted++;s->io.guard(s->io.ctx,JDY_G_KICK,seq);return;
            }
        }
        s->rejected++;return; /* 重复、旧会话或非法心跳不续期，不逐包应答。 */
    }
    s->rejected++;stop(s);
    /* 模块状态日志不解析为连接凭据；任何此类输出都撤销测试，且不回写模块。 */
    if(*t!='+')reply(s,"REJECTED MOTOR=0\r\n");
}
void jdy_diag_init(jdy_diag_t *s,const jdy_diag_io_t *io,uint32_t now)
{memset(s,0,sizeof(*s));s->io=*io;s->epoch=1;s->last_rx=now;s->io.guard(s->io.ctx,JDY_G_STOP,0);}
void jdy_diag_poll(jdy_diag_t *s,uint32_t now)
{
    if(s->blocked)return;
    if(s->active&&s->io.state(s->io.ctx)!=1)stop(s);
    if(s->length&&(uint32_t)(now-s->last_rx)>=1000U){
        s->length=0;s->dropping=true;stop(s);
    }
}
void jdy_diag_rx(jdy_diag_t *s,uint8_t byte,uint32_t now)
{
    jdy_diag_poll(s,now);s->last_rx=now;if(s->blocked)return;
    /* 新连接先发~：立即丢弃半行并撤销旧租约，不依赖四针板未引出的STATE。 */
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
