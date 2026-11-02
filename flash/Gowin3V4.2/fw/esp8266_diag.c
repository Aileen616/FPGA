#include "esp8266_diag.h"
#include <string.h>
/* 固定本地热点；CUR避免每次上电写模块Flash。密码是诊断默认值，可在部署前修改。 */
static const char *const init_commands[]={
    "AT\r\n", "ATE0\r\n", "AT+GMR\r\n", "AT+CWMODE_CUR=2\r\n",
    "AT+CWSAP_CUR=\"BX72_DIAG\",\"Bx72Diag8266\",6,3\r\n",
    "AT+CIPMUX=1\r\n", "AT+CIPMODE=0\r\n", "AT+CIPDINFO=0\r\n",
    "AT+CIPSERVER=1,3333\r\n", "AT+CIFSR\r\n"
};
/* 功能：求零结尾字符串长度；p不可空；返回字节数。 */
static size_t length(const char *p) { size_t n=0;while(p[n]) n++;return n; }
/* 功能：比较两个零结尾字符串；a/b不可空；相同返回true。 */
static bool equal(const char *a,const char *b)
{ while(*a && *a==*b) {a++;b++;}return *a==*b; }
/* 功能：追加无符号十进制；p须有至少10字节容量，v任意uint32；返回末尾位置。 */
static char *decimal(char *p,uint32_t v)
{ char t[10];unsigned n=0;do {t[n++]=(char)('0'+v%10U);v/=10U;}while(v);while(n) *p++=t[--n];return p; }
/* 功能：追加字符串；p容量须足够，t不可空；返回末尾位置，不加零结尾。 */
static char *append(char *p,const char *t) { while(*t) *p++=*t++;return p; }
/* 功能：严格解析十进制uint32；p/out不可空，不允许符号；成功更新位置并返回true。 */
static bool number(const char **p,uint32_t *out)
{
    uint32_t v=0;unsigned n=0;
    while(**p>='0' && **p<='9') {
        unsigned d=(unsigned)(**p-'0');if(v>(UINT32_MAX-d)/10U) return false;
        v=v*10U+d;(*p)++;n++;
    }
    *out=v;return n!=0;
}
/* 功能：撤销测试会话；s不可空；递增代号使旧缓存命令失效，无返回。 */
static void stop(esp_diag_t *s)
{
    s->io.guard(s->io.ctx,ESP_G_STOP,0);s->active=false;s->epoch++;
    if(s->epoch==0) s->epoch=1;
    for(unsigned i=0;i<5;i++) s->command_len[i]=0;
}
void esp_diag_transport_fault(esp_diag_t *s)
{ stop(s);s->blocked=true;s->ready=false;s->reply_len=0; }
/* 功能：排队一条应答；s/text不可空，id为0..4；拥塞时故障禁能，无返回。 */
static void reply(esp_diag_t *s,unsigned id,const char *text)
{
    size_t n=length(text);
    if(s->reply_len || s->send_state || n>=sizeof(s->reply)) {stop(s);s->rejected++;return;}
    memcpy(s->reply,text,n);s->reply_len=(unsigned)n;s->reply_id=(int)id;
}
/* 功能：生成实际保护状态；s不可空，id为0..4；无返回。 */
static void status(esp_diag_t *s,unsigned id)
{
    char text[160],*p=text;uint32_t state=s->io.state(s->io.ctx);
    p=append(p,"DIAG MOTOR=0 ACTIVE=");p=decimal(p,state&1U);
    p=append(p," FAULT=");p=decimal(p,(state>>1)&1U);
    p=append(p," EPOCH=");p=decimal(p,s->epoch);
    p=append(p," SEQ=");p=decimal(p,s->sequence);
    p=append(p," RX=");p=decimal(p,s->accepted);
    p=append(p," BAD=");p=decimal(p,s->rejected);p=append(p,"\r\n");*p=0;
    reply(s,id,text);
}
/* 功能：执行一条诊断行；s/text不可空，id为0..4；任何指令均不产生电机输出。 */
static void command(esp_diag_t *s,unsigned id,const char *text)
{
    uint32_t epoch=0,seq=0;const char *p;
    if(!s->ready || !s->connected[id]) return;
    if((int)id!=s->owner) {reply(s,id,"BUSY\r\n");return;}
    if(equal(text,"ping")) {reply(s,id,"PONG MOTOR=0\r\n");return;}
    if(equal(text,"status")) {status(s,id);return;}
    if(equal(text,"stop")) {stop(s);reply(s,id,"STOPPED MOTOR=0\r\n");return;}
    if(equal(text,"clear")) {
        stop(s);s->sequence=0;s->io.guard(s->io.ctx,ESP_G_CLEAR,0);status(s,id);return;
    }
    if(text[0]=='a' && text[1]=='r' && text[2]=='m' && text[3]==' ') {
        p=text+4;
        if(number(&p,&epoch) && *p==0 && epoch==s->epoch && !s->active &&
           (s->io.state(s->io.ctx)&3U)==0) {
            s->sequence=0;s->io.guard(s->io.ctx,ESP_G_ARM,0);
            s->active=(s->io.state(s->io.ctx)&3U)==1U;
            reply(s,id,s->active?"ARMED TEST_ONLY MOTOR=0\r\n":"FAULT\r\n");return;
        }
    } else if(text[0]=='h' && text[1]=='b' && text[2]==' ') {
        p=text+3;
        if(number(&p,&epoch) && *p++==' ' && number(&p,&seq) && *p==0 &&
           epoch==s->epoch && seq>s->sequence && s->active &&
           (s->io.state(s->io.ctx)&3U)==1U) {
            s->sequence=seq;s->accepted++;s->io.guard(s->io.ctx,ESP_G_KICK,seq);return;
        }
        /* 重复或旧心跳不续期，也不逐包应答，避免应答流量造成拥塞。 */
        s->rejected++;return;
    }
    s->rejected++;stop(s);reply(s,id,"REJECTED MOTOR=0\r\n");
}
/* 功能：处理AT完整响应行；s/line不可空；仅确认当前事务，异步断线优先，无返回。 */
static void at_line(esp_diag_t *s,char *line)
{
    while(*line==' ') line++;
    if(line[0]>='0' && line[0]<='4' && line[1]==',') {
        unsigned id=(unsigned)(line[0]-'0');
        if(equal(line+2,"CONNECT")) {
            s->connected[id]=true;s->command_len[id]=0;
            if(s->owner<0) {s->owner=(int)id;stop(s);}
        } else if(equal(line+2,"CLOSED")) {
            s->connected[id]=false;s->command_len[id]=0;
            if(s->owner==(int)id) {stop(s);s->owner=-1;}
            if(s->reply_id==(int)id && s->reply_len) esp_diag_transport_fault(s);
        }
        return;
    }
    if(equal(line,"ready") || equal(line,"WIFI DISCONNECT")) {esp_diag_transport_fault(s);return;}
    if(equal(line,"ERROR") || equal(line,"FAIL") || equal(line,"SEND FAIL") ||
       (line[0]=='b' && line[1]=='u' && line[2]=='s' && line[3]=='y')) {
        esp_diag_transport_fault(s);return;
    }
    if(!s->ready && s->waiting && equal(line,"OK")) {
        s->waiting=false;s->init_step++;
        if(s->init_step==sizeof(init_commands)/sizeof(init_commands[0])) s->ready=true;
    } else if(s->send_state==2 && equal(line,"SEND OK")) {
        s->send_state=0;s->reply_len=0;
    }
}
void esp_diag_init(esp_diag_t *s,const esp_diag_io_t *io,uint32_t now)
{
    memset(s,0,sizeof(*s));s->io=*io;s->owner=-1;s->reply_id=-1;s->now=now;
    s->deadline=now;s->epoch=1;s->last_rx=now;s->io.guard(s->io.ctx,ESP_G_STOP,0);
}
void esp_diag_rx(esp_diag_t *s,uint8_t byte,uint32_t now)
{
    s->now=now;s->last_rx=now;if(s->blocked) return;
    if(s->payload_left) {
        unsigned id=(unsigned)s->payload_id;s->payload_left--;
        if(byte=='\n') {
            s->command[id][s->command_len[id]]=0;
            command(s,id,s->command[id]);s->command_len[id]=0;
        } else if(byte!='\r') {
            if(byte<32 || byte>126 || s->command_len[id]>=sizeof(s->command[id])-1U) {
                esp_diag_transport_fault(s);return;
            }
            s->command[id][s->command_len[id]++]=(char)byte;
        }
        return;
    }
    if(byte=='>' && s->send_state==1) {
        if(!s->io.write(s->io.ctx,s->reply,s->reply_len)) {esp_diag_transport_fault(s);return;}
        s->send_state=2;s->deadline=now;s->at_len=0;return;
    }
    if(byte=='\r') return;
    if(byte=='\n') {
        s->at_line[s->at_len]=0;at_line(s,s->at_line);s->at_len=0;return;
    }
    if(s->at_len>=sizeof(s->at_line)-1U || byte<32 || byte>126) {
        esp_diag_transport_fault(s);return;
    }
    s->at_line[s->at_len++]=(char)byte;s->at_line[s->at_len]=0;
    if(byte==':' && s->at_len>=5 && s->at_line[0]=='+' && s->at_line[1]=='I' &&
       s->at_line[2]=='P' && s->at_line[3]=='D' && s->at_line[4]==',') {
        const char *p=s->at_line+5;uint32_t id,n;
        if(!number(&p,&id) || *p++!=',' || !number(&p,&n) || *p++!=':' || *p ||
           id>4 || n==0 || n>2048) {esp_diag_transport_fault(s);return;}
        s->payload_id=(int)id;s->payload_left=n;s->at_len=0;
    }
}
void esp_diag_poll(esp_diag_t *s,uint32_t now)
{
    s->now=now;if(s->blocked) return;
    /* 硬件已经先行撤销测试有效位；软件轮询只负责更换代号和清理残留命令。 */
    if(s->active && (s->io.state(s->io.ctx)&3U)!=1U) stop(s);
    if((s->payload_left || s->at_len) && (uint32_t)(now-s->last_rx)>=1000U) {
        esp_diag_transport_fault(s);return;
    }
    if((s->waiting || s->send_state) && (uint32_t)(now-s->deadline)>=3000U) {
        esp_diag_transport_fault(s);return;
    }
    if(!s->ready && !s->waiting) {
        const char *c=init_commands[s->init_step];
        if(!s->io.write(s->io.ctx,c,length(c))) {esp_diag_transport_fault(s);return;}
        s->waiting=true;s->deadline=now;
    } else if(s->ready && s->reply_len && !s->send_state) {
        char text[40],*p=text;p=append(p,"AT+CIPSEND=");p=decimal(p,(uint32_t)s->reply_id);
        *p++=',';p=decimal(p,s->reply_len);p=append(p,"\r\n");
        if(!s->io.write(s->io.ctx,text,(size_t)(p-text))) {esp_diag_transport_fault(s);return;}
        s->send_state=1;s->deadline=now;
    }
}
