#include "endpoint_black.h"
/* 新增识别服务：图像缓冲和识别器仅由本服务拥有；只发布视觉结果，不修改原电机控制通路。 */
static bool endpoint_run=true,endpoint_seen;
static uint8_t endpoint_phase,endpoint_error;
static uint32_t endpoint_sequence,endpoint_stamp,endpoint_start,endpoint_next,endpoint_index;
/* 正在计算的帧与已发布结果分开保存，计算期间不能给旧检测框换上新帧序号或时间。 */
static uint32_t endpoint_pending_sequence,endpoint_pending_stamp;
static uint32_t endpoint_elapsed,endpoint_updates,endpoint_step_max_us;
/* 计算耗时为微秒；错误快照保留本帧耗时和步数，便于区分算法与主循环调度开销。 */
static uint32_t endpoint_compute_us,endpoint_error_ms,endpoint_error_steps,endpoint_error_cpu_us;
static uint32_t endpoint_read_us; /* 本帧MMIO读取及阈值缓存生成用时，微秒，不含片段之间的调度。 */
static ep_result_t endpoint_last;
/* 功能：判断最后发布结果是否仍属本次观察窗口；now为毫秒；返回有效且年龄小于10秒，不表示可用于运动。 */
static bool endpoint_fresh(uint32_t now)
{return endpoint_seen&&!endpoint_error&&(uint32_t)(now-endpoint_stamp)<10000U;}
/* 功能：终止当前处理并报告错误；error为非零诊断码、now为毫秒；无返回，不伪造无目标有效帧。 */
static void endpoint_abort(uint8_t error,uint32_t now)
{
    endpoint_error_ms=endpoint_phase>=2?now-endpoint_pending_stamp:now-endpoint_start;
    endpoint_error_steps=endpoint_phase==3?ep_result()->steps:0;endpoint_error_cpu_us=endpoint_compute_us;
    ep_reset();endpoint_error=error;endpoint_phase=0;endpoint_next=now+250U;
}
/* 功能：非阻塞推进捕获、16像素读取和有界计算；无参数/返回；每轮计算时间目标2ms，最后一个原子步骤可能稍超出。 */
static void endpoint_service(void)
{
    uint32_t now=rd(JDY_BASE,4),flags=rd(VISION_BASE,0x104);
    if(!endpoint_phase){
        if(!endpoint_run||(int32_t)(now-endpoint_next)<0)return;
        if(flags&2U)return; /* 未完成的手工采集由硬件完成后再申请，不能改写其握手。 */
        wr(VISION_BASE,0x100,1);endpoint_start=now;endpoint_phase=1;return;
    }
    if(endpoint_phase==1){
        if(!(flags&1U)){if((uint32_t)(now-endpoint_start)>1000U)endpoint_abort(10,now);return;}
        uint32_t seq=rd(VISION_BASE,0x110);
        if(endpoint_seen&&seq==endpoint_sequence){endpoint_abort(11,now);return;}
        endpoint_pending_sequence=seq;endpoint_pending_stamp=now;endpoint_index=0;endpoint_compute_us=endpoint_read_us=0;ep_reset();endpoint_phase=2;return;
    }
    if(!(flags&1U)||rd(VISION_BASE,0x110)!=endpoint_pending_sequence){endpoint_abort(11,now);return;}
    if((uint32_t)(now-endpoint_pending_stamp)>10000U){endpoint_abort(12,now);return;}
    if(endpoint_phase==2){
        uint32_t read_begin=rd(TOF_BASE,8);
        for(unsigned n=0;n<16&&endpoint_index<EP_PIXELS;n++,endpoint_index++){
            wr(VISION_BASE,0x108,endpoint_index);ep_pixel(endpoint_index,(uint16_t)rd(VISION_BASE,0x10c));
        }
        endpoint_read_us+=rd(TOF_BASE,8)-read_begin;
        if(endpoint_index==EP_PIXELS){(void)ep_begin();endpoint_phase=3;}return;
    }
    uint32_t slice=rd(TOF_BASE,8),imu_slice=slice;
    for(unsigned n=0;n<64;n++){
        uint32_t before=rd(TOF_BASE,8);bool done=ep_step();uint32_t elapsed=rd(TOF_BASE,8)-before;
        endpoint_compute_us+=elapsed;
        if(elapsed>endpoint_step_max_us)endpoint_step_max_us=elapsed;
        if(done){
            endpoint_last=*ep_result();endpoint_error=endpoint_last.error;endpoint_seen=true;
            endpoint_sequence=endpoint_pending_sequence;endpoint_stamp=endpoint_pending_stamp;
            endpoint_elapsed=rd(JDY_BASE,4)-endpoint_stamp;++endpoint_updates;
            endpoint_phase=0;endpoint_next=rd(JDY_BASE,4)+100U;return;
        }
        uint32_t service_now=rd(TOF_BASE,8);
        /* IMU约5ms一帧且SPI读取有实际耗时；每0.5ms主动让出，避免读到跨采样边沿的数据。 */
        if((uint32_t)(service_now-imu_slice)>=500U){endpoint_imu_service();imu_slice=rd(TOF_BASE,8);}
        if((uint32_t)(rd(TOF_BASE,8)-slice)>=2000U)return;
    }
}
/* 功能：处理本地识别查询/启停；s为非空NUL字符串；命中true，查询不刷新帧时刻或启动运动。 */
static bool endpoint_command(const char *s)
{
    if(strncmp(s,"endpoint",8))return false;
    char b[256],*p;uint32_t n,now=rd(JDY_BASE,4);
    if(!strcmp(s,"endpoint work")){
        p=field(text(b,"ENDWORK V=1")," PHASE=",endpoint_phase);p=field(p," READ=",endpoint_index);
        p=field(p," STEPS=",endpoint_phase==3?ep_result()->steps:0);
        p=field(p," MS=",endpoint_phase>=2?now-endpoint_pending_stamp:0);p=field(p," CPUUS=",endpoint_compute_us);
        p=field(p," READUS=",endpoint_read_us);
        p=field(p," LASTMS=",endpoint_error_ms);p=field(p," LASTSTEP=",endpoint_error_steps);p=field(p," LASTCPUUS=",endpoint_error_cpu_us);
        p=text(p,"\r\n");*p=0;reply(b);return true;
    }
    if(!strcmp(s,"endpoint status")||!strcmp(s,"endpoint")){
        bool fresh=endpoint_fresh(now);p=text(b,"END1 V=11 ALG=3 STATIC=0");
        p=field(p," RUN=",endpoint_run);p=field(p," BUSY=",endpoint_phase);p=field(p," SEQ=",endpoint_sequence);
        p=field(p," VALID=",fresh);p=field(p," COUNT=",fresh?endpoint_last.count:0);
        p=field(p," CAND=",fresh&&endpoint_last.count==1);p=field(p," ERROR=",endpoint_error);
        p=field(p," AGE=",endpoint_seen?now-endpoint_stamp:UINT32_MAX);p=field(p," WORKMS=",endpoint_elapsed);
        p=field(p," UPDATES=",endpoint_updates);p=field(p," STEPUS=",endpoint_step_max_us);
        p=text(p,"\r\n");*p=0;reply(b);return true;
    }
    if(!strncmp(s,"endpoint box ",13)&&number(s+13,&n)&&n<endpoint_last.count&&endpoint_fresh(now)){
        const ep_box_t *a=&endpoint_last.box[n];p=field(text(b,"ENDBOX I="),"",n);
        p=field(p," SEQ=",endpoint_sequence);p=field(p," X=",a->x);p=field(p," Y=",a->y);
        p=field(p," W=",a->w);p=field(p," H=",a->h);p=field(p," AREA=",a->area);
        p=field(p," TH=",a->threshold);p=field(p," K=",a->kernel);p=field(p," SUPPORT=",a->support);
        p=text(p,"\r\n");*p=0;reply(b);return true;
    }
    if(!strncmp(s,"endpoint run ",13)&&number(s+13,&n)&&n<=1){endpoint_run=n!=0;reply("END1 RUN_OK=1\r\n");return true;}
    reply("END1 REJECT\r\n");return true;
}
/* 功能：判断图像是否由自动识别占用；无参数；占用时true，主机须先run 0并等BUSY0。 */
static bool endpoint_image_owned(void){return endpoint_run||endpoint_phase!=0;}
