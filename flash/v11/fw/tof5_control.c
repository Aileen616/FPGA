#include "tof5_control.h"
#include <string.h>
bool tof5_active(const tof5_control_t *s)
{return s->state==T5_RUN||s->state==T5_DECEL;}
void tof5_init(tof5_control_t *s)
{memset(s,0,sizeof(*s));s->target_counts=TOF5_DISTANCE_COUNTS;s->raw_invert=12;}
void tof5_stop(tof5_control_t *s,tof5_reason_t reason)
{s->state=T5_STOP;s->reason=reason;s->effort=0;}
bool tof5_offset(tof5_control_t *s,uint32_t mm)
{
    if(tof5_active(s)||mm>1000)return false;
    s->front_offset_mm=(uint16_t)mm;s->offset_known=true;return true;
}
/* 功能：检查测距质量和200ms测试新鲜度；in不可空；全部满足返回true。 */
static bool fresh(const tof5_input_t *in)
{return in->valid&&in->age_us<200000U&&in->distance_mm>0;}
bool tof5_begin(tof5_control_t *s,const tof5_input_t *in,uint32_t seq)
{
    if(tof5_active(s)||!s->offset_known||!s->target_counts||s->target_counts>31614U||(s->raw_invert!=12&&s->raw_invert!=10&&s->raw_invert!=5)||!seq||seq<=s->last_sequence||
       !fresh(in)||!in->drive_ok||in->distance_mm<=s->front_offset_mm+TOF5_SLOW_MM)return false;
    for(unsigned i=0;i<4;i++)s->baseline[i]=(uint32_t)in->position[i];
    s->started_ms=s->renewed_ms=in->now_ms;s->last_sequence=seq;
    s->state=T5_RUN;s->reason=T5_NONE;s->effort=0;s->progress=0;return true;
}
bool tof5_keep(tof5_control_t *s,uint32_t seq,uint32_t now_ms)
{
    if(!tof5_active(s)||!seq||seq<=s->last_sequence)return false;
    if((uint32_t)(now_ms-s->renewed_ms)>=250U){tof5_stop(s,T5_LINK);return false;}
    s->renewed_ms=now_ms;s->last_sequence=seq;return true;
}
void tof5_step(tof5_control_t *s,const tof5_input_t *in)
{
    if(!tof5_active(s))return;
    /* 故障不等待减速斜坡；测距丢失绝不能当作前方空旷。 */
    if(!fresh(in)){tof5_stop(s,T5_SENSOR);return;}
    if(!in->drive_ok){tof5_stop(s,T5_DRIVE);return;}
    if((uint32_t)(in->now_ms-s->renewed_ms)>=250U){tof5_stop(s,T5_LINK);return;}
    if((uint32_t)(in->now_ms-s->started_ms)>=150000U){tof5_stop(s,T5_TIMEOUT);return;}
    int32_t minimum=INT32_MAX,maximum=INT32_MIN;int64_t total=0;
    for(unsigned c=0;c<4;c++){
        uint32_t raw=(uint32_t)in->position[c]-s->baseline[c];
        /* 通道3/4编码器反向，必须在无符号域取反，避免INT32_MIN溢出。 */
        if(s->raw_invert&(1U<<c))raw=0U-raw;
        int32_t delta=(int32_t)raw;total+=delta;
        if(delta<minimum)minimum=delta;
        if(delta>maximum)maximum=delta;
    }
    s->progress=(int32_t)(total/4);
    if(minimum < -100 || maximum>40000 || (int64_t)maximum-minimum>790){tof5_stop(s,T5_ENCODER);return;}
    if(s->progress>=(int32_t)s->target_counts){tof5_stop(s,T5_DISTANCE);return;}
    uint32_t clearance=in->distance_mm>s->front_offset_mm?(uint32_t)in->distance_mm-s->front_offset_mm:0U;
    if(clearance<=TOF5_STOP_MM){tof5_stop(s,T5_OBSTACLE);return;}
    if(s->state==T5_RUN&&clearance<=TOF5_SLOW_MM){s->state=T5_DECEL;s->decel_ms=in->now_ms;}
    int32_t limit=TOF5_BASE_EFFORT;
    if(s->state==T5_DECEL){
        uint32_t elapsed=in->now_ms-s->decel_ms;
        /* 首次检测障碍后在1秒内降为0；障碍移开也不恢复速度。 */
        if(elapsed>=1000U){tof5_stop(s,T5_OBSTACLE);return;}
        limit=(int32_t)((1000U-elapsed)*TOF5_BASE_EFFORT/1000U);
        int32_t range_limit=(int32_t)((clearance-TOF5_STOP_MM)*TOF5_BASE_EFFORT/(TOF5_SLOW_MM-TOF5_STOP_MM));
        if(range_limit<limit)limit=range_limit;
        if(limit>s->effort)limit=s->effort;
    }else{
        /* 起步1秒斜坡，末段约100mm逐步减速；最低12.5%占空比，针对右移5%实测停滞候选修正，仍待硬件验证；避免永远到不了目标。 */
        uint32_t elapsed=in->now_ms-s->started_ms;
        if(elapsed<1000U)limit=(int32_t)(elapsed*TOF5_BASE_EFFORT/1000U);
        int32_t remain=(int32_t)s->target_counts-s->progress;
        int32_t finish=remain*TOF5_BASE_EFFORT/527;
        if(finish<4096)finish=4096;
        if(finish<limit)limit=finish;
    }
    s->effort=limit;
}
