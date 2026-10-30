#include "autonomy_policy.h"
#include <string.h>

/* 功能：比较回绕序号；a/b为uint32序号，跨度必须小于2^31；返回a是否更新。 */
static bool newer(uint32_t a, uint32_t b) { return a != b && (uint32_t)(a-b)<UINT32_C(0x80000000); }
/* 功能：判断新鲜度；时间单位ms，limit不可为0；在严格上限内返回true。 */
static bool fresh(uint32_t now, uint32_t stamp, uint32_t limit) { return (uint32_t)(now-stamp)<limit; }
/* 功能：计算较大值；输入uint16；返回较大值。 */
static uint16_t maximum(uint16_t a, uint16_t b) { return a>b?a:b; }

void ap_defaults(ap_config_t *c)
{
    memset(c,0,sizeof(*c));
    c->body_length_mm=280; c->body_width_mm=250;
    c->start_mm=200; c->slow_mm=200; c->emergency_mm=80; c->hysteresis_mm=30;
    c->max_speed_mm_s=100; c->latency_ms=500; c->error_margin_mm=20;
    c->encoder_window_ms=200;
    c->range_age_ms=200; c->imu_age_ms=20; c->control_gap_ms=250;
    c->recover_ms=200; c->wait_ms=5000; c->task_ms=80000; c->segment_mm=100;
}

bool ap_init(ap_policy_t *p, const ap_config_t *c)
{
    memset(p,0,sizeof(*p)); p->config=*c;
    bool valid=c->body_length_mm>=100&&c->body_length_mm<=1000&&
        c->body_width_mm>=100&&c->body_width_mm<=1000&&
        c->emergency_mm>=20&&c->emergency_mm<=1000&&c->hysteresis_mm>=1&&c->hysteresis_mm<=500&&
        c->start_mm>c->emergency_mm+c->hysteresis_mm&&c->start_mm<=5000&&
        c->slow_mm>c->emergency_mm+c->hysteresis_mm&&c->slow_mm<=5000&&
        c->max_speed_mm_s>0&&c->max_speed_mm_s<=2000&&c->brake_decel_mm_s2<=10000&&
        (uint32_t)c->latency_ms>=(uint32_t)c->range_age_ms+c->control_gap_ms&&
        c->latency_ms<=2000&&c->error_margin_mm<=1000&&
        c->range_age_ms>0&&c->range_age_ms<=1000&&c->imu_age_ms>0&&c->imu_age_ms<=100&&
        c->control_gap_ms>0&&c->control_gap_ms<=1000&&c->recover_ms>=100&&c->recover_ms<=5000&&
        c->wait_ms>c->recover_ms&&c->wait_ms<=30000&&c->task_ms>c->wait_ms&&c->task_ms<=600000&&
        c->segment_mm>0&&c->segment_mm<=1000&&(c->offset_known_mask&~AP_ALL)==0&&
        c->encoder_glitch_budget<=10&&c->encoder_window_ms>=10&&c->encoder_window_ms<=1000;
    for(unsigned i=0;i<4;i++)valid=valid&&c->offset_mm[i]<=1000;
    p->config_valid=valid; p->reason=valid?AP_OK:AP_CONFIG;
    return valid;
}

uint16_t ap_stop_distance(const ap_config_t *c, uint16_t speed)
{
    if(!c->brake_model_verified||!c->brake_decel_mm_s2||speed>2000)return UINT16_MAX;
    /* 向上取整：延迟路程＋匀减速制动路程＋误差。模型边界必须独立实测。 */
    uint64_t delay=((uint64_t)speed*c->latency_ms+999U)/1000U;
    uint64_t divisor=2U*(uint64_t)c->brake_decel_mm_s2;
    uint64_t brake=((uint64_t)speed*speed+divisor-1U)/divisor;
    uint64_t total=delay+brake+c->error_margin_mm;
    if(total>UINT16_MAX)return UINT16_MAX;
    return maximum(c->emergency_mm,(uint16_t)total);
}

/* 功能：接收真实新有效帧；p/in不可空；无返回，重复序号和坏帧不刷新时间戳。 */
static void observe(ap_policy_t *p, const ap_input_t *in)
{
    for(unsigned i=0;i<4;i++){
        const ap_range_t *r=&in->range[i];
        if(r->valid&&r->raw_mm&&(!p->accepted[i].valid||newer(r->sequence,p->accepted[i].sequence))&&
           fresh(in->now_ms,r->timestamp_ms,p->config.range_age_ms))p->accepted[i]=*r;
    }
    if(in->imu_valid&&(!p->imu_received||newer(in->imu_sequence,p->imu_sequence))&&
       fresh(in->now_ms,in->imu_timestamp_ms,p->config.imu_age_ms)){
        p->imu_received=true; p->imu_sequence=in->imu_sequence; p->imu_timestamp_ms=in->imu_timestamp_ms;
    }
}

/* 功能：仅发布真实新帧；参数/返回见头文件，不触发运动或任务续期。 */
void ap_refresh(ap_policy_t *p,const ap_input_t *in){observe(p,in);}

/* 功能：取得可信净距；p不可空、i为0..3；未标定或过期时返回false。 */
static bool clearance(const ap_policy_t *p, unsigned i, uint32_t now, uint16_t *mm)
{
    const ap_range_t *r=&p->accepted[i];
    if(!(p->config.offset_known_mask&(1U<<i))||!r->valid||!fresh(now,r->timestamp_ms,p->config.range_age_ms))return false;
    *mm=r->raw_mm>p->config.offset_mm[i]?(uint16_t)(r->raw_mm-p->config.offset_mm[i]):0;
    return true;
}

/* 功能：锁存安全故障或取消；p不可空、reason为有效原因；无返回，后续step不能自行恢复。 */
static void fault(ap_policy_t *p, ap_reason_t reason)
{ p->state=AP_FAULT; p->reason=reason; p->speed_limit_mm_s=0; }

/* 功能：检查方向与旋转包络；in不可空；有效返回true。 */
static bool direction_valid(const ap_input_t *in)
{
    uint8_t m=in->direction_mask;
    if(in->rotation)return m==AP_ALL&&in->rotation_envelope_verified;
    return m&&!(m&~AP_ALL)&&(m&(AP_FRONT|AP_REAR))!=(AP_FRONT|AP_REAR)&&
        (m&(AP_RIGHT|AP_LEFT))!=(AP_RIGHT|AP_LEFT);
}

/* 功能：检查当前安全健康；p/in不可空，nearest返回运动方向最近净距；返回拒绝原因。 */
static ap_reason_t health(ap_policy_t *p, const ap_input_t *in, uint16_t *nearest)
{
    const ap_config_t *c=&p->config;
    if(in->estop)return AP_ESTOP;
    if(in->cancel)return AP_CANCEL;
    if(!p->config_valid||!c->brake_model_verified||!c->brake_decel_mm_s2)return AP_CONFIG;
    if(!direction_valid(in))return AP_DIRECTION;
    if(!in->driver_ok)return AP_DRIVE;
    if(!in->encoder_reliable)return AP_ENCODER;
    if(!fresh(in->now_ms,p->encoder_window_start_ms,c->encoder_window_ms)){
        p->encoder_window_start_ms=in->now_ms;p->encoder_glitches=0;
    }
    if(in->encoder_event){
        if(!in->encoder_event_classified_glitch||!c->encoder_glitch_policy_verified||
           p->encoder_glitches>=c->encoder_glitch_budget)return AP_ENCODER;
        p->encoder_glitches++;
    }
    if(in->imu_hard_fault||!in->calibrated||!in->axis_confirmed||!p->imu_received||
       !fresh(in->now_ms,p->imu_timestamp_ms,c->imu_age_ms))return AP_IMU;
    if(in->speed_mm_s>c->max_speed_mm_s)return AP_OVERSPEED;
    p->stop_mm=ap_stop_distance(c,in->speed_mm_s);
    *nearest=UINT16_MAX;
    for(unsigned i=0;i<4;i++){
        uint16_t d; bool ok=clearance(p,i,in->now_ms,&d);
        if(in->direction_mask&(1U<<i)){
            if(!ok)return AP_RANGE;
            if(d<=p->stop_mm)return AP_NEAR;
            if(d<*nearest)*nearest=d;
        }else if(ok&&d<=c->emergency_mm)return AP_NEAR;
    }
    return AP_OK;
}

/* 功能：进入普通等待；p/in不可空；保留任务总时限、清除恢复帧计数，无返回。 */
static void wait_for(ap_policy_t *p, const ap_input_t *in, ap_reason_t reason)
{
    if(p->state!=AP_WAIT)p->wait_since_ms=in->now_ms;
    p->state=AP_WAIT; p->reason=reason; p->speed_limit_mm_s=0;
    p->clear_frames=0; p->clear_since_ms=in->now_ms;
    for(unsigned i=0;i<4;i++)p->resume_sequence[i]=p->accepted[i].sequence;
}

bool ap_begin(ap_policy_t *p, uint8_t task, uint32_t seq, const ap_input_t *in)
{
    if(p->state==AP_RUN||p->state==AP_WAIT)return false; /* 忙时拒绝不能清除在途任务。 */
    p->speed_limit_mm_s=0;
    if(task<3||task>6||!seq||seq<=p->command_sequence){p->reason=AP_COMMAND;return false;}
    observe(p,in); uint16_t nearest;
    ap_reason_t r=health(p,in,&nearest);
    uint16_t departure=ap_stop_distance(&p->config,p->config.max_speed_mm_s);
    if(r!=AP_OK){p->reason=r;return false;}
    if(in->encoder_event||in->speed_mm_s||in->complete||in->planner_wait||nearest<=p->config.start_mm||nearest<=departure||
       (in->target_required&&(!in->target_valid||!fresh(in->now_ms,in->target_timestamp_ms,300)))){
        p->reason=AP_BLOCKED;return false;
    }
    p->state=AP_RUN; p->reason=AP_OK; p->task=task; p->command_sequence=seq;
    p->started_ms=p->last_step_ms=in->now_ms;
    p->direction_mask=in->direction_mask; p->rotation=in->rotation;
    /* 本轮仅完成准入；下个有效控制步才授权速度，便于上层原子提交。 */
    return true;
}

void ap_step(ap_policy_t *p, const ap_input_t *in)
{
    if(p->state!=AP_RUN&&p->state!=AP_WAIT){p->speed_limit_mm_s=0;return;}
    observe(p,in); uint16_t nearest;
    ap_reason_t r=health(p,in,&nearest);
    if(r!=AP_OK){fault(p,r);return;}
    if(!fresh(in->now_ms,p->last_step_ms,p->config.control_gap_ms)){fault(p,AP_CONTROL);return;}
    p->last_step_ms=in->now_ms;
    if(!fresh(in->now_ms,p->started_ms,p->config.task_ms)){fault(p,AP_TIMEOUT);return;}
    if(in->complete){p->state=AP_DONE;p->reason=AP_OK;p->speed_limit_mm_s=0;return;}
    if(p->direction_mask!=in->direction_mask||p->rotation!=in->rotation){
        if(in->speed_mm_s||p->speed_limit_mm_s){fault(p,AP_DIRECTION);return;}
        p->direction_mask=in->direction_mask;p->rotation=in->rotation;
        wait_for(p,in,AP_BLOCKED);return;
    }
    if(p->state==AP_WAIT&&!fresh(in->now_ms,p->wait_since_ms,p->config.wait_ms)){fault(p,AP_TIMEOUT);return;}
    bool target_lost=in->target_required&&(!in->target_valid||!fresh(in->now_ms,in->target_timestamp_ms,300));
    if(in->planner_wait||target_lost||nearest<=(uint32_t)p->stop_mm+p->config.hysteresis_mm){
        wait_for(p,in,target_lost?AP_TARGET:AP_BLOCKED);return;
    }
    if(p->state==AP_WAIT){
        uint16_t required=maximum(p->config.start_mm,ap_stop_distance(&p->config,p->config.max_speed_mm_s));
        if(in->speed_mm_s||nearest<=(uint32_t)required+p->config.hysteresis_mm){wait_for(p,in,p->reason);return;}
        bool next=true;
        for(unsigned i=0;i<4;i++)if(in->direction_mask&(1U<<i))next=next&&newer(p->accepted[i].sequence,p->resume_sequence[i]);
        if(next){
            if(!p->clear_frames)p->clear_since_ms=in->now_ms;
            if(p->clear_frames<3)p->clear_frames++;
            for(unsigned i=0;i<4;i++)p->resume_sequence[i]=p->accepted[i].sequence;
        }
        if(p->clear_frames<3||fresh(in->now_ms,p->clear_since_ms,p->config.recover_ms))return;
        p->state=AP_RUN;p->reason=AP_OK;
    }
    /* 搜索可接受速度，始终以该速度的停车距离校验，不只看当前静止时的80mm。 */
    uint16_t low=0, high=p->config.max_speed_mm_s;
    while(low<high){
        uint16_t middle=(uint16_t)((uint32_t)low+((uint32_t)high-low+1U)/2U);
        if((uint32_t)ap_stop_distance(&p->config,middle)+p->config.hysteresis_mm<nearest)low=middle;
        else high=(uint16_t)(middle-1U);
    }
    uint16_t limit=low;
    uint16_t slow=maximum(p->config.slow_mm,(uint16_t)((uint32_t)p->stop_mm+p->config.hysteresis_mm+1U));
    if(nearest<slow){
        uint16_t ramp=(uint16_t)((uint32_t)(nearest-p->stop_mm)*p->config.max_speed_mm_s/(uint32_t)(slow-p->stop_mm));
        if(ramp<limit)limit=ramp;
    }
    p->speed_limit_mm_s=limit;
}

uint16_t ap_segment_limit(const ap_policy_t *p, uint16_t desired, uint16_t distance)
{
    if(p->state!=AP_RUN||!p->speed_limit_mm_s)return 0;
    uint32_t reserve=(uint32_t)ap_stop_distance(&p->config,p->speed_limit_mm_s)+p->config.hysteresis_mm;
    if(distance<=reserve)return 0;
    uint32_t available=distance-reserve;
    uint16_t result=desired<p->config.segment_mm?desired:p->config.segment_mm;
    return available<result?(uint16_t)available:result;
}
