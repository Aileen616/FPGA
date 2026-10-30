#include "bypass_return.h"
#include "bypass_direction_policy.h"
#include <string.h>
/* 功能、参数、返回值见头文件。 */
void bypass_init(bypass_fsm_t *s){memset(s,0,sizeof(*s));}
/* 功能、参数、返回值见头文件。 */
bool bypass_active(const bypass_fsm_t *s){return s->phase>B_IDLE&&s->phase<B_DONE;}
/* 功能、参数、返回值见头文件。 */
void bypass_abort(bypass_fsm_t *s,bypass_reason_t reason){s->phase=B_FAULT;s->reason=reason;}
/* 功能：进入新阶段；s/in不可空，phase合法；无返回，静止和新帧门禁重新计数。 */
static void enter(bypass_fsm_t *s,bypass_phase_t phase,const bypass_input_t *in)
{
    s->phase=phase;s->phase_ms=s->quiet_ms=in->now_ms;s->clear_samples=0;s->quiet_confirmed=false;
    s->last_front_sequence=in->front_sequence;s->last_left_sequence=in->left_sequence;
    if(phase==B_APPROACH||phase==B_SHIFT_RIGHT||phase==B_PASS||phase==B_RETURN_LEFT)++s->generation;
}
/* 功能、参数、返回值见头文件。 */
bool bypass_begin(bypass_fsm_t *s,const bypass_config_t *c,const bypass_input_t *in)
{
    if(bypass_active(s)||!c->geometry_confirmed||!c->lateral_scale_checked||
       c->body_length_mm<100||c->body_length_mm>1000||c->body_width_mm<100||c->body_width_mm>1000||
       !c->obstacle_width_mm||c->obstacle_width_mm>1000||!c->obstacle_depth_mm||c->obstacle_depth_mm>1000||
       c->shift_mm!=450||c->clearance_mm<200||c->clearance_mm>600||
       2U*c->shift_mm<(uint32_t)c->body_width_mm+c->obstacle_width_mm+2U*c->clearance_mm||
       !in->sensors_fresh||!in->imu_ready||!in->lease_valid||in->wheels_moving||
       in->front_mm<=800||
       s->generation>UINT32_MAX-4U)return false;
    s->config=*c;s->reason=B_OK;s->started_ms=in->now_ms;s->returned=false;
    s->pass_mm=s->approach_stop_mm=0;enter(s,B_APPROACH,in);return true;
}
/* 功能：连续200ms轮子静止门禁；s/in不可空；通过true，2秒未停稳锁故障。 */
static bool settled(bypass_fsm_t *s,const bypass_input_t *in)
{
    if(in->wheels_moving){s->quiet_ms=in->now_ms;s->quiet_confirmed=false;s->clear_samples=0;}
    if(!s->quiet_confirmed&&(uint32_t)(in->now_ms-s->phase_ms)>=2000U){bypass_abort(s,B_NOT_SETTLED);return false;}
    if((uint32_t)(in->now_ms-s->quiet_ms)<200U)return false;
    if(!s->quiet_confirmed){
        s->quiet_confirmed=true;s->clear_samples=0;
        s->last_front_sequence=in->front_sequence;s->last_left_sequence=in->left_sequence;
    }
    return true;
}
/* 功能、参数、返回值见头文件。 */
void bypass_step(bypass_fsm_t *s,const bypass_input_t *in)
{
    if(!bypass_active(s))return;
    if(!in->lease_valid){bypass_abort(s,B_LINK);return;}
    unsigned selected=bypass_tof_channel(s->phase);
    bool waiting=s->phase==B_WAIT_FRONT||s->phase==B_WAIT_SHIFT||s->phase==B_WAIT_PASS;
    if(selected<3&&!in->sensors_fresh&&!waiting){bypass_abort(s,B_SENSOR);return;}
    if(!in->imu_ready){bypass_abort(s,B_IMU);return;}
    if((uint32_t)(in->now_ms-s->started_ms)>=80000U){bypass_abort(s,B_TIMEOUT);return;}
    if(s->phase==B_APPROACH&&in->result_generation==s->generation&&in->result==B_SEG_OBSTACLE){s->approach_stop_mm=in->front_mm;enter(s,B_WAIT_FRONT,in);return;}
    /* 换向时先保持零输出，给新选中的探头采集机会；不刷新旧时间戳，不放宽运动期200ms门限。 */
    if(waiting&&!in->sensors_fresh){
        s->clear_samples=0;
        if(!settled(s,in))return;
        if((uint32_t)(in->now_ms-s->phase_ms)>=5000U)bypass_abort(s,B_SENSOR);
        return;
    }
    /* 只检查当前/下一运动方向，旁侧近物和无效回波不阻断本段。 */
    if(selected<3&&bypass_direction_distance(in,selected)<=(selected==2?200U:300U)){bypass_abort(s,B_BLOCKED);return;}
    if(s->phase==B_APPROACH||s->phase==B_SHIFT_RIGHT||s->phase==B_PASS||s->phase==B_RETURN_LEFT){
        if(in->result==B_SEG_RUNNING||in->result_generation!=s->generation)return;
        if(in->result==B_SEG_FAILED){bypass_abort(s,B_SEGMENT_FAULT);return;}
        if(s->phase==B_APPROACH){
            if(in->result==B_SEG_DISTANCE)enter(s,B_WAIT_END,in);
            else{s->approach_stop_mm=in->front_mm;enter(s,B_WAIT_FRONT,in);}
        }else if(in->result==B_SEG_OBSTACLE)bypass_abort(s,B_BLOCKED);
        else if(s->phase==B_SHIFT_RIGHT)enter(s,B_WAIT_SHIFT,in);
        else if(s->phase==B_PASS)enter(s,B_WAIT_PASS,in);
        else{s->returned=true;enter(s,B_WAIT_END,in);}
        return;
    }
    if(!settled(s,in))return;
    if(s->phase==B_WAIT_FRONT){
        if(in->right_mm<=800U){bypass_abort(s,B_BLOCKED);return;}
        /* 越障固定900mm，沿用前进停车时保存的距离，不再依赖此刻前向回波。 */
        s->pass_mm=900;enter(s,B_SHIFT_RIGHT,in);
    }else if(s->phase==B_WAIT_SHIFT||s->phase==B_WAIT_PASS){
        /* 必须先停稳，再接收三个不同序号的帧；移动期间的清路帧不计入。 */
        uint32_t seq=s->phase==B_WAIT_SHIFT?in->front_sequence:in->left_sequence;
        uint32_t *previous=s->phase==B_WAIT_SHIFT?&s->last_front_sequence:&s->last_left_sequence;
        uint16_t distance=s->phase==B_WAIT_SHIFT?in->front_mm:in->left_mm;
        unsigned threshold=s->phase==B_WAIT_SHIFT?800U:800U;
        if(seq!=*previous){*previous=seq;if(distance>threshold){if(s->clear_samples<3)++s->clear_samples;}else s->clear_samples=0;}
        if(s->clear_samples>=3)enter(s,s->phase==B_WAIT_SHIFT?B_PASS:B_RETURN_LEFT,in);
        else if((uint32_t)(in->now_ms-s->phase_ms)>=5000U)bypass_abort(s,B_BLOCKED);
    }else if(s->phase==B_WAIT_END)enter(s,B_DONE,in);
}
/* 功能、参数、返回值见头文件。 */
bool bypass_mix(unsigned direction,int32_t effort,int32_t ratio,int32_t out[4])
{
    static const int8_t signs[3][4]={{1,1,1,1},{1,-1,-1,1},{-1,1,1,-1}};
    if(direction>2||effort<0||effort>19660||ratio< -100||ratio>100||!out)return false;
    int32_t values[4],maximum=0;
    for(unsigned c=0;c<4;c++){
        values[c]=signs[direction][c]*effort+(c<2?1:-1)*effort*ratio/1000;
        int32_t magnitude=values[c]<0?-values[c]:values[c];if(magnitude>maximum)maximum=magnitude;
    }
    for(unsigned c=0;c<4;c++)out[c]=maximum>19660?values[c]*19660/maximum:values[c];
    return true;
}
