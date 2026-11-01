#include "heading_straight.h"
/* 功能、参数和返回值见头文件；先完整核对目标再改变状态。 */
bool straight_begin(heading_straight_t *s,heading_hold_t *h,const int32_t base[4])
{
    for(unsigned c=0;c<4;c++)if(base[c]<50||base[c]>1000)return false;
    if(!heading_begin(h))return false;
    for(unsigned c=0;c<4;c++)s->base[c]=base[c];
    s->ratio=0;h->correction=0;return true;
}
/* 功能、参数和返回值见头文件；续租命令不能加速纠偏斜坡，只有本地控制节拍改变ratio。 */
bool straight_targets(heading_straight_t *s,heading_hold_t *h,bool tick,int32_t out[4])
{
    if(!h->enabled||!h->running||h->fault||!h->calibrated||!h->axis_confirmed)return false;
    int32_t error=h->reference_md-(int32_t)(h->yaw_q16_md/65536);
    if(error>180000)error-=360000;
    if(error< -180000)error+=360000;
    if(error>20000||error< -20000||h->rate_mdps>90000||h->rate_mdps< -90000){
        h->fault=true;h->running=false;return false;
    }
    int32_t demand=(error*4-h->rate_mdps*2)/1000;
    if(demand>100)demand=100;
    if(demand< -100)demand=-100;
    if(tick){
        int32_t step=demand-s->ratio;
        if(step>2)step=2;
        if(step< -2)step=-2;
        s->ratio+=step;
    }
    h->correction=s->ratio; /* 测试工程的YAW.U单位为千分比，协议ITEST明确区分。 */
    for(unsigned c=0;c<4;c++)out[c]=s->base[c]*(1000+(c<2?s->ratio:-s->ratio))/1000;
    return true;
}
