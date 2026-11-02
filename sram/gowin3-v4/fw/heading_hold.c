#include "heading_hold.h"
#include <string.h>
/* 功能：求绝对值；v不能为INT32_MIN；返回非负幅值。 */
static int32_t absolute(int32_t v){return v<0?-v:v;}
/* 功能/参数/返回：见头文件。 */
void heading_init(heading_hold_t *h)
{memset(h,0,sizeof(*h));h->kp=10;h->kd=1;h->limit=150;}
/* 功能/参数/返回：见头文件；保留已确认轴向，禁用旧运行状态。 */
void heading_calibrate(heading_hold_t *h)
{
    h->calibrating=true;h->calibrated=false;h->enabled=false;h->running=false;h->fault=false;
    h->samples=0;h->yaw_q16_md=0;
    h->have_yaw_previous=false;
    for(unsigned i=0;i<3;i++){h->sums[i]=0;h->lo[i]=32767;h->hi[i]=-32768;}
}
/* 功能/参数/返回：见头文件。静止并不完全可观测，标定仍要求用户保持车体不动。 */
void heading_sample(heading_hold_t *h,const int16_t acc[3],const int16_t gyro[3],bool valid,uint32_t stamp_us)
{
    uint32_t dt=h->have_stamp?stamp_us-h->stamp_us:0;
    if(!valid||(h->have_stamp&&(dt==0||dt>=20000))){
        h->fault=true;h->calibrating=false;h->calibrated=false;h->running=false;
        h->feedback_valid=false;h->have_stamp=false;h->have_yaw_previous=false;h->dt_us=0;return;
    }
    h->stamp_us=stamp_us;h->dt_us=dt;h->have_stamp=true;h->feedback_valid=true;
    for(unsigned i=0;i<3;i++){
        int64_t value=(int64_t)gyro[i]*65536-(h->calibrated?h->bias_q16[i]:0);
        h->gyro_mdps[i]=(int32_t)(value*500000/INT64_C(2147483648));
    }
    if(h->calibrating){
        int64_t norm=0;for(unsigned i=0;i<3;i++)norm+=(int64_t)acc[i]*acc[i];
        /* ±6g量程下1g约5461counts；模长允许0.9g..1.1g，角速度允许±5deg/s。 */
        bool quiet=norm>=24159191&&norm<=36089604;
        for(unsigned i=0;i<3;i++){
            if(absolute(gyro[i])>328)quiet=false;
            if(gyro[i]<h->lo[i])h->lo[i]=gyro[i];
            if(gyro[i]>h->hi[i])h->hi[i]=gyro[i];
            if(h->hi[i]-h->lo[i]>40)quiet=false;
            h->sums[i]+=gyro[i];
        }
        if(!quiet){h->fault=true;h->calibrating=false;return;}
        if(++h->samples==1000){
            for(unsigned i=0;i<3;i++){
                h->bias_q16[i]=(int32_t)(h->sums[i]*65536/1000);
                int64_t value=(int64_t)gyro[i]*65536-h->bias_q16[i];
                h->gyro_mdps[i]=(int32_t)(value*500000/INT64_C(2147483648));
            }
            h->calibrated=true;h->calibrating=false;h->yaw_q16_md=0;
        }
        return;
    }
    if(!h->calibrated||!h->axis_confirmed||h->fault){h->have_yaw_previous=false;return;}
    int64_t corrected=((int64_t)gyro[h->axis]*65536-h->bias_q16[h->axis])*h->sign;
    h->rate_mdps=(int32_t)(corrected*500000/INT64_C(2147483648));
    /* ±500deg/s量程下，Q16毫度增量=(前后原始Q16计数之和)*dt_us/131072。
       用实测间隔替代固定5ms；首帧只建立积分起点，长间隔和积压由上层拒绝。 */
    if(h->have_yaw_previous)h->yaw_q16_md+=(h->previous_corrected+corrected)*dt/131072;
    h->previous_corrected=corrected;h->have_yaw_previous=true;
    const int64_t turn=INT64_C(360000)*65536,half=INT64_C(180000)*65536;
    if(h->yaw_q16_md>=half)h->yaw_q16_md-=turn;
    if(h->yaw_q16_md< -half)h->yaw_q16_md+=turn;
}
/* 功能/参数/返回：见头文件。 */
bool heading_begin(heading_hold_t *h)
{
    if(!h->enabled||!h->calibrated||!h->axis_confirmed||h->fault)return false;
    h->reference_md=(int32_t)(h->yaw_q16_md/65536);h->running=true;return true;
}
/* 功能/参数/返回：见头文件；保持前进或后退符号，避免低速修正导致某轮反转。 */
bool heading_targets(heading_hold_t *h,int32_t base,int32_t out[4])
{
    if(!h->running||h->fault||!h->calibrated||!h->axis_confirmed||absolute(base)<200||absolute(base)>1000)return false;
    int32_t error=h->reference_md-(int32_t)(h->yaw_q16_md/65536);
    if(error>180000)error-=360000;
    if(error< -180000)error+=360000;
    /* 偏离超过20度或角速度超过90deg/s时撤销，不持续饱和追赶。 */
    if(absolute(error)>20000||absolute(h->rate_mdps)>90000){h->fault=true;h->running=false;return false;}
    int32_t u=(error*h->kp-h->rate_mdps*h->kd)/1000;
    if(u>h->limit)u=h->limit;
    if(u< -h->limit)u=-h->limit;
    h->correction=u;out[0]=out[1]=base+u;out[2]=out[3]=base-u;return true;
}
