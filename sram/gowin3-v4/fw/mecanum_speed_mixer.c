#include "mecanum_speed_mixer.h"
#include <math.h>
#include <float.h>
#include <stddef.h>
#include <limits.h>
/* 无全局可变状态、物理寄存器或外设句柄；遵循现有X型麦轮坐标约定。 */

/* 功能：检查double是否有限；参数：v任意double；返回：有限true，NaN/无穷false。
 * 使用标准范围比较，避免旧MinGW的isfinite宏在严格转换检查下隐式窄化为float。
 */
static bool finite_value(double v)
{
    return v<=DBL_MAX && v>=-DBL_MAX;
}
/* 功能：检查正且有限实数；参数：v任意double；返回：满足条件true，否则false。 */
static bool positive_finite(double v)
{
    return finite_value(v) && v>0.0;
}
/* 功能：机体速度转编码器目标；参数范围见头文件；返回：成功true，否则清零并返回false。 */
bool mecanum_speed_targets(const mecanum_speed_geometry_t *g,const mecanum_speed_command_t *c,
                           int32_t out[4],double *scale)
{
    const double pi=3.14159265358979323846;
    double linear[4],raw[4],rotation,factor=1.0;
    unsigned i;
    if(scale!=NULL) *scale=0.0;
    if(out==NULL) return false;
    for(i=0;i<4U;i++) out[i]=0;
    if(g==NULL || c==NULL || !positive_finite(g->half_wheelbase_m) ||
       !positive_finite(g->half_track_m) || !finite_value(c->forward_m_s) ||
       !finite_value(c->left_m_s) || !finite_value(c->ccw_rad_s)) return false;
    rotation=(g->half_wheelbase_m+g->half_track_m)*c->ccw_rad_s;
    linear[0]=c->forward_m_s-c->left_m_s-rotation;
    linear[1]=c->forward_m_s+c->left_m_s+rotation;
    linear[2]=c->forward_m_s+c->left_m_s-rotation;
    linear[3]=c->forward_m_s-c->left_m_s+rotation;
    for(i=0;i<4U;i++) {
        if(!positive_finite(g->wheel_diameter_m[i]) || g->counts_per_rev[i]==0U ||
           g->max_cps[i]==0U || g->max_cps[i]>INT32_MAX) return false;
        raw[i]=linear[i]/(pi*g->wheel_diameter_m[i])*(double)g->counts_per_rev[i];
        if(!finite_value(raw[i])) return false;
        if(fabs(raw[i])>(double)g->max_cps[i]) {
            const double available=(double)g->max_cps[i]/fabs(raw[i]);
            if(available<factor) factor=available;
        }
    }
    for(i=0;i<4U;i++) {
        const double value=raw[i]*factor;
        /* 避免浮点舍入使边界越界；共同缩放保持四轮速度比例，仅计数取整有量化误差。 */
        if(value>=(double)g->max_cps[i]) out[i]=(int32_t)g->max_cps[i];
        else if(value<= -(double)g->max_cps[i]) out[i]= -(int32_t)g->max_cps[i];
        else out[i]=(int32_t)(value+(value>=0.0?0.5:-0.5));
    }
    if(scale!=NULL) *scale=factor;
    return true;
}
