#include "heading_calibration.h"
#include <string.h>
#include <limits.h>
#include <stddef.h>

/* 工程准入值，非Bosch器件规格；±500deg/s下65.536counts/(deg/s)。
 * RMS<=8counts(0.122deg/s)，100帧块均值跨度<=8counts，前后半窗均值差<=4counts。
 * 64counts极差仅用于突变保护；不以放大极差阈值替代方差/漂移检查。
 */
#define HCAL_MAX_SPAN 64
#define HCAL_MAX_VARIANCE 64
#define HCAL_MAX_BLOCK_DRIFT 8
#define HCAL_MAX_HALF_DRIFT 4
#define HCAL_MAX_ACC_BLOCK_DRIFT 64

/* 功能：求受限有符号数绝对值；v不等于INT32_MIN；返回非负整数。 */
static int32_t magnitude(int32_t v){return v<0?-v:v;}
/* 功能：锁存标定失败且撤销航向；c/h不可空，reason为原因位、axes为XYZ掩码；无返回。 */
static void reject(heading_calibration_t *c,heading_hold_t *h,uint32_t reason,uint32_t axes)
{c->reason|=reason;c->axis_mask|=axes;h->fault=true;h->calibrated=false;h->calibrating=false;h->running=false;h->enabled=false;}
/* 功能/参数/返回：见头文件。 */
void heading_calibration_start(heading_calibration_t *c,heading_hold_t *h)
{
    /* 仅清统计元数据，避免软核逐字节清6KB原始缓存阻塞200Hz采样。
     * collected归零后旧缓存不可读；随后每个有效索引由新帧覆盖。 */
    memset((unsigned char *)c+offsetof(heading_calibration_t,block_sum),0,
           sizeof(*c)-offsetof(heading_calibration_t,block_sum));
    heading_calibrate(h);
    for(unsigned i=0;i<3;i++){
        c->block_min[i]=c->acc_min[i]=INT32_MAX;
        c->block_max[i]=c->acc_max[i]=INT32_MIN;
    }
}
/* 功能/参数/返回：见头文件。 */
void heading_calibration_sample(heading_calibration_t *c,heading_hold_t *h,
                               const int16_t acc[3],const int16_t gyro[3],bool valid,uint32_t stamp_us)
{
    if(!h->calibrating){heading_sample(h,acc,gyro,valid,stamp_us);return;}
    /* 复用原始有效性/时间检查及单位换算；标定期间不执行原来的单极差判据。 */
    h->calibrating=false;heading_sample(h,acc,gyro,valid,stamp_us);
    if(h->fault||!h->feedback_valid){reject(c,h,HCAL_TIME,7);return;}
    h->calibrating=true;
    if(h->dt_us>c->max_dt_us)c->max_dt_us=h->dt_us;
    if(c->collected>=HCAL_FRAMES){reject(c,h,HCAL_TIME,7);return;}
    for(unsigned i=0;i<3;i++)c->raw[c->collected][i]=gyro[i];
    c->collected++;
    int64_t norm=0;
    for(unsigned i=0;i<3;i++)norm+=(int64_t)acc[i]*acc[i];
    if(norm<24159191||norm>36089604){reject(c,h,HCAL_ACCEL,7);return;}
    for(unsigned i=0;i<3;i++){
        if(gyro[i]<h->lo[i])h->lo[i]=gyro[i];
        if(gyro[i]>h->hi[i])h->hi[i]=gyro[i];
        if(magnitude(gyro[i])>328)reject(c,h,HCAL_RATE,1U<<i);
        if(h->hi[i]-h->lo[i]>HCAL_MAX_SPAN)reject(c,h,HCAL_SPAN,1U<<i);
    }
    if(h->fault)return;
    for(unsigned i=0;i<3;i++){
        h->sums[i]+=gyro[i];c->squares[i]+=(int32_t)gyro[i]*gyro[i];
        c->block_sum[i]+=gyro[i];c->acc_sum[i]+=acc[i];
        if(h->samples<HCAL_FRAMES/2)c->half_sum[i]+=gyro[i];
    }
    h->samples++;
    if(h->samples%HCAL_BLOCK==0){
        for(unsigned i=0;i<3;i++){
            if(c->block_sum[i]<c->block_min[i])c->block_min[i]=c->block_sum[i];
            if(c->block_sum[i]>c->block_max[i])c->block_max[i]=c->block_sum[i];
            if(c->acc_sum[i]<c->acc_min[i])c->acc_min[i]=c->acc_sum[i];
            if(c->acc_sum[i]>c->acc_max[i])c->acc_max[i]=c->acc_sum[i];
            c->block_sum[i]=c->acc_sum[i]=0;
        }
    }
    if(h->samples!=HCAL_FRAMES)return;
    for(unsigned i=0;i<3;i++){
        /* 整数比较不受均值取整影响；幅度先受限，64位中间量不会溢出。 */
        int64_t numerator=c->squares[i]*HCAL_FRAMES-h->sums[i]*h->sums[i];
        int32_t drift=c->block_max[i]-c->block_min[i];
        int32_t half=magnitude(2*c->half_sum[i]-(int32_t)h->sums[i]);
        c->variance_milli[i]=(uint32_t)(numerator/1000);
        c->drift_centi[i]=(uint32_t)drift;
        c->half_centi[i]=(uint32_t)(half/5);
        if(numerator>(int64_t)HCAL_MAX_VARIANCE*HCAL_FRAMES*HCAL_FRAMES)reject(c,h,HCAL_VARIANCE,1U<<i);
        if(drift>HCAL_MAX_BLOCK_DRIFT*(int32_t)HCAL_BLOCK||half>HCAL_MAX_HALF_DRIFT*(int32_t)(HCAL_FRAMES/2))reject(c,h,HCAL_DRIFT,1U<<i);
        if(c->acc_max[i]-c->acc_min[i]>HCAL_MAX_ACC_BLOCK_DRIFT*(int32_t)HCAL_BLOCK)reject(c,h,HCAL_TILT,1U<<i);
    }
    if(h->fault)return;
    for(unsigned i=0;i<3;i++)h->bias_q16[i]=(int32_t)(h->sums[i]*65536/HCAL_FRAMES);
    h->calibrated=true;h->calibrating=false;h->yaw_q16_md=0;h->have_yaw_previous=false;
}
