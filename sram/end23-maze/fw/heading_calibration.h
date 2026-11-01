#ifndef HEADING_CALIBRATION_H
#define HEADING_CALIBRATION_H
#include "heading_hold.h"

#define HCAL_FRAMES 1000U
#define HCAL_BLOCK 100U
/* 失败位：时间/无效帧、加速度、角速度绝对值、突变、方差、均值漂移、姿态变化。 */
enum { HCAL_TIME=1, HCAL_ACCEL=2, HCAL_RATE=4, HCAL_SPAN=8,
       HCAL_VARIANCE=16, HCAL_DRIFT=32, HCAL_TILT=64 };
/* 独立PWM试验使用；全局/static分配，禁止放在中断或小栈上。原始陀螺窗口约6KB。 */
typedef struct {
    int16_t raw[HCAL_FRAMES][3]; /* 最近一次标定的原始XYZ，counts，不滤波或剔除极值。 */
    int32_t block_sum[3],block_min[3],block_max[3],half_sum[3];
    int32_t acc_sum[3],acc_min[3],acc_max[3]; /* 每100帧加速度均值的分子，counts。 */
    int64_t squares[3]; /* 陀螺平方和，counts^2；绝对值检查后累计。 */
    uint32_t variance_milli[3],drift_centi[3],half_centi[3]; /* 方差×1000、均值差×100。 */
    uint32_t reason,axis_mask,collected,max_dt_us; /* 原因位、失败轴、存储帧数、最大帧间隔。 */
} heading_calibration_t;

/* 功能：清空诊断窗口并使旧零偏失效；c/h不可空；无返回，默认关闭航向。 */
void heading_calibration_start(heading_calibration_t *c,heading_hold_t *h);
/* 功能：整窗统计标定，非标定阶段复用原航向积分；c/h/acc/gyro不可空，时间/valid同heading_sample。
 * 无返回；1000帧全部满足判据才更新零偏，失败锁存且不自动重试。
 */
void heading_calibration_sample(heading_calibration_t *c,heading_hold_t *h,
                               const int16_t acc[3],const int16_t gyro[3],bool valid,uint32_t stamp_us);
#endif
