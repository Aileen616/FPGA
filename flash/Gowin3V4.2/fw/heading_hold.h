#ifndef HEADING_HOLD_H
#define HEADING_HOLD_H
#include <stdint.h>
#include <stdbool.h>
/* 按实际有效读出间隔处理陀螺样本；零偏Q16原始计数，航向Q16毫度，不是绝对方位。 */
typedef struct {
    int64_t sums[3],yaw_q16_md;
    int32_t bias_q16[3],lo[3],hi[3],rate_mdps,reference_md;
    uint32_t samples;
    uint32_t stamp_us,dt_us; /* 最近有效读出时间及间隔，单位us；无首帧间隔时dt=0。 */
    int32_t gyro_mdps[3]; /* 芯片XYZ轴角速度，毫度/秒；CAL=0时未扣除零偏。 */
    int64_t previous_corrected; /* 上次选定轴角速度原始计数Q16，供梯形积分。 */
    uint8_t axis;
    int8_t sign;
    bool calibrating,calibrated,axis_confirmed,enabled,running,fault;
    bool have_stamp,have_yaw_previous,feedback_valid; /* 时间/积分首帧与当前反馈有效标志。 */
    int32_t kp,kd,limit,correction; /* cps/deg、cps/(deg/s)、cps；默认10/1/150。 */
} heading_hold_t;
/* 功能：复位算法；h不可空；无返回，默认禁用航向。 */
void heading_init(heading_hold_t *h);
/* 功能：开始连续1000帧静止标定；h不可空；无返回，旧标定立即作废。 */
void heading_calibrate(heading_hold_t *h);
/* 功能：处理一帧完整新样本；h/acc/gyro不可空，valid表示新鲜且无FIFO积压。
 * stamp_us为单调递增的读出微秒时戳，允许uint32回绕；有效相邻间隔为1..19999us。
 * 无返回；首帧不积分，之后按实际间隔梯形积分；异常锁存，禁止自动恢复航向。
 * 读出时间仅为采样时刻的近似，不能补回未读出的瞬时运动，也不能给积压帧补新时间。
 */
void heading_sample(heading_hold_t *h,const int16_t acc[3],const int16_t gyro[3],bool valid,uint32_t stamp_us);
/* 功能：开始保持当前航向；h不可空，需标定和轴向确认；成功true，不产生运动输出。 */
bool heading_begin(heading_hold_t *h);
/* 功能：产生前进/后退直行的四通道修正目标；h/out不可空，base为±200..1000cps。
 * 通道1/2右侧、3/4左侧；返回false表示禁止运动；输出修正不超过150cps。
 */
bool heading_targets(heading_hold_t *h,int32_t base,int32_t out[4]);
#endif
