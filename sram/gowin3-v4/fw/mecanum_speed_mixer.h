#ifndef MECANUM_SPEED_MIXER_H
#define MECANUM_SPEED_MIXER_H
#include <stdbool.h>
#include <stdint.h>
/* X型麦克纳姆布局，沿用现有混合器：+x向前、+y向左、+z逆时针；轮序FL/FR/RL/RR。
 * 几何与每圈计数必须由实际底盘提供，不设虚构默认轮径、轮距或编码器分辨率。
 */
typedef struct {
    double half_wheelbase_m,half_track_m; /* 车心到轮轴前后/左右距离，单位m，正且有限。 */
    double wheel_diameter_m[4]; /* 四轮有效直径，m，正且有限。 */
    uint32_t counts_per_rev[4]; /* 车轮一圈有效计数，含实际减速比和解码倍率，1..UINT32_MAX。 */
    uint32_t max_cps[4]; /* 每轮实测允许目标上限，1..INT32_MAX且不得超过RTL逻辑上限。 */
} mecanum_speed_geometry_t;
typedef struct {
    double forward_m_s,left_m_s,ccw_rad_s; /* 均为有限实数；机体速度，尚未经过共同限幅。 */
} mecanum_speed_command_t;
/* 功能：把机体速度转换为四轮counts/s，并用共同缩放比例满足各轮速度上限。
 * 参数：geometry/command/out不可空；scale可空，非空时返回实际共同缩放比例0..1。
 * 返回：成功true，非法参数或浮点溢出false；失败时四轮输出和非空scale清零。
 * 说明：只做运动学，不直接访问硬件；电机和编码器电气方向在各自RTL适配层校准。
 */
bool mecanum_speed_targets(const mecanum_speed_geometry_t *geometry,
                           const mecanum_speed_command_t *command,
                           int32_t out[4],double *scale);
#endif
