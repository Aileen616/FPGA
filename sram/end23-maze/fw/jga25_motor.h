#ifndef JGA25_MOTOR_H
#define JGA25_MOTOR_H
#include "mecanum_mixer.h"
/* 用户确认的铭牌信息；280RPM不能当作负载下保证转速。 */
#define JGA25_NOMINAL_VOLTAGE_MV UINT32_C(12000)
#define JGA25_LABEL_SPEED_MRPM UINT32_C(280000)
/* 四轮顺序FL/FR/RL/RR；每输出轴转计数已包含AB四倍频及减速比。
 * counts_per_rev为1..INT32_MAX；max_mrpm为实测可达目标上限1..280000。
 * 不提供猜测的默认分辨率；全零配置明确无效。 */
typedef struct {
    uint32_t counts_per_rev[4];
    uint32_t max_mrpm[4];
} jga25_calibration_t;
/* 功能：获取新电机开环起步配置；无参数；返回25%驱动力上限、极性待标定。
 * 此限幅不是限流或限速，不能替代电源保护及编码器反馈。 */
mecanum_config_t jga25_open_loop_config(void);
/* 功能：四轮毫RPM目标换算成counts/s；全部指针不可空，输入不得超过标定上限。
 * 返回成功true；配置缺失、越界或溢出false，并清零非空输出；不访问硬件。 */
bool jga25_targets_cps(const jga25_calibration_t *cal, const int32_t mrpm[4], int32_t cps[4]);
#endif
