#include "mecanum_mixer.h"
#include <stddef.h>

/* 算法参考及许可证见references/mecanum/README.md；此文件为独立整数实现。
 * 不依赖STM32 HAL、RTOS或任何软核SDK；无外部全局变量和外设句柄。
 */

/* 功能：取得默认配置；参数：无；返回：约25%驱动力上限和未翻转方向。 */
mecanum_config_t mecanum_default_config(void)
{
    const mecanum_config_t config = {8192U, 0U};
    return config;
}

/* 功能：验证幅值及方向位；参数：config不可为空；返回：合法true，否则false。 */
bool mecanum_config_valid(const mecanum_config_t *config)
{
    return config != NULL && config->max_effort > 0U &&
           config->max_effort <= 32767U && config->invert_mask <= 15U;
}

/* 功能：生成四轮开环驱动力；参数：三个指针不可为空，三轴范围-1000..1000。
 * 返回：成功true；失败false并将非空output清零，调用层应同时禁能。
 */
bool mecanum_mix(const mecanum_config_t *config, const mecanum_command_t *command,
                 mecanum_output_t *output)
{
    int32_t mixed[4]; /* 混合量范围-3000..3000；与32767相乘不会溢出signed32。 */
    int32_t denominator = 1000;
    unsigned int i;
    if (output == NULL) {
        return false;
    }
    for (i = 0U; i < 4U; ++i) {
        output->effort[i] = 0;
    }
    if (!mecanum_config_valid(config) || command == NULL ||
        command->forward < -1000 || command->forward > 1000 ||
        command->left < -1000 || command->left > 1000 ||
        command->ccw < -1000 || command->ccw > 1000) {
        return false;
    }
    mixed[MECANUM_FL] = (int32_t)command->forward - command->left - command->ccw;
    mixed[MECANUM_FR] = (int32_t)command->forward + command->left + command->ccw;
    mixed[MECANUM_RL] = (int32_t)command->forward + command->left - command->ccw;
    mixed[MECANUM_RR] = (int32_t)command->forward - command->left + command->ccw;
    for (i = 0U; i < 4U; ++i) {
        const int32_t magnitude = mixed[i] < 0 ? -mixed[i] : mixed[i];
        if (magnitude > denominator) {
            denominator = magnitude;
        }
    }
    /* 使用共同分母，避免分别截断四轮幅值改变期望的运动方向比例。 */
    for (i = 0U; i < 4U; ++i) {
        int32_t effort = mixed[i] * (int32_t)config->max_effort / denominator;
        if ((config->invert_mask & (1U << i)) != 0U) {
            effort = -effort;
        }
        output->effort[i] = (int16_t)effort;
    }
    return true;
}
