#ifndef MECANUM_MIXER_H
#define MECANUM_MIXER_H

#include <stdbool.h>
#include <stdint.h>

/* 轮序固定；俯视车体，前方为+x、左方为+y、逆时针为+z，使用X型滚子布局。 */
typedef enum {
    MECANUM_FL = 0, MECANUM_FR = 1, MECANUM_RL = 2, MECANUM_RR = 3,
    MECANUM_WHEEL_COUNT = 4
} mecanum_wheel_t;

/* 三轴均为无量纲操作量，范围-1000..1000；不表示m/s或rad/s。 */
typedef struct {
    int16_t forward;
    int16_t left;
    int16_t ccw;
} mecanum_command_t;

typedef struct {
    uint16_t max_effort; /* 四轮共同幅值上限，1..32767；起步8192约25%占空比，须核对驱动。 */
    uint8_t invert_mask; /* bit0..3翻转对应电机电气方向；默认0，实际极性需要逐轮核对。 */
} mecanum_config_t;

typedef struct {
    int16_t effort[4]; /* FL/FR/RL/RR有符号驱动力，-32767..32767，无转速反馈含义。 */
} mecanum_output_t;

/* 功能：获得起步配置；参数：无；返回：限幅8192、方向翻转掩码0。 */
mecanum_config_t mecanum_default_config(void);

/* 功能：检查配置范围；参数：config不可为空；返回：合法为true，否则false。 */
bool mecanum_config_valid(const mecanum_config_t *config);

/* 功能：混合三轴并按同一比例限幅，再应用电机极性映射。
 * 参数：config、command、output均不可为空；输入范围见上述结构体。
 * 返回：成功true；非法参数false，output非空时保证四轮输出清零。
 * 说明：不动态分配内存、不操作硬件、不包含编码器PI；全部中间运算使用signed32。
 */
bool mecanum_mix(const mecanum_config_t *config, const mecanum_command_t *command,
                 mecanum_output_t *output);

#endif
