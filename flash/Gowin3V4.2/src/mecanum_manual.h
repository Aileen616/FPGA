#ifndef MECANUM_MANUAL_H
#define MECANUM_MANUAL_H
#include "mecanum_mixer.h"

/* 方向编号与手机MOTOR3协议一致；前/左/逆时针为正，滚子采用标准X布局。 */
typedef enum {
    MANUAL_FORWARD=1, MANUAL_BACKWARD=2, MANUAL_LEFT=3, MANUAL_RIGHT=4,
    MANUAL_CCW=5, MANUAL_CW=6, MANUAL_FRONT_LEFT=7, MANUAL_FRONT_RIGHT=8,
    MANUAL_BACK_LEFT=9, MANUAL_BACK_RIGHT=10
} mecanum_manual_key_t;

/* 功能：将离散遥控方向和输出档位映射为四轮驱动力。
 * 参数：key为1..10方向；level为1..3，对应幅值4096/12288/16384；output不可为空。
 * 返回：合法true；非法false，非空output清零；输出固定FL/FR/RL/RR，不含电气翻转。
 */
bool mecanum_manual_mix(uint32_t key,uint32_t level,mecanum_output_t *output);
#endif
