#ifndef MAZE_ODOMETRY_H
#define MAZE_ODOMETRY_H
#include <stdint.h>
#include <stdbool.h>

typedef struct {
    uint32_t counts_per_m_milli[4]; /* FL/FR/RL/RR每米计数的千分单位；默认复用5268.945counts/m。 */
} mo_config_t;
typedef struct {
    uint32_t count[4], seq, stamp_ms; /* 已按实际物理位归一正方向的同组四轮计数；自然回绕。 */
    int32_t yaw_md;                 /* 相对初始朝向，右转为正，毫度。 */
    uint8_t valid;                  /* 仅说明快照可用于里程；不参与运动准入。 */
} mo_input_t;
typedef struct {
    mo_config_t cfg;
    uint32_t previous[4], seq, stamp_ms, rejected;
    int64_t remainder[4], x_um, y_um; /* 换算余数与累计微米；x向初始右、y向初始前。 */
    int64_t rotate_remainder[2];    /* 保存定点旋转除法余数，避免低速每帧取整积累偏差。 */
    int32_t forward_remainder,right_remainder; /* 四轮平均后的微米余数，绝对值小于4。 */
    int32_t x_mm, y_mm;
    uint8_t seen, valid;            /* 异常仅标记地图里程，不清零导航或改变电机输出。 */
} mo_t;

/* 功能：建立里程参数；c可为空；返回无，非空时写入原项目换算系数。 */
void mo_defaults(mo_config_t *c);
/* 功能：初始化旁路里程；o/c不可为空；返回参数是否有效，不操作外设。 */
bool mo_init(mo_t *o,const mo_config_t *c);
/* 功能：消费同组四轮新快照；o/in不可为空；返回是否推进，重复或无效样本返回false。 */
bool mo_update(mo_t *o,const mo_input_t *in);
#endif
