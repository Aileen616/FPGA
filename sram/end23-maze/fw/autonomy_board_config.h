#ifndef AUTONOMY_BOARD_CONFIG_H
#define AUTONOMY_BOARD_CONFIG_H
/* 生产配置：几何来自用户；制动/安装确认尚无新实测证据，默认禁止自主起步。
 * 修改须有对应测量记录；HOST测试通过运行时夹具注入模型，不改变生产常量。
 */
#define AUTO_BRAKE_VERIFIED 0
#define AUTO_BRAKE_DECEL_MM_S2 0U
#define AUTO_OFFSET_KNOWN_MASK 0U
#define AUTO_ROTATION_ENVELOPE_VERIFIED 0
#endif
