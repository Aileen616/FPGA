#ifndef BX72_IMU_DISPLAY_H
#define BX72_IMU_DISPLAY_H
#include "bmi088.h"
#define IMU_TEXT_BYTES 512u
/* 显示状态：启动/等待首样本/有效/过期/通信或配置错误；不是姿态角。 */
typedef enum { IMU_INIT, IMU_WAIT, IMU_LIVE, IMU_STALE, IMU_ERROR } imu_screen_state_t;
/* 功能：生成32列16行ASCII页，含三轴角速度和加速度，后三行留空。
 * 参数：text须512字节；d非空；state为上列枚举；now_us为当前微秒；overruns为DRDY覆盖次数。
 * 返回：无；除LIVE外所有数值用短横线代替，不把未更新数据当作实时值。
 */
void imu_format_screen(char text[IMU_TEXT_BYTES], const bmi088_t *d,
                       imu_screen_state_t state, uint32_t now_us, uint32_t overruns);
#endif
