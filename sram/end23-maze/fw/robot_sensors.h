#ifndef ROBOT_SENSORS_H
#define ROBOT_SENSORS_H
#include "bmi088.h"
#include "vl53l1x_array.h"

typedef struct {
    tof_config_t tof;
    uint32_t imu_fresh_us;       /* 10000..1000000us，建议起点30000us。 */
    uint16_t stop_mm, slow_mm;   /* 实测制动距离决定，必须0<stop<slow<=4000；无默认值。 */
} robot_sensor_config_t;
typedef struct {
    bool sensor_fault, obstacle_stop, slowdown;
    uint16_t nearest_mm;        /* 有效ToF最小距离；没有有效值时0，不可当成安全距离。 */
    uint16_t speed_scale_q15;   /* 无符号0..32767；仅正常减速比例，故障/停车时为0。 */
} sensor_guard_t;
typedef struct {
    bmi088_t imu;
    vl53l1x_array_t tof;
    robot_sensor_config_t cfg;
    sensor_guard_t guard;
    sensor_result_t imu_result, tof_result;
} robot_sensors_t;

/* 功能：开始IMU及三路ToF初始化，初始化期间guard默认禁止运动。
 * 参数：r/bus/cfg非空；now_us当前微秒；cfg必须指定电平、实测阈值和三只地址。
 * 返回：PENDING已启动，负数失败；不操作电机、Flash或FPGA引脚映射。
 */
sensor_result_t robot_sensors_start(robot_sensors_t *r, const sensor_bus_t *bus,
                                    const robot_sensor_config_t *cfg, uint32_t now_us);
/* 功能：主循环执行采样及软件保护建议，每轮刷新guard。
 * 参数：r非空，events可为空表示本轮无IMU边沿，now_us来自单调微秒计时器。
 * 返回：无；详细状态在imu_result/tof_result/guard，tof.report_mask供硬件发布新帧。
 * 不在ISR调用；BSP临界区只用于原子取走事件，SPI/I2C均在主循环。
 */
void robot_sensors_poll(robot_sensors_t *r, const bmi088_events_t *events, uint32_t now_us);
/* 功能：从当前有效性和距离计算保护建议，不清除硬件故障锁存。
 * 参数：r非空；读取已经更新的新鲜度；无返回值。
 * 最终使能必须经过既有RTL safety_supervisor，不能直接使用speed_scale控制EN。
 */
void robot_sensors_guard(robot_sensors_t *r);
#endif
