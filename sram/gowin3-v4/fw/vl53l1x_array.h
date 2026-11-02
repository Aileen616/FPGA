#ifndef ROBOT_VL53L1X_ARRAY_H
#define ROBOT_VL53L1X_ARRAY_H
#include "sensor_bus.h"
#define TOF_MAX_COUNT 3u

/* 引脚电平必须依据模块原理图显式选择，UNKNOWN不能启动，不能猜为3.3V模块。 */
typedef enum { TOF_IO_UNKNOWN = 0, TOF_IO_1V8, TOF_IO_AVDD } tof_io_mode_t;
typedef struct {
    uint8_t count;                     /* 1..3；本项目选3，次序前左/前中/前右。 */
    uint8_t address7[TOF_MAX_COUNT];    /* 各自独立的7位地址，0x08..0x77，排除默认0x29。 */
    uint8_t distance_mode;             /* 1短距、2长距；ST UM2510定义。 */
    uint16_t budget_ms, period_ms;     /* TB支持20/50/100/200/500ms；20ms仅短距；IMP>=TB+5ms。 */
    uint32_t fresh_us;                 /* 2*IMP..2000000us；安装实测后设置。 */
    tof_io_mode_t io_mode;             /* 需用户确认模块电平，不提供假定默认值。 */
} tof_config_t;
typedef struct {
    uint16_t distance_mm;              /* 原始距离，0..65535mm；只有VALID可用于控制。 */
    uint8_t raw_status, range_status;  /* 原始5位状态与ULD映射状态；0为ULD有效。 */
    uint8_t stream_count;              /* 硬件帧计数，8位自然回绕。 */
    uint32_t timestamp_us, sequence;   /* 主机取到新帧时间/序号；不是光学曝光时间。 */
    bool received;                    /* 区分尚无帧与8位/32位计数回绕。 */
    sensor_validity_t validity;
    sensor_result_t error;
} tof_sample_t;
typedef struct {
    sensor_bus_t bus;
    tof_config_t cfg;
    tof_sample_t sample[TOF_MAX_COUNT];
    uint32_t since_us, running_us;
    uint32_t last_poll_us[TOF_MAX_COUNT]; /* 检测主机长时间未轮询，不能给积压帧补新时间。 */
    uint8_t state, index, offset, active_addr, poll_index;
    uint8_t report_mask;               /* 本次新帧/通信失败的通道位；不可当成周期心跳。 */
    sensor_result_t error;
} vl53l1x_array_t;

/* 功能：全部XSHUT拉低并开始逐只上电改址；只在车辆禁能时调用。
 * 参数：a/bus/cfg不可为空；三个I2C/XSHUT回调必需；配置范围见结构体。
 * 返回：PENDING已启动；E_ARG配置非法；E_IO无法关断。失败不自动重试。
 */
sensor_result_t vl53l1x_array_start(vl53l1x_array_t *a, const sensor_bus_t *bus,
                                   const tof_config_t *cfg, uint32_t now_us);
/* 功能：推进初始化或轮询一路测距；无等待循环；每次先重新计算所有数据新鲜度。
 * 参数：a不可为空、now_us为当前微秒，主循环调用周期<=1ms。
 * 返回：OK新帧/初始化完成，PENDING无数据，负数为错误；初始化失败锁定，运行失败作废对应通道。
 */
sensor_result_t vl53l1x_array_step(vl53l1x_array_t *a, uint32_t now_us);
/* 功能：查询全阵列初始化是否完成；a可为空；返回true完成，false未完成。 */
bool vl53l1x_array_ready(const vl53l1x_array_t *a);
/* 功能：不访问总线地标记过期数据；a不可为空，now_us当前微秒；无返回值。 */
void vl53l1x_array_health(vl53l1x_array_t *a, uint32_t now_us);
/* 功能：关闭全部模块并作废所有样本；a不可为空且已start；返回OK/E_ARG/E_IO。
 * XSHUT释放后地址丢失，下次必须重新执行完整start改址流程。
 */
sensor_result_t vl53l1x_array_stop(vl53l1x_array_t *a);
#endif
