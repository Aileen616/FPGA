#ifndef ROBOT_BMI088_H
#define ROBOT_BMI088_H
#include "sensor_bus.h"

#define BMI088_ACCEL_READY 1u
#define BMI088_GYRO_READY  2u

/* 由BSP/FPGA锁存DRDY边沿后在主循环原子取走；不能用定时器伪造就绪。
 * acc_us/gyro_us为对应真实边沿时间，uint32微秒。两部分异步，不宣称硬件同步。
 */
typedef struct { uint8_t mask; uint32_t acc_us, gyro_us; } bmi088_events_t;
typedef struct {
    int16_t acc_raw[3], gyro_raw[3]; /* 芯片轴XYZ，signed16；未作安装轴变换。 */
    float acc_mps2[3], gyro_radps[3]; /* ±6g、±500度/s换算后的SI单位。 */
    uint32_t acc_us, gyro_us;        /* 最近接受的DRDY边沿时间，us。 */
    uint32_t sensor_time;            /* 加速度24位时钟，1LSB=39.0625us。 */
    uint32_t acc_seq, gyro_seq;      /* 仅在真实新样本上递增，自然回绕。 */
    uint8_t have_mask;              /* 已获得加速度/陀螺仪数据的位掩码。 */
    sensor_validity_t validity;
} bmi088_sample_t;

typedef struct {
    sensor_bus_t bus;
    bmi088_sample_t sample;
    uint32_t since_us, wait_us, fresh_us; /* 初始化等待与有效期，单位us。 */
    uint8_t state, config_index, seen_mask; /* 私有状态、配置下标、历史事件标志。 */
    sensor_result_t error;
    uint8_t diag_chip,diag_reg,diag_expected,diag_actual,diag_mask; /* 最近配置验证上下文，8位寄存器值。 */
    uint8_t acc_id,gyro_id; /* 初始化实际读取的芯片ID，即使失败也保留。 */
} bmi088_t;

/* 功能：开始异步初始化，固定200Hz、±6g、±500度/s、INT1/INT3高有效。
 * 参数：d/bus不可为空；bus.spi必须实现；now_us为系统微秒；fresh_us范围10000..1000000。
 * 返回：PENDING已启动；E_ARG参数错误。只修改上下文，不等待硬件。
 */
sensor_result_t bmi088_start(bmi088_t *d, const sensor_bus_t *bus,
                            uint32_t now_us, uint32_t fresh_us);
/* 功能：推进复位、ID校验和逐项写入/读回；每次至多两次短SPI传输。
 * 参数：d不可为空，now_us为当前微秒；主循环周期建议不大于1ms。
 * 返回：OK完成、PENDING等待、负数故障；故障后必须显式start重试。
 */
sensor_result_t bmi088_init_step(bmi088_t *d, uint32_t now_us);
/* 功能：读取由DRDY确认的新数据并换算；总线失败时立即取消数据有效性。
 * 参数：d/events不可为空；事件由BSP原子消费一次，时间不能过期/来自未来。
 * 返回：OK至少更新一路；PENDING无新事件；负数表示失败。
 * 两部分读数分别有时间戳，不允许在中断中调用此函数。
 */
sensor_result_t bmi088_read(bmi088_t *d, const bmi088_events_t *events, uint32_t now_us);
/* 功能：不访问总线地更新样本新鲜度。
 * 参数：d不可为空；now_us为同源时钟。返回INVALID/VALID/STALE；空指针返回INVALID。
 */
sensor_validity_t bmi088_health(bmi088_t *d, uint32_t now_us);
/* 功能：查询初始化完成状态；d可为空；返回true表示可读，false表示未就绪。 */
bool bmi088_ready(const bmi088_t *d);
#endif
