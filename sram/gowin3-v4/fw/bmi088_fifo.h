#ifndef BMI088_FIFO_H
#define BMI088_FIFO_H
#include "bmi088.h"
/* 五线SPI采样状态；沿用200Hz/±6g/±500度每秒初始化，时间戳是读取时间。 */
typedef struct {
    bmi088_t core;
    uint32_t last_poll_us, gyro_us, acc_us, gyro_seq, acc_seq;
    uint32_t rejected_frames; /* 三轴同时0x8000的异常帧累计数，不计入有效序号，不刷新新鲜度。 */
    uint8_t phase, acc_id, gyro_id;
    uint8_t fifo_status, max_fifo; /* 原始FIFO状态/历史最大帧数；只作诊断，不放宽积压保护。 */
    uint8_t fifo_before, fifo_after, config0, config1, fault_stage;
    /* 诊断阶段：1配置、2身份、3加速度、4读前状态、5读数据、6读后状态、7服务间隔、8异常帧。 */
    uint32_t poll_gap_us, max_poll_gap_us; /* 实际两次轮询间隔及最大值，单位us。 */
    int16_t acc[3], gyro[3];
    bool new_gyro, valid;
    sensor_result_t error;
} bmi088_fifo_t;
/* 功能：启动五线SPI驱动；d/bus不可空，now为微秒；返回初始化结果。 */
sensor_result_t bmi088_fifo_start(bmi088_fifo_t *d,const sensor_bus_t *bus,uint32_t now);
/* 功能：推进初始化或最多读取一帧FIFO及一帧加速度；d不可空，now为微秒。
 * 返回OK新陀螺样本、PENDING等待、负数锁存故障；超过4帧积压或20ms失鲜时不可用于航向。
 */
sensor_result_t bmi088_fifo_step(bmi088_fifo_t *d,uint32_t now);
/* 功能：推进初始化或按真实DRDY读取数据寄存器（不启用FIFO）。
 * 参数：d/events不可空，now为同源微秒；过期事件不读取，调用方须检查读期间边沿竞争。
 * 返回：OK有新陀螺样本、PENDING等待、负数锁存错误；保留20ms失鲜和异常帧保护。
 */
sensor_result_t bmi088_irq_step(bmi088_fifo_t *d,const bmi088_events_t *events,uint32_t now);
#endif
