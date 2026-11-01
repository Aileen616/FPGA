#ifndef SENSOR_BUS_H
#define SENSOR_BUS_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

/* 结果：0成功；1等待；负数为参数、总线、芯片、配置、超时或状态错误。 */
typedef enum {
    SENSOR_OK = 0, SENSOR_PENDING = 1, SENSOR_E_ARG = -1,
    SENSOR_E_IO = -2, SENSOR_E_ID = -3, SENSOR_E_CONFIG = -4,
    SENSOR_E_TIMEOUT = -5, SENSOR_E_STATE = -6
} sensor_result_t;

/* INVALID包含未采到数据/通信或质量错误；STALE专指曾有数据但已过期。 */
typedef enum { SENSOR_INVALID = 0, SENSOR_VALID, SENSOR_STALE } sensor_validity_t;

/* 板级接口：无全局HAL句柄；ctx由实际BX72 SDK适配层持有，可为空。
 * 所有回调返回0成功、非0失败；总线回调必须在timeout_us内返回，禁止无限等待。
 * SPI：chip=0加速度CS、1陀螺仪CS；模式0或3、MSB先发，初测建议1MHz。
 * tx/rx不可为空、len为1..16；整个传输保持一个CS，结束/出错必须释放CS。
 * I2C：addr7是7位地址，reg为16位寄存器地址（先发高字节）；读使用重复START。
 * data不可为空、len为1..32；BSP内部若需8位地址，只在BSP左移一次。
 * xshut：index为0..2（前左/前中/前右）；released=false拉低，true释放；仅短GPIO操作。
 * XSHUT应按模块电平开漏/电平转换释放，不能默认用FPGA的3.3V推挽拉高。
 */
typedef struct {
    void *ctx;
    int (*spi)(void *ctx, uint8_t chip, const uint8_t *tx, uint8_t *rx,
               size_t len, uint32_t timeout_us);
    int (*i2c_read)(void *ctx, uint8_t addr7, uint16_t reg, uint8_t *data,
                    size_t len, uint32_t timeout_us);
    int (*i2c_write)(void *ctx, uint8_t addr7, uint16_t reg, const uint8_t *data,
                     size_t len, uint32_t timeout_us);
    int (*xshut)(void *ctx, uint8_t index, bool released);
} sensor_bus_t;

/* 功能：判断时间间隔是否到期，支持uint32微秒计时器回绕。
 * 参数：now/then为同一时钟域时间；interval为1..0x7fffffff us。
 * 返回：true到期；false未到期。调用间隔和事件年龄必须小于半个计数周期。
 */
static inline bool sensor_elapsed(uint32_t now, uint32_t then, uint32_t interval)
{
    return (uint32_t)(now - then) >= interval;
}
#endif
