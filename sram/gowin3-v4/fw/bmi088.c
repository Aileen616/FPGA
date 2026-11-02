#include "bmi088.h"
#include <string.h>

/* 寄存器和时序来源：Bosch BMI088 DS rev1.9，章节3、5、6。
 * 不使用MCU物理寄存器；这里只通过总线接口访问传感器寄存器。
 */
enum { B_WAIT, B_DUMMY, B_RESET_A, B_DUMMY_AGAIN, B_RESET_G,
       B_IDS, B_WRITE, B_VERIFY, B_READY, B_FAULT };
typedef struct { uint8_t chip, reg, value, mask; } config_t;
static const config_t config[] = {
    {0, 0x7c, 0x00, 0xff}, {0, 0x7d, 0x04, 0xff},
    {0, 0x40, 0xa9, 0xff}, {0, 0x41, 0x01, 0x03},
    {0, 0x53, 0x0a, 0x1f}, {0, 0x58, 0x04, 0x44},
    {1, 0x11, 0x00, 0xff}, {1, 0x0f, 0x02, 0x07},
    {1, 0x10, 0x04, 0x07}, {1, 0x16, 0x01, 0x0f},
    {1, 0x18, 0x01, 0x85}, {1, 0x15, 0x80, 0xc0}
};

/* 功能：读取传感器连续寄存器，处理加速度独有的dummy字节。
 * 参数：d/out不可为空；chip=0/1；reg=0..127；n=1..12字节。
 * 返回：OK成功、E_IO传输失败。总线回调负责CS生命周期。
 */
static sensor_result_t rd(bmi088_t *d, uint8_t chip, uint8_t reg, uint8_t *out, size_t n)
{
    uint8_t tx[16] = {0}, rx[16] = {0};
    size_t offset = chip == 0 ? 2u : 1u;
    tx[0] = (uint8_t)(reg | 0x80u);
    if (d->bus.spi(d->bus.ctx, chip, tx, rx, n + offset, 2000) != 0) return SENSOR_E_IO;
    memcpy(out, rx + offset, n);
    return SENSOR_OK;
}
/* 功能：写单个传感器寄存器；d不可为空，chip=0/1，reg<128，v为8位值。
 * 返回：OK成功、E_IO失败。写操作没有加速度dummy字节。
 */
static sensor_result_t wr(bmi088_t *d, uint8_t chip, uint8_t reg, uint8_t v)
{
    uint8_t tx[2] = {reg, v}, rx[2];
    return d->bus.spi(d->bus.ctx, chip, tx, rx, 2, 2000) == 0 ? SENSOR_OK : SENSOR_E_IO;
}
/* 功能：锁定故障并作废缓存；d非空、e为负错误码；返回e。 */
static sensor_result_t fail(bmi088_t *d, sensor_result_t e)
{
    d->state = B_FAULT; d->error = e; d->sample.validity = SENSOR_INVALID;
    return e;
}
/* 功能：解码小端signed16，避免越界强制类型转换；p指向至少2字节；返回原始计数。 */
static int16_t le16(const uint8_t *p)
{
    int32_t v = (int32_t)p[0] + ((int32_t)p[1] << 8);
    return (int16_t)(v >= 32768 ? v - 65536 : v);
}

/* 功能/参数/返回：见bmi088.h；拷贝bus避免调用者传入d内部成员时被清零。 */
sensor_result_t bmi088_start(bmi088_t *d, const sensor_bus_t *bus,
                            uint32_t now_us, uint32_t fresh_us)
{
    sensor_bus_t copy;
    if (!d || !bus || !bus->spi || fresh_us < 10000u || fresh_us > 1000000u) return SENSOR_E_ARG;
    copy = *bus; memset(d, 0, sizeof(*d)); d->bus = copy;
    d->fresh_us = fresh_us; d->state = B_WAIT; d->since_us = now_us; d->wait_us = 1000;
    return SENSOR_PENDING;
}
/* 功能/参数/返回：见bmi088.h。所有等待通过时间差完成，不使用阻塞延时。 */
sensor_result_t bmi088_init_step(bmi088_t *d, uint32_t now_us)
{
    uint8_t a = 0, g = 0;
    sensor_result_t rc = SENSOR_OK;
    const config_t *c;
    if (!d || !d->bus.spi) return SENSOR_E_ARG;
    if (d->state == B_FAULT) return d->error;
    if (d->state == B_READY) return SENSOR_OK;
    if (!sensor_elapsed(now_us, d->since_us, d->wait_us)) return SENSOR_PENDING;
    d->since_us = now_us; d->wait_us = 1000;
    switch (d->state) {
    case B_WAIT: d->state = B_DUMMY; break;
    case B_DUMMY:
        rc = rd(d, 0, 0x00, &a, 1); d->state = B_RESET_A; break;
    case B_RESET_A:
        rc = wr(d, 0, 0x7e, 0xb6); d->state = B_DUMMY_AGAIN; break;
    case B_DUMMY_AGAIN:
        rc = rd(d, 0, 0x00, &a, 1); d->state = B_RESET_G; break;
    case B_RESET_G:
        rc = wr(d, 1, 0x14, 0xb6); d->wait_us = 30000; d->state = B_IDS; break;
    case B_IDS:
        rc = rd(d, 0, 0, &a, 1);
        if (rc == SENSOR_OK) rc = rd(d, 1, 0, &g, 1);
        d->acc_id=a;d->gyro_id=g;
        if (rc == SENSOR_OK && (a != 0x1e || g != 0x0f)) rc = SENSOR_E_ID;
        d->state = B_WRITE; break;
    case B_WRITE:
        c = &config[d->config_index]; rc = wr(d, c->chip, c->reg, c->value);
        /* 保守采用Bosch API的5ms电源配置间隔，也覆盖普通寄存器写间隔。 */
        d->wait_us = 5000; d->state = B_VERIFY; break;
    case B_VERIFY:
        c = &config[d->config_index]; rc = rd(d, c->chip, c->reg, &a, 1);
        d->diag_chip=c->chip;d->diag_reg=c->reg;d->diag_expected=c->value;d->diag_actual=a;d->diag_mask=c->mask;
        if (rc == SENSOR_OK && (a & c->mask) != (c->value & c->mask)) rc = SENSOR_E_CONFIG;
        if (++d->config_index == sizeof(config) / sizeof(config[0])) d->state = B_READY;
        else d->state = B_WRITE;
        break;
    default: rc = SENSOR_E_STATE; break;
    }
    if (rc < 0) return fail(d, rc);
    return d->state == B_READY ? SENSOR_OK : SENSOR_PENDING;
}
/* 功能/参数/返回：见bmi088.h。 */
bool bmi088_ready(const bmi088_t *d) { return d && d->state == B_READY; }
/* 功能/参数/返回：见bmi088.h；读失败后两路必须重新采到才能重新有效。 */
sensor_validity_t bmi088_health(bmi088_t *d, uint32_t now_us)
{
    if (!d) return SENSOR_INVALID;
    if (!bmi088_ready(d) || d->sample.have_mask != 3u) d->sample.validity = SENSOR_INVALID;
    else if (sensor_elapsed(now_us, d->sample.acc_us, d->fresh_us) ||
             sensor_elapsed(now_us, d->sample.gyro_us, d->fresh_us)) d->sample.validity = SENSOR_STALE;
    else d->sample.validity = SENSOR_VALID;
    return d->sample.validity;
}
/* 功能/参数/返回：见bmi088.h。不把读失败、旧事件或零填充值作为新样本。 */
sensor_result_t bmi088_read(bmi088_t *d, const bmi088_events_t *events, uint32_t now_us)
{
    uint8_t data[9], id, chip, bit;
    bool updated = false;
    sensor_result_t rc;
    if (!d || !events || (events->mask & ~3u)) return SENSOR_E_ARG;
    if (!bmi088_ready(d)) return SENSOR_E_STATE;
    for (chip = 0; chip < 2; ++chip) {
        uint32_t stamp = chip == 0 ? events->acc_us : events->gyro_us;
        uint32_t old = chip == 0 ? d->sample.acc_us : d->sample.gyro_us;
        bit = (uint8_t)(1u << chip);
        if (!(events->mask & bit)) continue;
        if (sensor_elapsed(now_us, stamp, d->fresh_us)) continue;
        if ((d->seen_mask & bit) && (stamp == old || (uint32_t)(stamp - old) >= 0x80000000u)) continue;
        rc = rd(d, chip, 0x00, &id, 1);
        if (rc == SENSOR_OK && id != (chip == 0 ? 0x1e : 0x0f)) rc = SENSOR_E_ID;
        if (rc == SENSOR_OK) rc = rd(d, chip, chip == 0 ? 0x12 : 0x02, data, chip == 0 ? 9 : 6);
        if (rc != SENSOR_OK) {
            d->error = rc; d->sample.have_mask = 0; d->sample.validity = SENSOR_INVALID;
            return rc;
        }
        if (chip == 0) {
            for (unsigned i = 0; i < 3; ++i) {
                d->sample.acc_raw[i] = le16(data + 2 * i);
                d->sample.acc_mps2[i] = d->sample.acc_raw[i] * (6.0f * 9.80665f / 32768.0f);
            }
            d->sample.sensor_time = (uint32_t)data[6] | ((uint32_t)data[7] << 8) | ((uint32_t)data[8] << 16);
            d->sample.acc_us = stamp; ++d->sample.acc_seq;
        } else {
            for (unsigned i = 0; i < 3; ++i) {
                d->sample.gyro_raw[i] = le16(data + 2 * i);
                d->sample.gyro_radps[i] = d->sample.gyro_raw[i] * (500.0f * 0.01745329252f / 32768.0f);
            }
            d->sample.gyro_us = stamp; ++d->sample.gyro_seq;
        }
        d->sample.have_mask |= bit; d->seen_mask |= bit; updated = true;
    }
    bmi088_health(d, now_us);
    if (updated) d->error = SENSOR_OK;
    return updated ? SENSOR_OK : SENSOR_PENDING;
}
