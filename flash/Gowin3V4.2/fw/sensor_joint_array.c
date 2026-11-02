/* 四路静态候选：派生自vl53l1x_array，原三路接口保留；I2C事务上限20ms。 */
/* Copyright (c) 2018 STMicroelectronics.
 * 寄存器配置/时序值移植自STM32duino VL53L1X；许可全文见LICENSE.ST。
 * 本项目将初始化改为有界状态机，统一7位地址，并逐次检查全部I2C返回值。
 */
#include "sensor_joint_array.h"
#include <string.h>
#include "vl53l1x_config.inc"

enum { T_OFF, T_RESET_WAIT, T_BOOT, T_CONFIG, T_CAL_START, T_CAL_WAIT,
       T_SETUP, T_START_ALL, T_RUNNING, T_FAULT };

/* 功能：读连续寄存器；a/p非空，reg为16位地址，n为1..32；返回OK/E_IO。 */
static sensor_result_t read_reg(vl53l1x_array_t *a, uint16_t reg, uint8_t *p, size_t n)
{
    return a->bus.i2c_read(a->bus.ctx, a->active_addr, reg, p, n, 20000) == 0 ? SENSOR_OK : SENSOR_E_IO;
}
/* 功能：写连续寄存器；a/p非空，reg为16位地址，n为1..32；返回OK/E_IO。 */
static sensor_result_t write_reg(vl53l1x_array_t *a, uint16_t reg, const uint8_t *p, size_t n)
{
    return a->bus.i2c_write(a->bus.ctx, a->active_addr, reg, p, n, 20000) == 0 ? SENSOR_OK : SENSOR_E_IO;
}
/* 功能：按大端写1/2/4字节值；a非空，n=1/2/4，v为对应无符号值；返回OK/E_IO。 */
static sensor_result_t write_value(vl53l1x_array_t *a, uint16_t reg, uint32_t v, size_t n)
{
    uint8_t p[4];
    for (size_t i = 0; i < n; ++i) p[n - 1 - i] = (uint8_t)(v >> (8u * i));
    return write_reg(a, reg, p, n);
}
/* 功能：读取芯片16位ID；a非空；返回OK、E_ID型号不符或E_IO。 */
static sensor_result_t check_id(vl53l1x_array_t *a)
{
    uint8_t p[2];
    sensor_result_t rc = read_reg(a, 0x010f, p, 2);
    if (rc != SENSOR_OK) return rc;
    return p[0] == 0xea && p[1] == 0xcc ? SENSOR_OK : SENSOR_E_ID;
}
/* 功能：读取GPIO控制及状态并按真实极性判断就绪；a/ready非空；返回OK/E_IO。 */
static sensor_result_t data_ready(vl53l1x_array_t *a, bool *ready)
{
    uint8_t p[2];
    /* 寄存器支持自动递增：同一次事务读取相邻控制/状态，仍按真实极性判断。 */
    sensor_result_t rc = read_reg(a, 0x0030, p, 2);
    if (rc == SENSOR_OK) {a->gpio_ready_level=(p[0]&0x10u)?0u:1u;*ready=(p[1]&1u)==a->gpio_ready_level;}
    return rc;
}
/* 功能：关断全部传感器，即使某个GPIO失败仍继续尝试其他通道。
 * 参数：a非空且已配置；返回OK或E_IO，不会把失败当成已关断。
 */
static sensor_result_t shutdown_all(vl53l1x_array_t *a)
{
    sensor_result_t rc = SENSOR_OK;
    for (uint8_t i = 0; i < a->cfg.count; ++i) {
        if (a->bus.xshut(a->bus.ctx, i, false) != 0) rc = SENSOR_E_IO;
        a->sample[i].validity = SENSOR_INVALID;
    }
    return rc;
}
/* 功能：记录初始化故障、尽力关闭所有模块；a非空，rc负数；返回最初错误。 */
static sensor_result_t init_fail(vl53l1x_array_t *a, sensor_result_t rc)
{
    (void)shutdown_all(a); a->state = T_FAULT; a->error = rc;
    return rc;
}
/* 功能：校验多机配置；c可为空；返回true合法、false非法，不访问硬件。 */
static bool valid_config(const tof_config_t *c)
{
    if (!c || c->count < 1 || c->count > TOF_MAX_COUNT ||
        (c->distance_mode != 1 && c->distance_mode != 2) ||
        (c->io_mode != TOF_IO_1V8 && c->io_mode != TOF_IO_AVDD)) return false;
    if (c->budget_ms != 20 && c->budget_ms != 50 && c->budget_ms != 100 &&
        c->budget_ms != 200 && c->budget_ms != 500) return false;
    if (c->distance_mode == 2 && c->budget_ms == 20) return false;
    if (c->period_ms < c->budget_ms + 5u || c->period_ms > 1000u ||
        c->fresh_us < 2u * c->period_ms * 1000u || c->fresh_us > 2000000u) return false;
    for (uint8_t i = 0; i < c->count; ++i) {
        if (c->address7[i] < 0x08 || c->address7[i] > 0x77 || c->address7[i] == 0x29) return false;
        for (uint8_t j = 0; j < i; ++j) if (c->address7[i] == c->address7[j]) return false;
    }
    return true;
}
/* 功能/参数/返回：见tof4_array.h。 */
sensor_result_t vl53l1x_array_start(vl53l1x_array_t *a, const sensor_bus_t *bus,
                                   const tof_config_t *cfg, uint32_t now_us)
{
    sensor_bus_t b; tof_config_t c;
    if (!a || !bus || !bus->i2c_read || !bus->i2c_write || !bus->xshut || !valid_config(cfg)) return SENSOR_E_ARG;
    b = *bus; c = *cfg; memset(a, 0, sizeof(*a)); a->bus = b; a->cfg = c;
    if (shutdown_all(a) != SENSOR_OK) return init_fail(a, SENSOR_E_IO);
    a->state = T_RESET_WAIT; a->since_us = now_us;
    return SENSOR_PENDING;
}
/* 功能/参数/返回：见tof4_array.h。 */
sensor_result_t vl53l1x_array_stop(vl53l1x_array_t *a)
{
    sensor_result_t rc;
    if (!a || !a->bus.xshut) return SENSOR_E_ARG;
    sensor_result_t original=a->error;
    rc = shutdown_all(a); a->state = original<0?T_FAULT:T_OFF; a->error = original<0?original:rc;
    return rc;
}
/* 功能/参数/返回：见tof4_array.h。 */
bool vl53l1x_array_ready(const vl53l1x_array_t *a) { return a && a->state == T_RUNNING; }
/* 功能/参数/返回：见tof4_array.h；错误的样本保持INVALID，不被超时逻辑美化。 */
void vl53l1x_array_health(vl53l1x_array_t *a, uint32_t now_us)
{
    if (!a) return;
    for (uint8_t i = 0; i < a->cfg.count; ++i) {
        tof_sample_t *s = &a->sample[i];
        /* 连续记录实际样本年龄；只统计有效到过期的转换，不用查询频率代替事件数。 */
        uint32_t age=now_us-s->timestamp_us;
        if(s->received&&age>a->max_age_us[i])a->max_age_us[i]=age;
        if (s->received && s->validity == SENSOR_VALID && sensor_elapsed(now_us, s->timestamp_us, a->cfg.fresh_us)) {
            ++a->stale_events[i];s->validity = SENSOR_STALE; s->error = SENSOR_E_TIMEOUT;
        }
        if (a->state == T_RUNNING && !s->received && sensor_elapsed(now_us, a->running_us, a->cfg.fresh_us))
            s->error = SENSOR_E_TIMEOUT;
    }
}
/* 功能：执行一条最终工作配置；a非空、offset为0..12；返回OK/E_IO/E_CONFIG。
 * 限定每次至多两次总线操作，避免长配置表阻塞主循环。
 */
static sensor_result_t setup_one(vl53l1x_array_t *a)
{
    static const uint16_t budgets[5] = {20,50,100,200,500};
    static const uint16_t ta[2][5] = {{0x0051,0x01ae,0x02e1,0x03e1,0x0591},
                                     {0x001e,0x00ad,0x01cc,0x02d9,0x048f}};
    static const uint16_t tb[2][5] = {{0x006e,0x01e8,0x0388,0x0496,0x05c1},
                                     {0x0022,0x00c6,0x01ea,0x02f8,0x04a4}};
    uint8_t p[2]; unsigned b = 0, m = a->cfg.distance_mode - 1u;
    sensor_result_t rc;
    while (budgets[b] != a->cfg.budget_ms) ++b;
    switch (a->offset) {
    case 0: return write_value(a, 0x0086, 1, 1);
    case 1: return write_value(a, 0x0087, 0, 1);
    case 2: return write_value(a, 0x0008, 9, 1);
    case 3: return write_value(a, 0x000b, 0, 1);
    case 4: return write_value(a, 0x004b, m ? 0x0a : 0x14, 1);
    case 5: return write_value(a, 0x0060, m ? 0x0f : 0x07, 1);
    case 6: return write_value(a, 0x0063, m ? 0x0d : 0x05, 1);
    case 7: return write_value(a, 0x0069, m ? 0xb8 : 0x38, 1);
    case 8: return write_value(a, 0x0078, m ? 0x0f0d : 0x0705, 2);
    case 9: return write_value(a, 0x007a, m ? 0x0e0e : 0x0606, 2);
    case 10: return write_value(a, 0x005e, ta[m][b], 2);
    case 11: return write_value(a, 0x0061, tb[m][b], 2);
    case 12:
        rc = read_reg(a, 0x00de, p, 2);
        if (rc == SENSOR_OK) {
            uint32_t pll = (((uint32_t)p[0] << 8) | p[1]) & 0x03ffu;
            if (pll == 0) return SENSOR_E_CONFIG;
            /* ULD系数1.075用43/40整数实现；最大1023*1000*43小于2^32。 */
            rc = write_value(a, 0x006c, pll * a->cfg.period_ms * 43u / 40u, 4);
        }
        return rc;
    default: return SENSOR_E_STATE;
    }
}
/* 功能：将原始5位状态映射到ULD状态；raw为0..31；返回0有效或对应非零质量错误。 */
static uint8_t range_status(uint8_t raw)
{
    switch (raw) {
    case 9: return 0; case 6: return 1; case 4: return 2; case 8: return 3;
    case 5: return 4; case 3: return 5; case 19: return 6; case 7: return 7;
    case 12: return 9; case 18: return 10; case 22: return 11; case 23: return 12;
    case 13: return 13; default: return 255;
    }
}
/* 功能：清中断后有界确认真实GPIO，避免延迟释放时多等一个四路轮转周期。
 * 参数：a非空且active_addr为当前通道，gpio非空；返回OK或E_IO。
 * 最多读取三次，未解除仍保留wait_ready_low；不能把写ACK或重复结果当新帧。
 */
static sensor_result_t confirm_clear(vl53l1x_array_t *a, uint8_t *gpio)
{
    for(unsigned attempt=0;attempt<3;attempt++){
        sensor_result_t rc=read_reg(a,0x0031,gpio,1);
        if(rc!=SENSOR_OK)return rc;
        if((*gpio&1u)!=a->gpio_ready_level)break;
    }
    return SENSOR_OK;
}

/* 功能：轮询一路结果，先完整读出并成功清中断后才发布样本。
 * 参数：a非空且RUNNING，now_us当前微秒；返回OK/PENDING/负错误码。
 */
static sensor_result_t poll_one(vl53l1x_array_t *a, uint32_t now_us)
{
    uint8_t p[3], distance[2], cleared_gpio=0, i = a->poll_index;
    bool ready = false;
    bool service_stale = sensor_elapsed(now_us, a->last_poll_us[i], a->cfg.fresh_us);
    tof_sample_t *s = &a->sample[i];
    sensor_result_t rc;
    a->poll_index = (uint8_t)((i + 1u) % a->cfg.count);
    a->last_poll_us[i] = now_us;
    a->active_addr = a->cfg.address7[i];
    rc = data_ready(a, &ready);
    if (rc == SENSOR_OK && !ready) { a->wait_ready_low[i] = false; return SENSOR_PENDING; }
    /* ST完整API只在back-to-back模式检查STREAM；本驱动使用0x40定时模式。
     * 用已清除后的未就绪->就绪交接防止旧帧续期，不强制STREAM递增。
     */
    if (rc == SENSOR_OK && a->wait_ready_low[i]) {
        /* 错过低电平后重新清除并实读确认；本次只恢复握手，绝不发布旧帧。 */
        rc=write_value(a,0x0086,1,1);
        if(rc==SENSOR_OK)rc=confirm_clear(a,&cleared_gpio);
        if(rc!=SENSOR_OK){s->validity=SENSOR_INVALID;s->error=rc;a->report_mask|=(uint8_t)(1u<<i);return rc;}
        a->wait_ready_low[i]=(cleared_gpio&1u)==a->gpio_ready_level;
        if(a->wait_ready_low[i])++a->clear_still_ready[i];else ++a->clear_confirmed[i];
        return SENSOR_PENDING;
    }
    /* 只取实际使用的状态/STREAM及距离；全部成功前不清中断、不发布半帧。 */
    if (rc == SENSOR_OK) rc = read_reg(a, 0x0089, p, sizeof(p));
    if (rc == SENSOR_OK) rc = read_reg(a, 0x0096, distance, sizeof(distance));
    if (rc == SENSOR_OK) rc = write_value(a, 0x0086, 1, 1);
    /* 联合调度可能漏过两次轮询间短暂的未就绪阶段；清除后立即读取真实GPIO，不假设写ACK等于清除成功。 */
    if (rc == SENSOR_OK) rc = confirm_clear(a, &cleared_gpio);
    if (rc != SENSOR_OK) {
        s->validity = SENSOR_INVALID; s->error = rc; a->report_mask |= (uint8_t)(1u << i);
        return rc;
    }
    a->wait_ready_low[i] = (cleared_gpio&1u)==a->gpio_ready_level;
    if(a->wait_ready_low[i])++a->clear_still_ready[i];else ++a->clear_confirmed[i];
    if (service_stale) {
        /* 主机停顿后首个积压结果的测量时间不可知：丢弃并清中断，等待下一帧。 */
        s->stream_count = p[2]; s->received = true;
        s->validity = SENSOR_STALE; s->error = SENSOR_E_TIMEOUT;
        a->report_mask |= (uint8_t)(1u << i); return SENSOR_E_TIMEOUT;
    }
    s->raw_status = p[0] & 0x1fu; s->range_status = range_status(s->raw_status);
    s->distance_mm = (uint16_t)(((uint16_t)distance[0] << 8) | distance[1]);
    /* 时间仍取轮询开始，保守计算新鲜度；不得后移到总线完成来美化延迟。 */
    if(s->received){uint32_t gap=now_us-s->timestamp_us;if(gap>a->max_publish_gap_us[i])a->max_publish_gap_us[i]=gap;}
    s->stream_count = p[2]; s->received = true; s->timestamp_us = now_us; ++s->sequence;
    s->validity = s->range_status == 0 && s->distance_mm > 0 && s->distance_mm <= 4000 ? SENSOR_VALID : SENSOR_INVALID;
    s->error = SENSOR_OK; a->report_mask |= (uint8_t)(1u << i);
    return SENSOR_OK;
}
/* 功能/参数/返回：见tof4_array.h。启动中先改址并验证，之后才释放下一只模块。 */
sensor_result_t vl53l1x_array_step(vl53l1x_array_t *a, uint32_t now_us)
{
    uint8_t p[8], boot = 0;
    bool ready = false;
    sensor_result_t rc = SENSOR_OK;
    if (!a || !a->bus.i2c_read || !a->bus.i2c_write || !a->bus.xshut) return SENSOR_E_ARG;
    a->report_mask = 0; vl53l1x_array_health(a, now_us);
    if (a->state == T_FAULT) return a->error;
    if (a->state == T_RUNNING) return poll_one(a, now_us);
    switch (a->state) {
    case T_RESET_WAIT:
        if (!sensor_elapsed(now_us, a->since_us, 1000)) return SENSOR_PENDING;
        if (a->bus.xshut(a->bus.ctx, a->index, true) != 0) return init_fail(a, SENSOR_E_IO);
        a->active_addr = 0x29; a->since_us = now_us; a->state = T_BOOT; break;
    case T_BOOT:
        if (sensor_elapsed(now_us, a->since_us, 100000)) return init_fail(a, SENSOR_E_TIMEOUT);
        if (!sensor_elapsed(now_us, a->since_us, 2000)) return SENSOR_PENDING;
        rc = read_reg(a, 0x00e5, &boot, 1);
        if (rc == SENSOR_OK && !(boot & 1u)) return SENSOR_PENDING;
        if (rc == SENSOR_OK) rc = check_id(a);
        if (rc == SENSOR_OK) rc = write_value(a, 0x0001, a->cfg.address7[a->index], 1);
        if (rc == SENSOR_OK) {
            a->active_addr = a->cfg.address7[a->index]; rc = check_id(a);
        }
        a->offset = 0; a->state = T_CONFIG; break;
    case T_CONFIG: {
        size_t n = sizeof(vl53l1x_defaults) - a->offset;
        if (n > sizeof(p)) n = sizeof(p);
        memcpy(p, vl53l1x_defaults + a->offset, n);
        if (a->offset == 0) {
            p[1] = p[2] = a->cfg.io_mode == TOF_IO_AVDD ? 1 : 0;
        }
        rc = write_reg(a, (uint16_t)(0x002d + a->offset), p, n);
        a->offset = (uint8_t)(a->offset + n);
        if (a->offset == sizeof(vl53l1x_defaults)) a->state = T_CAL_START;
        break;
    }
    case T_CAL_START:
        rc = write_value(a, 0x0087, 0x40, 1); a->since_us = now_us; a->state = T_CAL_WAIT; break;
    case T_CAL_WAIT:
        if (sensor_elapsed(now_us, a->since_us, 500000)) return init_fail(a, SENSOR_E_TIMEOUT);
        rc = data_ready(a, &ready);
        if (rc == SENSOR_OK && ready) { a->offset = 0; a->state = T_SETUP; }
        break;
    case T_SETUP:
        rc = setup_one(a);
        if (++a->offset == 13) {
            if (++a->index == a->cfg.count) { a->index = 0; a->state = T_START_ALL; }
            else { a->since_us = now_us; a->state = T_RESET_WAIT; }
        }
        break;
    case T_START_ALL:
        a->active_addr = a->cfg.address7[a->index]; rc = write_value(a, 0x0087, 0x40, 1);
        if (++a->index == a->cfg.count) {
            a->state = T_RUNNING; a->running_us = now_us;
            for (uint8_t i = 0; i < a->cfg.count; ++i) a->last_poll_us[i] = now_us;
        }
        break;
    default: return SENSOR_E_STATE;
    }
    if (rc < 0) return init_fail(a, rc);
    return a->state == T_RUNNING ? SENSOR_OK : SENSOR_PENDING;
}
