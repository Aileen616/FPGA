#include "robot_sensors.h"
#include <string.h>

/* 功能/参数/返回：见robot_sensors.h；配置无效时不启用任何模块。 */
sensor_result_t robot_sensors_start(robot_sensors_t *r, const sensor_bus_t *bus,
                                    const robot_sensor_config_t *cfg, uint32_t now_us)
{
    robot_sensor_config_t c;
    sensor_bus_t b;
    sensor_result_t rc;
    if (!r || !bus || !cfg || !bus->spi || cfg->tof.count != TOF_MAX_COUNT ||
        cfg->stop_mm == 0 || cfg->slow_mm <= cfg->stop_mm || cfg->slow_mm > 4000 ||
        cfg->imu_fresh_us < 10000 || cfg->imu_fresh_us > 1000000) return SENSOR_E_ARG;
    c = *cfg; b = *bus; memset(r, 0, sizeof(*r)); r->cfg = c; r->guard.sensor_fault = true;
    rc = vl53l1x_array_start(&r->tof, &b, &c.tof, now_us);
    if (rc < 0) { r->tof_result = rc; return rc; }
    rc = bmi088_start(&r->imu, &b, now_us, c.imu_fresh_us);
    r->imu_result = rc; r->tof_result = SENSOR_PENDING;
    return rc;
}
/* 功能/参数/返回：见robot_sensors.h；减速区采用线性比例，最终速度需受轮控限幅。 */
void robot_sensors_guard(robot_sensors_t *r)
{
    sensor_guard_t *g;
    uint16_t nearest = UINT16_MAX;
    if (!r) return;
    g = &r->guard; memset(g, 0, sizeof(*g));
    g->sensor_fault = !bmi088_ready(&r->imu) || !vl53l1x_array_ready(&r->tof) ||
                      r->imu.sample.validity != SENSOR_VALID;
    for (unsigned i = 0; i < TOF_MAX_COUNT; ++i) {
        const tof_sample_t *s = &r->tof.sample[i];
        if (s->validity != SENSOR_VALID) g->sensor_fault = true;
        else if (s->distance_mm < nearest) nearest = s->distance_mm;
    }
    g->nearest_mm = nearest == UINT16_MAX ? 0 : nearest;
    g->obstacle_stop = nearest <= r->cfg.stop_mm;
    g->slowdown = nearest < r->cfg.slow_mm;
    if (g->sensor_fault || g->obstacle_stop) g->speed_scale_q15 = 0;
    else if (g->slowdown) g->speed_scale_q15 = (uint16_t)(
        (uint32_t)(nearest - r->cfg.stop_mm) * 32767u / (r->cfg.slow_mm - r->cfg.stop_mm));
    else g->speed_scale_q15 = 32767;
}
/* 功能/参数/返回：见robot_sensors.h；没有就绪事件也检查过期，避免软件保持旧VALID。 */
void robot_sensors_poll(robot_sensors_t *r, const bmi088_events_t *events, uint32_t now_us)
{
    static const bmi088_events_t no_event = {0, 0, 0};
    if (!r) return;
    if (!bmi088_ready(&r->imu)) r->imu_result = bmi088_init_step(&r->imu, now_us);
    else r->imu_result = bmi088_read(&r->imu, events ? events : &no_event, now_us);
    bmi088_health(&r->imu, now_us);
    r->tof_result = vl53l1x_array_step(&r->tof, now_us);
    robot_sensors_guard(r);
}
