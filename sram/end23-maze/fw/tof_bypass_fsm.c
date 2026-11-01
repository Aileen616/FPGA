#include "tof_bypass_fsm.h"
#include <string.h>

/* 功能：初始化任务；s不可空；无返回。 */
void bypass_init(bypass_fsm_t *s) { memset(s, 0, sizeof(*s)); }
/* 功能：查询活动状态；s不可空；任务尚未终结时true。 */
bool bypass_active(const bypass_fsm_t *s) { return s->phase > B_IDLE && s->phase < B_DONE; }
/* 功能：锁存停止原因；s不可空、reason有效；无返回。 */
void bypass_abort(bypass_fsm_t *s, bypass_reason_t reason)
{ s->phase = B_FAULT; s->reason = reason; }
/* 功能：切换阶段并重置静止验证；s/in不可空，phase为合法阶段；无返回。 */
static void enter(bypass_fsm_t *s, bypass_phase_t phase, const bypass_input_t *in)
{
    s->phase = phase; s->phase_ms = s->quiet_ms = in->now_ms;
    s->clear_samples = 0; s->last_front_sequence = in->front_sequence;
    if (phase == B_APPROACH || phase == B_SHIFT_RIGHT || phase == B_PASS) ++s->generation;
}
/* 功能：核验几何和标定；c不可空；范围和绕行余量合法时true。 */
static bool configuration_valid(const bypass_config_t *c)
{
    return c->geometry_confirmed && c->lateral_scale_checked &&
        c->body_length_mm >= 100 && c->body_length_mm <= 1000 &&
        c->body_width_mm >= 100 && c->body_width_mm <= 1000 &&
        c->obstacle_width_mm > 0 && c->obstacle_width_mm <= 1000 &&
        c->obstacle_depth_mm > 0 && c->obstacle_depth_mm <= 1000 &&
        c->clearance_mm >= 300 && c->clearance_mm <= 600 && c->shift_mm <= 1500 &&
        2U * c->shift_mm >= (uint32_t)c->body_width_mm + c->obstacle_width_mm + 2U*c->clearance_mm;
}
/* 功能：显式启动一次任务；参数均不可空；拒绝错误配置、运动中重启及无效传感器。 */
bool bypass_begin(bypass_fsm_t *s, const bypass_config_t *c, const bypass_input_t *in)
{
    if (bypass_active(s) || !configuration_valid(c) || !in->sensors_fresh ||
        !in->lease_valid || in->wheels_moving || in->front_mm <= 800 ||
        in->right_mm <= c->shift_mm + c->clearance_mm || in->left_mm <= 300) return false;
    s->config = *c; s->reason = B_OK; s->started_ms = in->now_ms;
    s->pass_mm = s->approach_stop_mm = 0;
    /* 段号不清零，避免上一轮迟到的到距消息被新任务消费。 */
    if (s->generation > UINT32_MAX - 3U) return false;
    enter(s, B_APPROACH, in); return true;
}
/* 功能：验证连续300ms轮子静止；s/in不可空；超过2秒仍动则故障，静止true。 */
static bool settled(bypass_fsm_t *s, const bypass_input_t *in)
{
    if (in->wheels_moving) s->quiet_ms = in->now_ms;
    if ((uint32_t)(in->now_ms - s->phase_ms) >= 2000U) {
        bypass_abort(s, B_NOT_SETTLED); return false;
    }
    return (uint32_t)(in->now_ms - s->quiet_ms) >= 300U;
}
/* 功能：执行阶段状态机；s/in不可空；不直接访问寄存器或发送电机指令。 */
void bypass_step(bypass_fsm_t *s, const bypass_input_t *in)
{
    if (!bypass_active(s)) return;
    if (!in->lease_valid) { bypass_abort(s, B_LINK); return; }
    if (!in->sensors_fresh) { bypass_abort(s, B_SENSOR); return; }
    if ((uint32_t)(in->now_ms - s->started_ms) >= 80000U) { bypass_abort(s, B_TIMEOUT); return; }
    if (in->front_mm <= 300 || in->right_mm <= 300 || in->left_mm <= 300) {
        bypass_abort(s, B_BLOCKED); return;
    }
    if (s->phase == B_APPROACH || s->phase == B_SHIFT_RIGHT || s->phase == B_PASS) {
        if (in->result == B_SEG_RUNNING || in->result_generation != s->generation) return;
        if (in->result == B_SEG_FAILED) { bypass_abort(s, B_SEGMENT_FAULT); return; }
        if (s->phase == B_APPROACH) {
            if (in->result == B_SEG_DISTANCE) enter(s, B_WAIT_END, in);
            else { s->approach_stop_mm = in->front_mm; enter(s, B_WAIT_FRONT, in); }
        } else if (in->result == B_SEG_OBSTACLE) {
            /* 侧方或绕行前方再遇障时结束本次，不递归尝试未知路线。 */
            bypass_abort(s, B_BLOCKED);
        } else enter(s, s->phase == B_SHIFT_RIGHT ? B_WAIT_SHIFT : B_WAIT_END, in);
        return;
    }
    if (s->phase == B_WAIT_SHIFT && in->front_sequence != s->last_front_sequence) {
        s->last_front_sequence = in->front_sequence;
        if (in->front_mm > 800) { if (s->clear_samples < 3) ++s->clear_samples; }
        else s->clear_samples = 0;
    }
    if (!settled(s, in)) return;
    if (s->phase == B_WAIT_FRONT) {
        if (in->right_mm <= s->config.shift_mm + s->config.clearance_mm) {
            bypass_abort(s, B_BLOCKED); return;
        }
        /* 经过纸盒远端后，整个车长再加净距；不按固定时间宣称绕过。 */
        uint32_t pass = (uint32_t)s->approach_stop_mm + s->config.obstacle_depth_mm +
                        s->config.body_length_mm + s->config.clearance_mm;
        if (pass > 3000U) { bypass_abort(s, B_BAD_CONFIG); return; }
        s->pass_mm = (uint16_t)pass; enter(s, B_SHIFT_RIGHT, in);
    } else if (s->phase == B_WAIT_SHIFT) {
        if (s->clear_samples >= 3) enter(s, B_PASS, in);
    } else if (s->phase == B_WAIT_END) enter(s, B_DONE, in);
}
