#ifndef TOF_BYPASS_FSM_H
#define TOF_BYPASS_FSM_H
#include <stdbool.h>
#include <stdint.h>
/* 阶段请求交给板端运动执行器；WAIT阶段必须保持输出撤销。 */
typedef enum { B_IDLE, B_APPROACH, B_WAIT_FRONT, B_SHIFT_RIGHT,
    B_WAIT_SHIFT, B_PASS, B_WAIT_END, B_DONE, B_FAULT } bypass_phase_t;
typedef enum { B_OK, B_BAD_CONFIG, B_SENSOR, B_LINK, B_BLOCKED,
    B_SEGMENT_FAULT, B_NOT_SETTLED, B_TIMEOUT, B_CANCEL } bypass_reason_t;
/* 执行器结果：无结果、编码器到距、障碍停车、其他故障。 */
typedef enum { B_SEG_RUNNING, B_SEG_DISTANCE, B_SEG_OBSTACLE, B_SEG_FAILED } bypass_result_t;
typedef struct {
    uint16_t body_length_mm, body_width_mm, obstacle_width_mm, obstacle_depth_mm;
    uint16_t shift_mm, clearance_mm;
    bool geometry_confirmed, lateral_scale_checked;
} bypass_config_t;
typedef struct {
    uint32_t now_ms, front_sequence;
    uint16_t front_mm, right_mm, left_mm; /* 已扣除安装缩进的净距，单位毫米。 */
    bool sensors_fresh, lease_valid, wheels_moving;
    uint32_t result_generation; /* 只能提交当前段号的结果，旧段结果拒绝消费。 */
    bypass_result_t result;
} bypass_input_t;
typedef struct {
    bypass_config_t config;
    bypass_phase_t phase;
    bypass_reason_t reason;
    uint32_t started_ms, phase_ms, quiet_ms, generation, last_front_sequence;
    uint16_t pass_mm, approach_stop_mm;
    uint8_t clear_samples;
} bypass_fsm_t;
/* 功能：初始化为禁能；s不可空；无返回。 */
void bypass_init(bypass_fsm_t *s);
/* 功能：启动单次向右绕行；s/c/in不可空，配置须已确认且横移尺度已作实测核对；成功true。 */
bool bypass_begin(bypass_fsm_t *s, const bypass_config_t *c, const bypass_input_t *in);
/* 功能：推进阶段并检查保护；s/in不可空，建议10ms调用；无返回，运动请求由phase/generation表示。 */
void bypass_step(bypass_fsm_t *s, const bypass_input_t *in);
/* 功能：锁存故障并停止请求；s不可空，reason为非B_OK；无返回，不自动复位。 */
void bypass_abort(bypass_fsm_t *s, bypass_reason_t reason);
/* 功能：判断是否仍执行任务；s不可空；活动阶段返回true。 */
bool bypass_active(const bypass_fsm_t *s);
#endif
