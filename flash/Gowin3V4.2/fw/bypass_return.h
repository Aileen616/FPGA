#ifndef BYPASS_RETURN_H
#define BYPASS_RETURN_H
#include <stdbool.h>
#include <stdint.h>
/* 固定纸盒右绕再回线；WAIT阶段只能输出零，段号防止迟到结果串段。 */
typedef enum { B_IDLE,B_APPROACH,B_WAIT_FRONT,B_SHIFT_RIGHT,B_WAIT_SHIFT,
    B_PASS,B_WAIT_PASS,B_RETURN_LEFT,B_WAIT_END,B_DONE,B_FAULT } bypass_phase_t;
typedef enum { B_OK,B_BAD_CONFIG,B_SENSOR,B_LINK,B_BLOCKED,B_SEGMENT_FAULT,
    B_NOT_SETTLED,B_TIMEOUT,B_CANCEL,B_IMU } bypass_reason_t;
typedef enum { B_SEG_RUNNING,B_SEG_DISTANCE,B_SEG_OBSTACLE,B_SEG_FAILED } bypass_result_t;
typedef struct {
    uint16_t body_length_mm,body_width_mm,obstacle_width_mm,obstacle_depth_mm;
    uint16_t shift_mm,clearance_mm;
    bool geometry_confirmed,lateral_scale_checked;
} bypass_config_t;
typedef struct {
    uint32_t now_ms,front_sequence,left_sequence;
    uint16_t front_mm,right_mm,left_mm; /* 车体边缘净距，毫米。 */
    bool sensors_fresh,lease_valid,wheels_moving,imu_ready;
    uint32_t result_generation;
    bypass_result_t result;
} bypass_input_t;
typedef struct {
    bypass_config_t config;
    bypass_phase_t phase;
    bypass_reason_t reason;
    uint32_t started_ms,phase_ms,quiet_ms,generation,last_front_sequence,last_left_sequence;
    uint16_t pass_mm,approach_stop_mm;
    uint8_t clear_samples;
    bool quiet_confirmed,returned;
} bypass_fsm_t;
/* 功能：初始化禁能状态；s不可空；无返回。 */
void bypass_init(bypass_fsm_t *s);
/* 功能：显式启动单次右绕回线；s/c/in不可空；满足几何、静止和传感器条件返回true。 */
bool bypass_begin(bypass_fsm_t *s,const bypass_config_t *c,const bypass_input_t *in);
/* 功能：每10ms推进状态和保护；s/in不可空；无返回，段请求由phase和generation表示。 */
void bypass_step(bypass_fsm_t *s,const bypass_input_t *in);
/* 功能：锁存故障；s不可空，reason非B_OK；无返回，不自动恢复。 */
void bypass_abort(bypass_fsm_t *s,bypass_reason_t reason);
/* 功能：判断活动阶段；s不可空；活动返回true。 */
bool bypass_active(const bypass_fsm_t *s);
/* 功能：混合平移和航向；direction为0前/1右/2左，effort为0..19660，ratio为-100..100千分比。
 * out不可空，按板通道RR/FR/RL/FL输出；整体等比例限幅，合法返回true，非法不写输出。
 */
bool bypass_mix(unsigned direction,int32_t effort,int32_t ratio,int32_t out[4]);
#endif
