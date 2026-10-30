#ifndef AUTONOMY_POLICY_H
#define AUTONOMY_POLICY_H
#include <stdbool.h>
#include <stdint.h>

/* 方向位与规范ToF顺序一致：前X2、右X3、左X4、后X1；可组合斜向。 */
enum { AP_FRONT=1, AP_RIGHT=2, AP_LEFT=4, AP_REAR=8, AP_ALL=15 };
typedef enum { AP_IDLE, AP_RUN, AP_WAIT, AP_DONE, AP_FAULT } ap_state_t;
typedef enum {
    AP_OK, AP_CONFIG, AP_COMMAND, AP_RANGE, AP_IMU, AP_ENCODER,
    AP_DRIVE, AP_CONTROL, AP_ESTOP, AP_NEAR, AP_OVERSPEED,
    AP_BLOCKED, AP_TARGET, AP_TIMEOUT, AP_DIRECTION, AP_CANCEL
} ap_reason_t;

typedef struct {
    uint16_t body_length_mm, body_width_mm; /* 100..1000mm，用户实测280/250。 */
    uint16_t start_mm, slow_mm, emergency_mm, hysteresis_mm; /* 净距，默认200/200/80/30mm。 */
    uint16_t offset_mm[4]; /* 探头测距面至车体外沿的内缩量，0..1000mm。 */
    uint8_t offset_known_mask; /* 已核实安装偏移的方向位；未知不得参与运动。 */
    uint16_t max_speed_mm_s, brake_decel_mm_s2, latency_ms, error_margin_mm;
    /* 制动能力必须由外部验证；默认false/减速度0，测试模型不能冒充实测。 */
    bool brake_model_verified;
    bool encoder_glitch_policy_verified; /* 只有真实分类与频率界限已验证才允许短暂容错。 */
    uint8_t encoder_glitch_budget; /* 每窗口0..10次；默认0，不凭空放宽编码器保护。 */
    uint16_t encoder_window_ms; /* 近期异常窗口10..1000ms，默认200ms。 */
    uint16_t range_age_ms, imu_age_ms, control_gap_ms, recover_ms, wait_ms;
    uint32_t task_ms; /* 单次任务总时限；等待不会重置总计时。 */
    uint16_t segment_mm; /* 分段规划最大长度，默认100mm；不代表已执行位移。 */
} ap_config_t;

typedef struct {
    uint32_t sequence, timestamp_ms;
    uint16_t raw_mm;
    bool valid; /* 仅质量通过的真实新帧为true；无效帧不刷新最后有效样本。 */
} ap_range_t;

typedef struct {
    uint32_t now_ms;
    ap_range_t range[4];
    uint32_t imu_sequence, imu_timestamp_ms;
    bool imu_valid, calibrated, axis_confirmed, imu_hard_fault;
    bool driver_ok, encoder_reliable, encoder_event, encoder_event_classified_glitch;
    /* 只有独立证据已确认的短毛刺可标classified；未知非法AB仍属于不可靠反馈。 */
    bool estop, cancel, complete, planner_wait;
    bool target_required, target_valid;
    uint32_t target_timestamp_ms;
    bool external_linked; /* 自主模式不依赖外部链路；人工停止命令另行优先处理。 */
    bool rotation, rotation_envelope_verified; /* 四个点测距不能自行证明旋转包络安全。 */
    uint8_t direction_mask; /* 非旋转必须为单方向或相邻两方向；禁止相反方向同时提交。 */
    uint16_t speed_mm_s; /* 车体实测速度大小，0..2000；不是PWM百分比。 */
} ap_input_t;

typedef struct {
    ap_config_t config;
    ap_state_t state;
    ap_reason_t reason;
    uint32_t command_sequence, started_ms, last_step_ms, wait_since_ms, clear_since_ms;
    uint32_t imu_sequence, imu_timestamp_ms, resume_sequence[4];
    uint32_t encoder_window_start_ms;
    ap_range_t accepted[4];
    uint8_t task, direction_mask, clear_frames;
    uint8_t encoder_glitches; /* 近期窗口中的新异常数量；历史累计数不用于永久拒绝启动。 */
    bool rotation, imu_received, config_valid;
    uint16_t speed_limit_mm_s, stop_mm; /* 授权速度上限；零表示必须撤销运动请求。 */
} ap_policy_t;

/* 功能：生成默认配置；c不可空；无返回，制动和安装偏移默认未确认。 */
void ap_defaults(ap_config_t *c);
/* 功能：初始化离线策略；p/c不可空；返回配置数值是否有效，不启动任务。 */
bool ap_init(ap_policy_t *p, const ap_config_t *c);
/* 功能：计算速度相关停车净距；c不可空、速度0..2000mm/s；无模型时返回UINT16_MAX。 */
uint16_t ap_stop_distance(const ap_config_t *c, uint16_t speed_mm_s);
/* 功能：显式启动任务3..6；p/in不可空、seq非零且递增；成功true，否则保持零授权。 */
bool ap_begin(ap_policy_t *p, uint8_t task, uint32_t seq, const ap_input_t *in);
/* 功能：推进本地策略；p/in不可空，建议每10ms调用；无返回，不访问外设或输出PWM。 */
void ap_step(ap_policy_t *p, const ap_input_t *in);
/* 功能：仅发布新有效传感器帧；p/in不可空；无返回，不续租、不改变任务状态。 */
void ap_refresh(ap_policy_t *p,const ap_input_t *in);
/* 功能：按净距限制下一平移段；p不可空、desired/clearance单位mm；返回0表示不准规划前进。 */
uint16_t ap_segment_limit(const ap_policy_t *p, uint16_t desired_mm, uint16_t clearance_mm);
#endif
