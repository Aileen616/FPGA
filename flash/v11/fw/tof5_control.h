#ifndef TOF5_CONTROL_H
#define TOF5_CONTROL_H
#include <stdbool.h>
#include <stdint.h>
/* 状态：未启动、正常前进、已锁定减速、已撤销输出；停车不表示实体已静止。 */
typedef enum { T5_IDLE, T5_RUN, T5_DECEL, T5_STOP } tof5_state_t;
/* 停机原因与到距完成分开；故障和障碍不算完成5米。 */
typedef enum { T5_NONE, T5_DISTANCE, T5_OBSTACLE, T5_SENSOR,
    T5_LINK, T5_TIMEOUT, T5_ENCODER, T5_CANCEL, T5_DRIVE } tof5_reason_t;
typedef struct {
    uint32_t now_ms, age_us;           /* 毫秒时钟及测距年龄，支持uint32回绕。 */
    int32_t position[4];              /* 协议通道1..4原始编码器计数。 */
    uint16_t distance_mm;             /* X2测距面到障碍距离，单位mm。 */
    bool valid, drive_ok;             /* 有效新鲜测距、驱动允许；false立即撤销输出。 */
} tof5_input_t;
typedef struct {
    tof5_state_t state;
    tof5_reason_t reason;
    uint32_t started_ms, renewed_ms, decel_ms, last_sequence;
    uint32_t target_counts; /* 本段目标1..31614计数。 */
    uint8_t raw_invert; /* 前进12，右移10。 */
    uint32_t baseline[4];             /* 起点原始计数，无符号减法处理回绕。 */
    int32_t effort, progress;          /* 四轮统一PWM指令0..19660、最快轮行程counts。 */
    uint16_t front_offset_mm;         /* 测距面落后车头前缘0..1000mm。 */
    bool offset_known;                /* 安装尺寸未填写时不允许起步。 */
} tof5_control_t;
#define TOF5_WIDTH_MM 255U
#define TOF5_DISTANCE_COUNTS 31614U /* ceil(6m * 5268.945counts/m)，有效系数继承已有试验。 */
#define TOF5_BASE_EFFORT 19660        /* 19660/32767，约25%基础占空比；不是速度。 */
#define TOF5_SLOW_MM 800U          /* 车头到障碍测试减速阈值；未完成实车制动验证。 */
#define TOF5_STOP_MM 300U          /* 车头到障碍测试强制撤销输出阈值。 */
/* 功能：初始化禁能状态；s不可空；无返回值。 */
void tof5_init(tof5_control_t *s);
/* 功能：设置测距面相对车头偏移；s不可空、mm为0..1000，运动中拒绝；返回成功与否。 */
bool tof5_offset(tof5_control_t *s, uint32_t mm);
/* 功能：以全新序号启动一次任务；s/in不可空、seq递增且非0；通过全部前置条件返回true。 */
bool tof5_begin(tof5_control_t *s, const tof5_input_t *in, uint32_t seq);
/* 功能：只接受未过期任务的新续期；s不可空、seq递增非0、now_ms当前时间；成功true。 */
bool tof5_keep(tof5_control_t *s, uint32_t seq, uint32_t now_ms);
/* 功能：周期推进本地保护与减速；s/in不可空、建议每10ms调用；无返回，目标写入s。 */
void tof5_step(tof5_control_t *s, const tof5_input_t *in);
/* 功能：锁定停止原因并清目标；s不可空、reason为非NONE；无返回，不能自动恢复。 */
void tof5_stop(tof5_control_t *s, tof5_reason_t reason);
/* 功能：查询运动阶段；s不可空；RUN/DECEL返回true。 */
bool tof5_active(const tof5_control_t *s);
#endif
