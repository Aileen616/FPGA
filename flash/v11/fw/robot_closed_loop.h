#ifndef ROBOT_CLOSED_LOOP_H
#define ROBOT_CLOSED_LOOP_H
#include <stdbool.h>
#include <stdint.h>

#define ROBOT_CL_ID UINT32_C(0x42580202)
#define ROBOT_CL_COMMIT UINT32_C(0x434d4402)
/* 固定轮序；每轮速度单位counts/s，位置单位counts。 */
typedef enum { ROBOT_CL_FL=0, ROBOT_CL_FR=1, ROBOT_CL_RL=2, ROBOT_CL_RR=3 } robot_cl_wheel_t;
enum {
    ROBOT_CL_CTRL=0x008, ROBOT_CL_STATUS=0x00c, ROBOT_CL_SEQ=0x010,
    ROBOT_CL_COMMIT_REG=0x014, ROBOT_CL_LIMIT=0x018, ROBOT_CL_TARGET=0x020,
    ROBOT_CL_SNAP_TARGET=0x030, ROBOT_CL_SPEED=0x040, ROBOT_CL_POSITION=0x050,
    ROBOT_CL_SNAP_SEQ=0x060, ROBOT_CL_AGE=0x064, ROBOT_CL_FAULT=0x068,
    ROBOT_CL_DIAG=0x070, ROBOT_CL_SNAP_STATUS=0x074, ROBOT_CL_CLOCK=0x078,
    ROBOT_CL_WINDOW=0x07c, ROBOT_CL_GAINS=0x080, ROBOT_CL_ILLEGAL=0x0a0,
    ROBOT_CL_NO_EDGE=0x0b0, ROBOT_CL_MIN_TARGET=0x0b4, ROBOT_CL_MIN_EFFORT=0x0b8,
    ROBOT_CL_ILLEGAL_LIMIT=0x0bc, ROBOT_CL_EFFORT_LIMIT=0x0c0,
    ROBOT_CL_TIMEOUT=0x0d0, ROBOT_CL_SAMPLE_SEQ=0x0d4, ROBOT_CL_EFFORT=0x0e0
};
/* 功能：完成一次有界的32位读事务；ctx可空，offset是字节偏移，value不可空。
 * 返回：完成且总线无错true；超时或总线错误false。回调负责实际SDK和内存屏障。
 */
typedef bool (*robot_cl_read_fn)(void *ctx, uint16_t offset, uint32_t *value);
/* 功能：完成一次有界的32位写事务；ctx可空，offset为字节偏移，value为全字数据。
 * 返回：写已完成true，失败或结果不确定false。严禁将偏移直接当物理地址。
 */
typedef bool (*robot_cl_write_fn)(void *ctx, uint16_t offset, uint32_t value);
typedef struct { void *ctx; robot_cl_read_fn read32; robot_cl_write_fn write32; } robot_cl_bus_t;
typedef struct {
    uint32_t kp_q16[4],ki_tick_q16[4]; /* 非负Q16.16且最高位为0；Ki包含一次控制周期。 */
    uint32_t no_edge_windows; /* 1..UINT32_MAX个测速窗口；必须按最低有效速度和起动时间配置。 */
    uint32_t min_target_cps; /* 1..核心目标上限：无边沿检测的速度门槛，counts/s。 */
    uint16_t min_effort; /* 1..32767：无边沿检测的驱动力门槛。 */
    uint32_t illegal_limit; /* 单窗口非法跳变阈值，0仅记录不自动停车。 */
    uint16_t effort_limit[4]; /* 各轮0..32767，0独立禁能；活动轮上限不得低于min_effort；不是电流限制。 */
} robot_cl_config_t;
typedef struct {
    robot_cl_bus_t bus;
    uint32_t next_sequence,clock_hz,window_cycles,watchdog_cycles,max_target_cps;
    bool initialized,configured,fresh_for_arm;
} robot_cl_t;
typedef struct {
    int32_t target_cps[4],measured_cps[4],position[4];
    int32_t effort[4]; /* 同拍PI输出，-32767..32767驱动力，不是实测电流或转速。 */
    uint32_t illegal[4],sequence,sample_sequence,status,diagnostics;
    /* diagnostics位0..3量化，4..7溢出，8..11编码器同步ready，
       16..23每轮两位故障原因，24为无边沿保护启用；ready不代表传感器接线健康。 */
} robot_cl_snapshot_t;

/* 功能：验证四轮闭环接口身份、读取时基并请求禁能。
 * 参数：client、bus及两回调不可空；同一核心只允许一个串行命令源。
 * 返回：初始化完成true；任一步失败false。不选择CPU、不自动使能。
 */
bool robot_cl_init(robot_cl_t *client, const robot_cl_bus_t *bus);
/* 功能：在禁能状态下设置四轮PI和编码器保护门槛。
 * 参数：client已初始化；config不可空，至少一个活动轮；活动轮Kp/Ki至少一个非零，范围见结构体。
 * 返回：全部参数成功写入true；失败false并尽力禁能，必须重新完整配置。
 */
bool robot_cl_configure(robot_cl_t *client, const robot_cl_config_t *config);
/* 功能：按SEQ、四轮shadow、COMMIT顺序原子提交新目标，也用于新鲜命令的周期刷新。
 * 参数：client已配置，targets含FL/FR/RL/RR四项，非空且绝对值不超过核心限值。
 * 返回：提交并核对序号成功true；失败false并尽力禁能，禁止应用在命令源过期后盲目刷新。
 */
bool robot_cl_submit(robot_cl_t *client, const int32_t targets[4]);
/* 功能：显式请求使能；参数：client已配置且提交过新目标，不可空。
 * 返回：请求接受true，前置状态或事务失败false；应用必须读取状态确认实际armed。
 */
bool robot_cl_request_arm(robot_cl_t *client);
/* 功能：请求禁能并清除软件重启资格；参数：client已初始化，不可空。
 * 返回：写成功true，否则false；通信失效仍由RTL命令超时独立禁能。
 */
bool robot_cl_disarm(robot_cl_t *client);
/* 功能：清故障并清除重启资格；参数：client已初始化且真实故障已排除。
 * 返回：请求成功true，否则false；仍须完整提交新命令后显式arm。
 */
bool robot_cl_clear_fault(robot_cl_t *client);
/* 功能：读取实时状态；参数：client和status不可空。
 * 返回：读取成功true，否则false且非空status置0，不能把失败时的0当有效状态。
 */
bool robot_cl_status(robot_cl_t *client, uint32_t *status);
/* 功能：冻结并读取完整四轮反馈；参数：client、out不可空，不允许并发快照或命令访问。
 * 返回：读取成功且快照编号一致true；失败false，out清零且不能当有效反馈。
 */
bool robot_cl_snapshot(robot_cl_t *client, robot_cl_snapshot_t *out);
#endif
