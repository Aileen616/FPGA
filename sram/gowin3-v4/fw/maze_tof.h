#ifndef MAZE_TOF_H
#define MAZE_TOF_H

#include <stdint.h>
#include <stdbool.h>
#include "maze_odometry.h"

/* 固定车体方向按顺时针排列；不对应XSHUT通道顺序。 */
enum { MT_FRONT, MT_RIGHT, MT_BACK, MT_LEFT, MT_DIRS };
enum { MT_IDLE, MT_CHECK, MT_CRUISE, MT_OBSERVE, MT_ALIGN,
       MT_DECIDE, MT_CROSS, MT_ACQUIRE, MT_GOAL,
       MT_WAIT, MT_FAULT, MT_DONE, MT_EXHAUSTED, MT_STOPPED };
enum { MT_OK, MT_CONFIG, MT_RANGE, MT_IMU, MT_GEOMETRY,
       MT_REFERENCE, MT_NODE, MT_CAPACITY, MT_TIMEOUT, MT_UNSUPPORTED,
       MT_USER_STOP, MT_CLOCK, MT_LINK };
enum { MT_VISION_UNKNOWN, MT_VISION_NEGATIVE, MT_VISION_POSITIVE };
#define MT_MAX_NODES 32
enum { MT_MAP_EXPLORE, MT_MAP_RETURN_GOAL, MT_MAP_READY, MT_MAP_GOTO, MT_MAP_NO_GOAL };

typedef struct {
    uint16_t length_mm, width_mm;       /* 完整外廓，当前用户确认为260×250毫米。 */
    uint16_t inset_mm[4];              /* 前右后左向内缩进，当前为0/60/0/60毫米。 */
    uint16_t corridor_mm;              /* 通道净宽，0表示尚未配置，不能启动。 */
    uint16_t margin_mm, error_mm;       /* 剩余间隙、位置区间误差上界，毫米。 */
    uint16_t range_error_mm;            /* 单次测距误差预算，毫米。 */
    uint16_t max_range_mm, tolerance_mm;
    uint16_t speed_mm_s, probe_mm_s;    /* 运动请求和已验证速度上界，非实测轮速。 */
    uint16_t lateral_mm_s, brake_mm;
    uint16_t fresh_ms, confirm_ms, settle_ms, wait_ms;
    uint16_t yaw_limit_md;              /* 相对初始航向最大偏差，毫度。 */
    uint16_t goal_mm;                   /* 仅前向终点候选使用的停车距离。 */
    uint8_t confirm_frames, vision_frames;
    uint8_t installation_confirmed;    /* 四探头在各侧中点，方向及内缩已核对。 */
    uint8_t dynamics_verified;         /* 实车速度/制动/测距误差标定，默认关闭。 */
    uint8_t front_goal_only;           /* 1表示已确认终点仅在初始前向尽头；0不作排除。 */
    uint8_t offline_only;              /* 离线模型许可，不产生硬件驱动许可。 */
    uint8_t mapping_enabled;           /* 功能选择：1启用完整探索与编码器建图；不是新增运动许可。 */
} mt_config_t;

typedef struct {
    uint32_t seq, stamp_ms;             /* 独立新帧序号、实际采集时间。 */
    uint16_t raw_mm;
    uint8_t valid;
} mt_range_input_t;

typedef struct {
    uint32_t now_ms;
    mt_range_input_t range[4];
    uint32_t imu_seq, imu_stamp_ms;
    int32_t yaw_md;                    /* 标定后相对初始方向，右转为正。 */
    int32_t yaw_rate_md_s;
    uint8_t imu_valid, stop;
    uint32_t vision_seq, vision_capture_ms, vision_token;
    uint8_t vision_result, vision_coverage; /* 建图模式为图像可用标记，阴性仅撤销候选；旧模式仍要求视野覆盖。 */
    mo_input_t encoder;                /* 同组已归一四轮快照；无效只标记里程，不改变ToF控制。 */
} mt_input_t;

typedef struct {
    int16_t forward_mm_s, right_mm_s;   /* 平移意图，必须由板级适配器再次约束。 */
    int32_t yaw_rate_md_s;             /* 航向纠正请求，不是角速度测量。 */
    uint32_t vision_token;
    uint8_t vision_request, state, reason;
    uint8_t hardware_permitted;        /* 离线配置始终为0；本模块不直接驱动硬件。 */
} mt_output_t;

typedef struct {
    int32_t mm, pending_mm, edge_d, edge_b;
    uint32_t seq, stamp_ms, pending_since, epoch;
    uint32_t sample_stamp, quality_stamp[8]; /* 按采集时间核验坏样本跨度，拒绝重复采集帧。 */
    int32_t raw_mm, raw_delta;         /* 上两个有效测距的趋势，毫米；不作为积分里程。 */
    uint32_t raw_dt;                   /* 上次有效相邻帧间隔，毫秒。 */
    uint16_t quality_bits;
    uint8_t seen, accepted, pending_count, quality_count;
    uint8_t armed, edge_ready;
    uint8_t raw_count;                 /* 有效趋势样本数，饱和到2；坏帧撤销趋势。 */
} mt_sensor_t;

typedef struct {
    int16_t parent;
    uint8_t entry, exits, tried, completed;
    int32_t distance_mm[4];             /* 节点停车时的局部签名，-1表示未知。 */
    int32_t x_mm,y_mm,edge_mm,return_error_mm; /* 首次里程坐标、父边轴向长度及复访误差，毫米。 */
    uint32_t revision;                 /* 最近节点变更序号，用于电脑补传，不控制运动。 */
    uint8_t metric_valid,goal,vision_checked; /* 里程有效、终点及已确认观察标记，复访不重复请求。 */
} mt_node_t;

typedef struct {
    mt_config_t cfg;
    mt_sensor_t sensor[4];
    mt_node_t nodes[MT_MAX_NODES];       /* 固定容量，无动态内存。 */
    mt_output_t output;
    uint32_t now_ms, last_tick, entered_ms, still_since, wait_since;
    uint32_t imu_seq, imu_stamp, yaw_bad_since, token, vision_seq;
    uint32_t vision_since, vision_first, anchor_epoch, cross_epoch;
    uint32_t side_epoch[2];
    int32_t yaw_md, yaw_rate_md_s, side_target[2], still_distance[4];
    int32_t align_target, edge_anchor, cross_anchor;
    int16_t current_node, return_target;
    uint16_t node_count, transitions, pending_events, return_matches;
    uint8_t excluded_mask;              /* 已按规则排除终点的方向掩码，位0前/1右/2后/3左。 */
    uint16_t excluded_deadends;         /* 按已确认场地规则排除终点的侧后死路次数。 */
    uint8_t state, reason, dir, returning, pending_mask, anchor_dir;
    uint8_t cross_dir, align_from_edge, imu_seen, imu_accepted, yaw_bad_count;
    uint8_t vision_count, vision_value, goal_checked, wait_resume, started;
    uint8_t decision_node_ready, wait_active;
    mo_t odometry;                    /* 旁路里程，与原运动控制量分离。 */
    uint32_t map_revision;
    int16_t map_events[16];            /* 增量通知环；溢出后电脑拉全图，不影响探索。 */
    int16_t goal_node,target_node,travel_target;
    int32_t depart_x_mm,depart_y_mm;
    uint8_t map_phase,map_complete,depart_entry;
} mt_t;

/* 功能：设置已确认车体安装参数和待标定候选参数。
 * 参数：c不可为空；通道宽度保持0，实车动态标定保持关闭。
 * 返回：无；不能仅调用此函数就启动。 */
void mt_defaults(mt_config_t *c);

/* 功能：检查配置并初始化导航上下文，不产生运动。
 * 参数：n/c不可为空；c中通道宽度和安全预算必须有效。
 * 返回：true配置可用于指定离线或已标定模式；false保持配置故障停止。 */
bool mt_init(mt_t *n, const mt_config_t *c);

/* 功能：显式开始一次探索任务。
 * 参数：n不可为空；dir为0至3；now_ms为单调毫秒时钟，可自然回绕。
 * 返回：true进入起步检查；false不改变停机许可，终态需重新初始化。 */
bool mt_start(mt_t *n, uint8_t dir, uint32_t now_ms);

/* 功能：非阻塞推进感知、状态机及运动意图。
 * 参数：n/in不可为空；通常每10毫秒调用；时间、序号必须来自采集链。
 * 返回：内部只读输出指针；空参数返回空指针；不访问HAL、MMIO或编码器。 */
const mt_output_t *mt_tick(mt_t *n, const mt_input_t *in);

/* 功能：人工停止并锁定本次任务。
 * 参数：n可为空，空指针无动作。
 * 返回：无；恢复需重新初始化和显式启动。 */
void mt_cancel(mt_t *n);

/* 功能：在已完成地图上请求目标节点；n不可为空，target为已有编号；返回请求是否有效，不写外设。 */
bool mt_request_target(mt_t *n,uint16_t target);
/* 功能：读取下一条节点变更；n/id/next不可为空，after为电脑已有序号；返回0无变更、1有节点、2须全图同步。 */
unsigned mt_map_event(const mt_t *n,uint32_t after,int16_t *id,uint32_t *next);

#endif
