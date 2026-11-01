#ifndef ROBOT_MISSION_H
#define ROBOT_MISSION_H
#include <stdint.h>
#include <stdbool.h>
/* 状态：禁能、遥控、路径点、侧移绕障、越障、回线、目标搜索、完成、故障。 */
typedef enum { R_IDLE,R_MANUAL,R_ROUTE,R_SHIFT,R_PASS,R_REJOIN,R_SEEK,R_DONE,R_FAULT } robot_mode_t;
typedef enum { R_OK,R_CANCEL,R_STALE,R_BLOCKED,R_TIMEOUT,R_DRIVE,R_GEOMETRY,R_HEADING,R_TARGET_LOST } robot_fault_t;
typedef struct { int32_t x_mm,y_mm; } robot_point_t;
typedef struct {
    uint32_t now_ms,encoder[4],range_seq[4]; /* 编码器规范轮序FL/FR/RL/RR；测距顺序前/右/左/后。 */
    uint16_t range_mm[4];bool range_valid[4],imu_valid,drive_ok;
    int32_t yaw_md,rate_mdps; /* 标定后相对航向，单位毫度和毫度/秒。 */
} robot_input_t;
typedef struct {
    robot_mode_t mode;robot_fault_t fault;
    robot_point_t points[8];uint8_t point_count,point_index;
    uint16_t offsets[4];bool configured,have_pose;
    uint16_t auto_slow_mm,auto_clear_mm; /* 自主规划使用公共策略的净距参数。 */
    int64_t x_q16,y_q16; /* 任务起点坐标，单位mm*65536；范围限制±20m。 */
    uint32_t previous_encoder[4],started_ms,renewed_ms,phase_ms,last_seq;
    uint32_t target_ms,target_seq,clear_seq;uint8_t clear_count;
    int32_t reference_md,forward,left,turn,wheel_cps[4];
    int32_t manual_f,manual_l,manual_r;uint32_t manual_ms;
    int32_t bypass_x,bypass_y,bypass_sign,bypass_f,bypass_l;
    int32_t target_error;bool target_valid,target_associated,target_seen;
} robot_mission_t;
/* 功能：初始化禁能；s非空；无返回，不启动任何外设。 */
void robot_init(robot_mission_t *s);
/* 功能：设置四个测距面到车体边缘偏移；s/off非空，mm为0..1000，必须停机；返回成功。 */
bool robot_geometry(robot_mission_t *s,const uint16_t off[4]);
/* 功能：添加路径点；s非空，x/y为起点坐标±6000mm，最多8点且停机；返回成功。 */
bool robot_point(robot_mission_t *s,int32_t x,int32_t y);
/* 功能：启动一次有界任务；s/in非空，mode为MANUAL/ROUTE/SEEK，seq全新非0；返回成功。 */
bool robot_begin(robot_mission_t *s,const robot_input_t *in,robot_mode_t mode,uint32_t seq);
/* 功能：续期遥控/视觉任务；s非空、seq严格递增、now毫秒；超250ms拒绝，返回成功。 */
bool robot_keep(robot_mission_t *s,uint32_t seq,uint32_t now);
/* 功能：提交视觉观察；s非空，序号递增，error=-1000..1000，associated仅表示已配置测距光轴落在目标框内；返回接受。 */
bool robot_target(robot_mission_t *s,uint32_t seq,uint32_t now,bool valid,int32_t error,bool associated);
/* 功能：执行10ms控制步；s/in非空；无返回，输出仅为wheel_cps，失效清零且不自动重启。 */
void robot_step(robot_mission_t *s,const robot_input_t *in);
/* 功能：锁定结束原因并清四轮目标；s非空，reason为枚举；无返回。 */
void robot_stop(robot_mission_t *s,robot_fault_t reason);
/* 功能：查询活动阶段；s非空；返回是否需执行控制。 */
bool robot_active(const robot_mission_t *s);
/* 功能：整数atan2；x/y范围±2^20且不同时为0；返回±180000毫度，零向量返回0。 */
int32_t robot_atan2(int32_t y,int32_t x);
/* 功能：整数平方根；n为uint32；返回向下取整非负根。 */
uint32_t robot_sqrt(uint32_t n);
#endif
