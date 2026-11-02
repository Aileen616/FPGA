#ifndef MECANUM_OPEN_LOOP_APP_H
#define MECANUM_OPEN_LOOP_APP_H

#include "robot_open_loop.h"

/* 裸机主循环状态；所有函数由同一任务串行调用，中断只接收数据/置标志。 */
typedef struct {
    robot_open_loop_t drive;
    mecanum_command_t latest;
    uint32_t input_time_ms,last_submit_ms; /* 外部单调毫秒时基，允许uint32自然回绕。 */
    uint32_t refresh_ms,input_timeout_ms; /* 刷新间隔/操作输入有效期，1..INT32_MAX ms。 */
    bool have_input,running; /* running只表示软件正在维护请求，实际授权仍以RTL状态为准。 */
} mecanum_open_loop_app_t;

/* 功能：初始化控制服务并禁能；参数：app/bus/config不可空；refresh_ms及input_timeout_ms为正毫秒。
 * 返回：成功true；失败false。要求2*refresh_ms严格小于RTL看门狗周期，为调度留余量。
 */
bool mecanum_app_init(mecanum_open_loop_app_t *app, const robot_bus_t *bus,
                      const mecanum_config_t *config, uint32_t refresh_ms, uint32_t input_timeout_ms);

/* 功能：接收一份新鲜操作输入；参数：app/command不可空，now_ms为接收时刻。
 * 返回：接受true；非法输入false并尝试禁能。只能由真实新输入/有效按键采样刷新时间。
 */
bool mecanum_app_set_input(mecanum_open_loop_app_t *app, const mecanum_command_t *command, uint32_t now_ms);

/* 功能：显式提交并请求启动；参数：app不可空、now_ms当前毫秒；返回：请求成功true。
 * 不自动清故障，过期输入不得启动；实际运动许可在后续轮询中确认。
 */
bool mecanum_app_start(mecanum_open_loop_app_t *app, uint32_t now_ms);

/* 功能：停止服务、丢弃旧输入并请求禁能；参数：app不可空；返回：禁能请求接受true。 */
bool mecanum_app_stop(mecanum_open_loop_app_t *app);

/* 功能：显式清故障，保持停止并要求下一次输入；参数：app不可空；返回：请求接受true。 */
bool mecanum_app_clear_fault(mecanum_open_loop_app_t *app);

/* 功能：主循环周期服务，按需刷新命令；参数：app不可空，now_ms当前毫秒。
 * 返回：正常true；输入过期、RTL故障/失去授权或总线异常false并停止维护命令。
 * 本函数无HAL_Delay或忙等待；推荐每1..5ms调用，取决于实际主循环负载。
 */
bool mecanum_app_poll(mecanum_open_loop_app_t *app, uint32_t now_ms);

#endif
