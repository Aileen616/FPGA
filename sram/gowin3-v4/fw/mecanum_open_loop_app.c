#include "mecanum_open_loop_app.h"
#include <stddef.h>
#include <string.h>

/* 无全局状态和外部外设句柄；now_ms由最终平台时基提供，所有总线回调必须有限超时。 */

/* 功能：检查输入是否仍有效；参数：app不可空，now_ms当前毫秒；返回：新鲜true，否则false。 */
static bool fresh_input(const mecanum_open_loop_app_t *app, uint32_t now_ms)
{
    return app->have_input && (uint32_t)(now_ms-app->input_time_ms)<app->input_timeout_ms;
}

/* 功能：初始化裸机服务；参数：三个指针不可空，间隔/超时为1..INT32_MAX ms。
 * 返回：成功true；失败false，已识别到目标核心时保持禁能。
 */
bool mecanum_app_init(mecanum_open_loop_app_t *app, const robot_bus_t *bus,
                      const mecanum_config_t *config, uint32_t refresh_ms, uint32_t input_timeout_ms)
{
    robot_bus_t saved_bus;
    mecanum_config_t saved_config;
    if(app==NULL || bus==NULL || config==NULL) return false;
    saved_bus=*bus;saved_config=*config;
    memset(app,0,sizeof(*app));
    if(!robot_ol_init(&app->drive,&saved_bus,&saved_config)) return false;
    if(refresh_ms==0U || refresh_ms>INT32_MAX || input_timeout_ms==0U || input_timeout_ms>INT32_MAX ||
       (uint64_t)refresh_ms*2U*app->drive.clock_hz >= (uint64_t)app->drive.watchdog_cycles*1000U) {
        app->drive.initialized=false;
        return false;
    }
    app->refresh_ms=refresh_ms;app->input_timeout_ms=input_timeout_ms;
    return true;
}

/* 功能：接收新鲜操作量；参数：app/command不可空，now_ms为输入接收时刻。
 * 返回：成功true；非法操作量false并停止，不能用旧缓存重复刷新输入时间戳。
 */
bool mecanum_app_set_input(mecanum_open_loop_app_t *app, const mecanum_command_t *command, uint32_t now_ms)
{
    mecanum_output_t checked;
    if(app==NULL || !app->drive.initialized) return false;
    if(!mecanum_mix(&app->drive.config,command,&checked)) {
        (void)mecanum_app_stop(app);return false;
    }
    app->latest=*command;app->input_time_ms=now_ms;app->have_input=true;
    return true;
}

/* 功能：用当前新鲜命令显式启动；参数：app不可空，now_ms当前时间；返回：请求成功true。 */
bool mecanum_app_start(mecanum_open_loop_app_t *app, uint32_t now_ms)
{
    robot_open_loop_snapshot_t state;
    if(app==NULL || !app->drive.initialized) return false;
    if(!fresh_input(app,now_ms) || !robot_ol_snapshot(&app->drive,&state) || (state.status&2U)!=0U ||
       !robot_ol_submit(&app->drive,&app->latest) || !robot_ol_request_arm(&app->drive)) {
        (void)mecanum_app_stop(app);return false;
    }
    app->last_submit_ms=now_ms;app->running=true;
    return true;
}

/* 功能：停止服务并丢弃输入；参数：app不可空；返回：禁能事务成功true，否则false。 */
bool mecanum_app_stop(mecanum_open_loop_app_t *app)
{
    if(app==NULL) return false;
    app->running=false;app->have_input=false;
    return robot_ol_disarm(&app->drive);
}

/* 功能：请求清故障且丢弃旧输入；参数：app不可空；返回：清故障事务成功true，否则false。 */
bool mecanum_app_clear_fault(mecanum_open_loop_app_t *app)
{
    if(app==NULL) return false;
    app->running=false;app->have_input=false;
    return robot_ol_clear_fault(&app->drive);
}

/* 功能：周期维护命令并监视输入/RTL状态；参数：app不可空、now_ms当前毫秒。
 * 返回：正常true；异常false并禁能。读取失败后不继续用心跳维持旧运动命令。
 */
bool mecanum_app_poll(mecanum_open_loop_app_t *app, uint32_t now_ms)
{
    robot_open_loop_snapshot_t state;
    if(app==NULL || !app->drive.initialized) return false;
    if(!app->running) return true;
    if(!fresh_input(app,now_ms)) {(void)mecanum_app_stop(app);return false;}
    if((uint32_t)(now_ms-app->last_submit_ms)<app->refresh_ms) return true;
    if(!robot_ol_snapshot(&app->drive,&state) || (state.status&15U)!=9U ||
       !robot_ol_submit(&app->drive,&app->latest)) {
        (void)mecanum_app_stop(app);return false;
    }
    app->last_submit_ms=now_ms;
    return true;
}
