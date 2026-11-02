#include "robot_closed_loop.h"
#include <stddef.h>
#include <string.h>
#include <limits.h>
/* 无外部全局变量、硬编码物理地址或HAL句柄；实际处理器通过bus回调接入。 */

/* 功能：判断实例可用；参数：c可空；返回：已初始化且两回调有效true，否则false。 */
static bool usable(const robot_cl_t *c)
{
    return c != NULL && c->initialized && c->bus.read32 != NULL && c->bus.write32 != NULL;
}
/* 功能：发生错误时尽力禁能；参数：c为已绑定回调实例，可空；返回：固定false。 */
static bool fail(robot_cl_t *c)
{
    if (c != NULL) {
        c->fresh_for_arm = false;
        if (c->initialized && c->bus.write32 != NULL)
            (void)c->bus.write32(c->bus.ctx, ROBOT_CL_CTRL, 2U);
    }
    return false;
}
/* 功能：解码二补码signed32；参数：v为32位寄存器值；返回：对应-2^31..2^31-1整数。 */
static int32_t signed_value(uint32_t v)
{
    return v <= INT32_MAX ? (int32_t)v : -INT32_C(1) - (int32_t)(UINT32_MAX-v);
}
/* 功能：检查身份、时基并禁能；参数：c/bus不可空；返回：完成true，失败false。 */
bool robot_cl_init(robot_cl_t *c, const robot_cl_bus_t *bus)
{
    uint32_t id,cap,seq;
    robot_cl_bus_t saved;
    if (c == NULL || bus == NULL) return false;
    saved=*bus;
    memset(c,0,sizeof(*c));c->bus=saved;
    if (saved.read32 == NULL || saved.write32 == NULL) return false;
    /* 身份核实前不写任何寄存器，避免误操作单轮或开环核心。 */
    if (!saved.read32(saved.ctx,0,&id) || id!=ROBOT_CL_ID ||
        !saved.read32(saved.ctx,4,&cap) || cap!=4U) return false;
    c->initialized=true;
    if (!robot_cl_disarm(c) ||
        !saved.read32(saved.ctx,ROBOT_CL_SEQ,&seq) ||
        !saved.read32(saved.ctx,ROBOT_CL_LIMIT,&c->max_target_cps) ||
        !saved.read32(saved.ctx,ROBOT_CL_CLOCK,&c->clock_hz) ||
        !saved.read32(saved.ctx,ROBOT_CL_WINDOW,&c->window_cycles) ||
        !saved.read32(saved.ctx,ROBOT_CL_TIMEOUT,&c->watchdog_cycles) ||
        c->max_target_cps==0U || c->max_target_cps>INT32_MAX ||
        c->clock_hz==0U || c->window_cycles<70U || c->watchdog_cycles<2U) {
        (void)fail(c);c->initialized=false;return false;
    }
    c->next_sequence=seq+UINT32_C(1);
    return true;
}
/* 功能：禁能下完整配置；参数：c/config有效，范围见头文件；返回：写完true，否则false。 */
bool robot_cl_configure(robot_cl_t *c, const robot_cl_config_t *config)
{
    uint32_t status;
    unsigned i,active=0U;
    if (!usable(c)) return false;
    c->configured=false;c->fresh_for_arm=false;
    if (config==NULL || config->no_edge_windows==0U ||
        config->min_target_cps==0U || config->min_target_cps>c->max_target_cps ||
        config->min_effort==0U || config->min_effort>32767U) return fail(c);
    for(i=0;i<4U;i++) {
        if(config->kp_q16[i]>INT32_MAX || config->ki_tick_q16[i]>INT32_MAX ||
           config->effort_limit[i]>32767U) return fail(c);
        if(config->effort_limit[i]!=0U) {
            if(config->effort_limit[i]<config->min_effort ||
               (config->kp_q16[i]==0U && config->ki_tick_q16[i]==0U)) return fail(c);
            active++;
        }
    }
    if(active==0U) return fail(c);
    if (!robot_cl_status(c,&status) || (status&1U)!=0U) return fail(c);
    /* 先撤销全部轮子的输出上限，配置失败后保持禁能，不能自动恢复旧参数运行。 */
    for(i=0;i<4U;i++)
        if(!c->bus.write32(c->bus.ctx,(uint16_t)(ROBOT_CL_EFFORT_LIMIT+4U*i),0U)) return fail(c);
    for(i=0;i<4U;i++) {
        if (!c->bus.write32(c->bus.ctx,(uint16_t)(ROBOT_CL_GAINS+8U*i),config->kp_q16[i]) ||
            !c->bus.write32(c->bus.ctx,(uint16_t)(ROBOT_CL_GAINS+8U*i+4U),config->ki_tick_q16[i]))
            return fail(c);
    }
    if (!c->bus.write32(c->bus.ctx,ROBOT_CL_NO_EDGE,config->no_edge_windows) ||
        !c->bus.write32(c->bus.ctx,ROBOT_CL_MIN_TARGET,config->min_target_cps) ||
        !c->bus.write32(c->bus.ctx,ROBOT_CL_MIN_EFFORT,config->min_effort) ||
        !c->bus.write32(c->bus.ctx,ROBOT_CL_ILLEGAL_LIMIT,config->illegal_limit)) return fail(c);
    for(i=0;i<4U;i++) {
        uint32_t readback;
        uint16_t offset=(uint16_t)(ROBOT_CL_EFFORT_LIMIT+4U*i);
        if(!c->bus.write32(c->bus.ctx,offset,config->effort_limit[i]) ||
           !c->bus.read32(c->bus.ctx,offset,&readback) || readback!=config->effort_limit[i]) return fail(c);
    }
    c->configured=true;return true;
}
/* 功能：提交完整四轮命令；参数：c已配置，targets有效；返回：接受且序号匹配true，否则false。 */
bool robot_cl_submit(robot_cl_t *c, const int32_t targets[4])
{
    uint32_t accepted,sequence;
    unsigned i;
    if (!usable(c)) return false;
    if (!c->configured || targets==NULL) return fail(c);
    for(i=0;i<4U;i++)
        if ((int64_t)targets[i]>(int64_t)c->max_target_cps ||
            (int64_t)targets[i]< -(int64_t)c->max_target_cps) return fail(c);
    c->fresh_for_arm=false;sequence=c->next_sequence;
    if (!c->bus.write32(c->bus.ctx,ROBOT_CL_SEQ,sequence)) return fail(c);
    for(i=0;i<4U;i++)
        if (!c->bus.write32(c->bus.ctx,(uint16_t)(ROBOT_CL_TARGET+4U*i),(uint32_t)targets[i])) return fail(c);
    /* 即使COMMIT传输结果不确定也跳过该序号；失败后只能显式恢复。 */
    c->next_sequence=sequence+UINT32_C(1);
    if (!c->bus.write32(c->bus.ctx,ROBOT_CL_COMMIT_REG,ROBOT_CL_COMMIT) ||
        !c->bus.read32(c->bus.ctx,ROBOT_CL_SEQ,&accepted) || accepted!=sequence) return fail(c);
    c->fresh_for_arm=true;return true;
}
/* 功能：请求arm并拒绝已过期/故障状态；参数：c已配置提交；返回：请求写成功true，否则false。 */
bool robot_cl_request_arm(robot_cl_t *c)
{
    uint32_t status;
    if (!usable(c)) return false;
    if (!c->configured || !c->fresh_for_arm || !robot_cl_status(c,&status) || (status&6U)!=0U)
        return fail(c);
    if (!c->bus.write32(c->bus.ctx,ROBOT_CL_CTRL,1U)) return fail(c);
    return true;
}
/* 功能：请求禁能；参数：c已初始化；返回：写成功true，否则false。 */
bool robot_cl_disarm(robot_cl_t *c)
{
    if (!usable(c)) return false;
    c->fresh_for_arm=false;
    return c->bus.write32(c->bus.ctx,ROBOT_CL_CTRL,2U);
}
/* 功能：请求清故障；参数：c已初始化；返回：写成功true，否则false。 */
bool robot_cl_clear_fault(robot_cl_t *c)
{
    if (!usable(c)) return false;
    c->fresh_for_arm=false;
    if (!c->bus.write32(c->bus.ctx,ROBOT_CL_CTRL,8U)) return fail(c);
    return true;
}
/* 功能：读实时状态；参数：c/status不可空；返回：读取成功true，否则false且status清零。 */
bool robot_cl_status(robot_cl_t *c, uint32_t *status)
{
    uint32_t value;
    if (status==NULL) return false;
    *status=0;
    if (!usable(c) || !c->bus.read32(c->bus.ctx,ROBOT_CL_STATUS,&value)) return false;
    *status=value;
    return true;
}
/* 功能：读取完整原子快照；参数：c/out不可空；返回：一致且成功true，否则false并清零。 */
bool robot_cl_snapshot(robot_cl_t *c, robot_cl_snapshot_t *out)
{
    robot_cl_snapshot_t tmp={0};
    uint32_t v,last;
    unsigned i;
    if(out==NULL) return false;
    memset(out,0,sizeof(*out));
    if(!usable(c) || !c->bus.write32(c->bus.ctx,ROBOT_CL_CTRL,4U) ||
       !c->bus.read32(c->bus.ctx,ROBOT_CL_SNAP_SEQ,&tmp.sequence)) return false;
    for(i=0;i<4U;i++) {
        if(!c->bus.read32(c->bus.ctx,(uint16_t)(ROBOT_CL_SNAP_TARGET+4U*i),&v)) return false;
        tmp.target_cps[i]=signed_value(v);
        if(!c->bus.read32(c->bus.ctx,(uint16_t)(ROBOT_CL_SPEED+4U*i),&v)) return false;
        tmp.measured_cps[i]=signed_value(v);
        if(!c->bus.read32(c->bus.ctx,(uint16_t)(ROBOT_CL_POSITION+4U*i),&v)) return false;
        tmp.position[i]=signed_value(v);
        if(!c->bus.read32(c->bus.ctx,(uint16_t)(ROBOT_CL_ILLEGAL+4U*i),&tmp.illegal[i])) return false;
        if(!c->bus.read32(c->bus.ctx,(uint16_t)(ROBOT_CL_EFFORT+4U*i),&v)) return false;
        tmp.effort[i]=signed_value(v);
        if(tmp.effort[i]<-32767 || tmp.effort[i]>32767) return false;
    }
    if(!c->bus.read32(c->bus.ctx,ROBOT_CL_DIAG,&tmp.diagnostics) ||
       !c->bus.read32(c->bus.ctx,ROBOT_CL_SNAP_STATUS,&tmp.status) ||
       !c->bus.read32(c->bus.ctx,ROBOT_CL_SAMPLE_SEQ,&tmp.sample_sequence) ||
       !c->bus.read32(c->bus.ctx,ROBOT_CL_SNAP_SEQ,&last) || last!=tmp.sequence) return false;
    *out=tmp;return true;
}
