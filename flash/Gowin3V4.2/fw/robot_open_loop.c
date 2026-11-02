#include "robot_open_loop.h"
#include <stddef.h>
#include <string.h>

/* 无外部全局变量、物理寄存器地址或外设句柄；平台仅需实现有界读写回调。 */

/* 功能：检查实例可用性；参数：client可空；返回：初始化且回调有效为true。 */
static bool usable(const robot_open_loop_t *client)
{
    return client != NULL && client->initialized &&
           client->bus.read32 != NULL && client->bus.write32 != NULL;
}

/* 功能：异常后尽力撤销驱动；参数：client可空；返回：固定false供上层传播失败。 */
static bool fail_disarm(robot_open_loop_t *client)
{
    if (client != NULL) {
        client->fresh_for_arm = false;
        if (client->bus.write32 != NULL) {
            (void)client->bus.write32(client->bus.ctx, ROBOT_OL_REG_CTRL, 2U);
        }
    }
    return false;
}

/* 功能：将32位符号扩展寄存器解码为signed16；参数：value为寄存器值。
 * 返回：-32768..32767；通过signed32算术避免超范围无符号到有符号转换。
 */
static int16_t decode_effort(uint32_t value)
{
    const uint32_t low = value & UINT32_C(0xffff);
    return (int16_t)(low >= UINT32_C(0x8000) ? (int32_t)low - 65536 : (int32_t)low);
}

/* 功能：绑定总线、验证接口并禁能；参数：client/bus/config均不可空。
 * 返回：全部操作成功true；失败false，不自动选择CPU或使能电机。
 */
bool robot_ol_init(robot_open_loop_t *client, const robot_bus_t *bus,
                   const mecanum_config_t *config)
{
    uint32_t id, caps, limit, previous;
    robot_bus_t saved_bus;
    mecanum_config_t saved_config;
    if (client == NULL || bus == NULL || config == NULL) {
        return false;
    }
    saved_bus = *bus;
    saved_config = *config;
    memset(client, 0, sizeof(*client));
    client->bus = saved_bus;
    client->config = saved_config;
    if (saved_bus.read32 == NULL || saved_bus.write32 == NULL) {
        return false;
    }
    /* 先确认接口身份，不能向另一个外设或M1速度核心误写控制寄存器。 */
    if (!saved_bus.read32(saved_bus.ctx, ROBOT_OL_REG_ID, &id) || id != ROBOT_OL_ID ||
        !saved_bus.read32(saved_bus.ctx, ROBOT_OL_REG_CAPS, &caps) || caps != ROBOT_OL_CAPS) {
        return false;
    }
    if (!saved_bus.write32(saved_bus.ctx, ROBOT_OL_REG_CTRL, 2U) ||
        !mecanum_config_valid(&saved_config) ||
        !saved_bus.read32(saved_bus.ctx, ROBOT_OL_REG_MAX, &limit) ||
        limit == 0U || limit > 32767U || saved_config.max_effort > limit ||
        !saved_bus.read32(saved_bus.ctx, ROBOT_OL_REG_SEQ, &previous) ||
        !saved_bus.read32(saved_bus.ctx, ROBOT_OL_REG_CLOCK, &client->clock_hz) ||
        !saved_bus.read32(saved_bus.ctx, ROBOT_OL_REG_TIMEOUT, &client->watchdog_cycles) ||
        client->clock_hz == 0U || client->watchdog_cycles < 2U) {
        return false;
    }
    client->next_sequence = previous + UINT32_C(1);
    client->initialized = true;
    return true;
}

/* 功能：发送序号、完整四轮shadow及commit；参数：client已初始化，command不可空。
 * 返回：成功true；任何异常false并尝试禁能；命令源过期时应用层必须停止调用本函数。
 */
bool robot_ol_submit(robot_open_loop_t *client, const mecanum_command_t *command)
{
    mecanum_output_t output;
    uint32_t accepted;
    unsigned int i;
    if (!usable(client)) {
        return false;
    }
    if (!mecanum_mix(&client->config, command, &output)) {
        return fail_disarm(client);
    }
    client->fresh_for_arm = false;
    if (!client->bus.write32(client->bus.ctx, ROBOT_OL_REG_SEQ, client->next_sequence)) {
        return fail_disarm(client);
    }
    for (i = 0U; i < 4U; ++i) {
        if (!client->bus.write32(client->bus.ctx, (uint16_t)(ROBOT_OL_REG_TARGET + 4U * i),
                                 (uint32_t)(int32_t)output.effort[i])) {
            return fail_disarm(client);
        }
    }
    if (!client->bus.write32(client->bus.ctx, ROBOT_OL_REG_COMMIT, ROBOT_OL_COMMIT)) {
        /* 写返回失败也可能已到达硬件；跳过该序号，避免重放不确定的事务。 */
        client->next_sequence += UINT32_C(1);
        return fail_disarm(client);
    }
    accepted = client->next_sequence;
    client->next_sequence += UINT32_C(1);
    if (!client->bus.read32(client->bus.ctx, ROBOT_OL_REG_SEQ, &accepted) ||
        accepted != client->next_sequence - UINT32_C(1)) {
        return fail_disarm(client);
    }
    client->fresh_for_arm = true;
    return true;
}

/* 功能：请求使能；参数：client不可空且已成功提交新命令；返回：写接受true，否则false。 */
bool robot_ol_request_arm(robot_open_loop_t *client)
{
    if (!usable(client) || !client->fresh_for_arm) {
        return false;
    }
    if (!client->bus.write32(client->bus.ctx, ROBOT_OL_REG_CTRL, 1U)) {
        return fail_disarm(client);
    }
    return true;
}

/* 功能：请求禁能并清除软件重新使能资格；参数：client不可空；返回：写接受true，否则false。 */
bool robot_ol_disarm(robot_open_loop_t *client)
{
    if (!usable(client)) {
        return false;
    }
    client->fresh_for_arm = false;
    return client->bus.write32(client->bus.ctx, ROBOT_OL_REG_CTRL, 2U);
}

/* 功能：请求清故障，保持禁能；参数：client不可空；返回：写接受true，否则false。 */
bool robot_ol_clear_fault(robot_open_loop_t *client)
{
    if (!usable(client)) {
        return false;
    }
    client->fresh_for_arm = false;
    if (!client->bus.write32(client->bus.ctx, ROBOT_OL_REG_CTRL, 8U)) {
        return fail_disarm(client);
    }
    return true;
}

/* 功能：获取同一时刻的四轮快照；参数：client/snapshot不可空；返回：完整读取成功true。 */
bool robot_ol_snapshot(robot_open_loop_t *client, robot_open_loop_snapshot_t *snapshot)
{
    robot_open_loop_snapshot_t temporary = {0};
    uint32_t value, end_sequence;
    unsigned int i;
    if (snapshot == NULL) {
        return false;
    }
    memset(snapshot, 0, sizeof(*snapshot));
    if (!usable(client) || !client->bus.write32(client->bus.ctx, ROBOT_OL_REG_CTRL, 4U) ||
        !client->bus.read32(client->bus.ctx, ROBOT_OL_REG_SNAP_SEQ, &temporary.sequence)) {
        return false;
    }
    for (i = 0U; i < 4U; ++i) {
        if (!client->bus.read32(client->bus.ctx, (uint16_t)(ROBOT_OL_REG_SNAP_TARGET + 4U * i), &value)) {
            return false;
        }
        temporary.target.effort[i] = decode_effort(value);
        if (!client->bus.read32(client->bus.ctx, (uint16_t)(ROBOT_OL_REG_SNAP_APPLIED + 4U * i), &value)) {
            return false;
        }
        temporary.applied.effort[i] = decode_effort(value);
    }
    if (!client->bus.read32(client->bus.ctx, ROBOT_OL_REG_SNAP_IO, &temporary.io_bits) ||
        !client->bus.read32(client->bus.ctx, ROBOT_OL_REG_SNAP_STATUS, &temporary.status) ||
        !client->bus.read32(client->bus.ctx, ROBOT_OL_REG_SNAP_SEQ, &end_sequence) ||
        end_sequence != temporary.sequence) {
        return false;
    }
    *snapshot = temporary;
    return true;
}
