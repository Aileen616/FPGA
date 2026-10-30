#ifndef ROBOT_OPEN_LOOP_H
#define ROBOT_OPEN_LOOP_H

#include "mecanum_mixer.h"

#define ROBOT_OL_ID UINT32_C(0x42584f01)
#define ROBOT_OL_CAPS UINT32_C(0x00000401)
#define ROBOT_OL_COMMIT UINT32_C(0x4f4c0001)

/* 寄存器均为字节偏移，不是物理地址；实际CPU总线由平台回调接入。 */
enum {
    ROBOT_OL_REG_ID = 0x000, ROBOT_OL_REG_CAPS = 0x004,
    ROBOT_OL_REG_CTRL = 0x008, ROBOT_OL_REG_STATUS = 0x00c,
    ROBOT_OL_REG_SEQ = 0x010, ROBOT_OL_REG_COMMIT = 0x014,
    ROBOT_OL_REG_MAX = 0x018, ROBOT_OL_REG_TARGET = 0x020,
    ROBOT_OL_REG_SNAP_TARGET = 0x030, ROBOT_OL_REG_SNAP_APPLIED = 0x040,
    ROBOT_OL_REG_SNAP_SEQ = 0x060, ROBOT_OL_REG_AGE = 0x064,
    ROBOT_OL_REG_FAULT = 0x068, ROBOT_OL_REG_ERRORS = 0x06c,
    ROBOT_OL_REG_SNAP_IO = 0x070, ROBOT_OL_REG_SNAP_STATUS = 0x074,
    ROBOT_OL_REG_CLOCK = 0x080, ROBOT_OL_REG_TIMEOUT = 0x084
};

/* 功能：完成一个32位总线读事务；ctx可空，offset字节偏移，value不可空。
 * 返回：完成且无总线错误true，否则false；必须有有限超时，禁止无限等待。
 */
typedef bool (*robot_read32_fn)(void *ctx, uint16_t offset, uint32_t *value);
/* 功能：完成一个32位总线写事务；ctx可空，offset字节偏移，value全字数据。
 * 返回：写入完成且无总线错误true，否则false；回调必须保证写序和有限超时。
 */
typedef bool (*robot_write32_fn)(void *ctx, uint16_t offset, uint32_t value);

typedef struct {
    void *ctx; /* 平台私有上下文，可为空。 */
    robot_read32_fn read32;
    robot_write32_fn write32;
} robot_bus_t;

typedef struct {
    robot_bus_t bus;
    mecanum_config_t config;
    uint32_t next_sequence; /* 模2^32递增；初始化读取最后序号后加一。 */
    uint32_t clock_hz; /* 从核心读取，Hz；仅用于解释命令年龄。 */
    uint32_t watchdog_cycles; /* 从核心读取，系统拍。 */
    bool initialized;
    bool fresh_for_arm; /* 本软件实例自上次clear/disarm以后是否成功提交过命令。 */
} robot_open_loop_t;

typedef struct {
    mecanum_output_t target; /* 同一快照的目标驱动力；不是目标counts/s。 */
    mecanum_output_t applied; /* 同一快照的斜坡输出；不是实测轮速或实际电机转速。 */
    uint32_t sequence;
    uint32_t io_bits; /* 快照bit0..3使能，4..7方向，8..11瞬时PWM电平。 */
    uint32_t status; /* 快照bit0 armed，1 fault，2 stale，3 allowed，8..13故障原因。 */
} robot_open_loop_snapshot_t;

/* 功能：核对开环ID/能力/限幅/时基并发送禁能请求；不自动arm。
 * 参数：client、bus、config不可为空；回调必须有效，调用方串行访问同一设备。
 * 返回：初始化成功true，任一检查/事务失败false；不代表实体已通过测试。
 */
bool robot_ol_init(robot_open_loop_t *client, const robot_bus_t *bus,
                   const mecanum_config_t *config);

/* 功能：混合并原子提交四轮新命令，也可用于周期心跳；不自动arm。
 * 参数：client已初始化，command不可空且每轴-1000..1000。
 * 返回：提交并核对序号成功true；失败false并尝试一次禁能，不无限重试。
 */
bool robot_ol_submit(robot_open_loop_t *client, const mecanum_command_t *command);

/* 功能：显式请求使能；参数：client已初始化且刚提交合法命令，不可空。
 * 返回：请求写入成功true，否则false；须读状态确认，不能把true当作已运动。
 */
bool robot_ol_request_arm(robot_open_loop_t *client);

/* 功能：显式禁能；参数：client已初始化，不可空。
 * 返回：请求写入成功true，否则false；写失败时仍由RTL看门狗独立禁能。
 */
bool robot_ol_disarm(robot_open_loop_t *client);

/* 功能：请求清故障并清除重新使能资格；参数：client不可空，真实故障应已解除。
 * 返回：请求写入成功true，否则false；清故障不提交命令、不自动恢复运动。
 */
bool robot_ol_clear_fault(robot_open_loop_t *client);

/* 功能：冻结并读取四轮输出状态；参数：client和snapshot不可空，禁止并发snapshot写入。
 * 返回：成功且前后快照编号一致true，否则false并将非空snapshot清零。
 */
bool robot_ol_snapshot(robot_open_loop_t *client, robot_open_loop_snapshot_t *snapshot);

#endif
