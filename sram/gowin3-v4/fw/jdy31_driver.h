#ifndef JDY31_DRIVER_H
#define JDY31_DRIVER_H

#include "jdy31_diag.h"

/* 指令仅代表手机操作意图；驱动不调用电机、导航或整车应用。 */
typedef enum {
    JDY31_STOP=0, JDY31_FORWARD=1, JDY31_BACKWARD=2,
    JDY31_LEFT=3, JDY31_RIGHT=4, JDY31_ROTATE_LEFT=5, JDY31_ROTATE_RIGHT=6
} jdy31_key_t;

typedef struct {
    void *ctx; /* 总线上下文，可空；本驱动不持有其生命周期。 */
    uint32_t (*read32)(void *ctx,unsigned offset); /* 同步读取；offset为0..0x1c的4字节对齐偏移。 */
    void (*write32)(void *ctx,unsigned offset,uint32_t value); /* 同步写入，不可空。 */
} jdy31_bus_t;

typedef struct {
    jdy31_key_t key; /* 0停止或1..6方向；valid=false时固定为0。 */
    uint8_t level; /* 1低/2中/3高档，非实测速度；无效时为0。 */
    bool valid; /* 硬件租约有效且已收到合法方向指令时为true。 */
    bool fault; /* 保护锁存或UART故障；清除不代表实车已允许运动。 */
    uint32_t epoch,sequence; /* 会话代号/递增序号，无单位。 */
} jdy31_command_t;

typedef struct {
    jdy_diag_t protocol; /* 私有协议状态，仅驱动函数修改。 */
    jdy31_bus_t bus; /* 初始化时复制总线访问接口。 */
    uint8_t tx[512]; /* 静态有界TX队列，保留1字节区分满/空。 */
    unsigned head,tail; /* 环形队列索引0..511，无单位。 */
    bool initialized; /* ID匹配且初始化完成时置位。 */
} jdy31_driver_t;

/* 功能：初始化独立驱动并撤销旧租约；driver/bus及回调不可空；成功true，参数或外设ID错误false。 */
bool jdy31_init(jdy31_driver_t *driver,const jdy31_bus_t *bus);
/* 功能：非阻塞处理最多8个RX字节和1个TX字节；driver可空或未初始化，此时无操作；无返回。 */
void jdy31_poll(jdy31_driver_t *driver);
/* 功能：取得当前有效指令快照；driver/out不可空；可读true，未初始化/参数错误false且out若非空置安全默认。 */
bool jdy31_get_command(jdy31_driver_t *driver,jdy31_command_t *out);
/* 功能：立即撤销租约并使旧会话指令失效；driver可空/未初始化，此时无操作；无返回。 */
void jdy31_stop(jdy31_driver_t *driver);

/* 裸机单线程调用；不要在中断和主循环中同时访问同一driver。推荐静态分配并每1ms或更快轮询。 */
#endif
