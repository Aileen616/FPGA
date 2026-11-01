#ifndef JDY31_MOTOR_H
#define JDY31_MOTOR_H
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

/* 保护状态：bit0为运动及通信同时有效，bit1为任一锁存故障，bit2为通信有效；不是实测速度。 */
enum { JDY_M_CLEAR=1, JDY_M_ARM=2, JDY_M_STOP=4 };
typedef struct {
    void *ctx; /* 板级上下文，可空；回调函数本身均不可空。 */
    bool (*write)(void *,const char *,size_t); /* 应答必须整包入队，失败即停机。 */
    void (*guard)(void *,unsigned,uint32_t); /* 清故障、启动或停止；最后参数保留且固定0。 */
    uint32_t (*state)(void *); /* bit2仅供首次启动检查；对外状态仅公布低两位。 */
    bool (*configure)(void *,uint32_t,uint32_t); /* 停机时设置轮掩码1..15、方向翻转0..15。 */
    bool (*drive)(void *,uint32_t,uint32_t,uint32_t); /* 合法运动提交：序号1..UINT32_MAX、方向1..10、档位1..3；失败停机。 */
} jdy_motor_io_t;
typedef struct {
    jdy_motor_io_t io;
    uint32_t epoch,sequence,accepted,rejected,last_rx;
    uint32_t key,level; /* 方向1..10、输出档位1..3；0表示停止，不是实测速度。 */
    uint32_t wheel_mask,invert; /* 物理位0..3为通道3/2/4/1；初始尚未配置。 */
    unsigned length;
    bool active,blocked,dropping,configured;
    char line[80]; /* 最长79字节ASCII命令；禁止截断后执行。 */
} jdy_motor_t;

/* 功能：初始化完整低输出遥控协议并停止；s/io及所有回调不可空，now为毫秒；无返回。 */
void jdy_motor_init(jdy_motor_t *s,const jdy_motor_io_t *io,uint32_t now);
/* 功能：处理一个UART字节；s不可空，byte为任意字节，now为毫秒；无返回。 */
void jdy_motor_rx(jdy_motor_t *s,uint8_t byte,uint32_t now);
/* 功能：同步硬件故障并清理超时残包；s不可空，now为毫秒；无返回。 */
void jdy_motor_poll(jdy_motor_t *s,uint32_t now);
/* 功能：串口故障永久撤销本次运行；s不可空；无返回，只能通过复位恢复。 */
void jdy_motor_transport_fault(jdy_motor_t *s);
#endif
