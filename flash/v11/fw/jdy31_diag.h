#ifndef JDY31_DIAG_H
#define JDY31_DIAG_H
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
/* 仅测试租约，无运动接口；保护寄存器状态bit0有效、bit1故障锁存。 */
enum { JDY_G_CLEAR=1, JDY_G_ARM=2, JDY_G_KICK=3, JDY_G_STOP=4 };
typedef struct {
    void *ctx; /* 用户上下文，可空。 */
    bool (*write)(void *,const char *,size_t); /* 完整应答入队，禁止半包写入。 */
    void (*guard)(void *,unsigned,uint32_t); /* 事件及uint32序号，无返回。 */
    uint32_t (*state)(void *); /* 读取独立硬件保护。 */
} jdy_diag_io_t;
typedef struct {
    jdy_diag_io_t io;
    uint32_t epoch,sequence,accepted,rejected,last_rx;
    uint32_t key,level; /* 遥控指令回读：1前进/2后退/3左转/4右转/5左旋/6右旋，档位1..3；0为停止。 */
    unsigned length;
    bool active,blocked,dropping;
    char line[80]; /* 最大79字节ASCII指令，换行结束；静态分配。 */
} jdy_diag_t;
/* 功能：初始化直通诊断；s/io与回调不可空，now为毫秒；无返回。 */
void jdy_diag_init(jdy_diag_t *s,const jdy_diag_io_t *io,uint32_t now);
/* 功能：输入UART字节；s不可空，byte任意字节，now为毫秒；无返回。 */
void jdy_diag_rx(jdy_diag_t *s,uint8_t byte,uint32_t now);
/* 功能：同步硬件故障与清理残包；s不可空，now为毫秒；无返回。 */
void jdy_diag_poll(jdy_diag_t *s,uint32_t now);
/* 功能：锁存硬件UART异常直到复位；s不可空；无返回。 */
void jdy_diag_transport_fault(jdy_diag_t *s);
#endif
