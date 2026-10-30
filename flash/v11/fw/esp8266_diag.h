#ifndef ESP8266_DIAG_H
#define ESP8266_DIAG_H
#include <stdbool.h>
#include <stdint.h>
#include <stddef.h>
/* 仅诊断测试租约，无运动接口。状态位：bit0测试有效，bit1故障锁存。 */
enum { ESP_G_CLEAR=1, ESP_G_ARM=2, ESP_G_KICK=3, ESP_G_STOP=4 };
typedef struct {
    void *ctx; /* 回调上下文，可为空。 */
    bool (*write)(void *, const char *, size_t); /* 完整入队才返回true，不得部分入队。 */
    void (*guard)(void *, unsigned, uint32_t); /* 参数为上述事件及递增序号。 */
    uint32_t (*state)(void *); /* 读取独立硬件保护状态。 */
} esp_diag_io_t;
typedef struct {
    esp_diag_io_t io;
    uint32_t now, deadline, last_rx, epoch, sequence, accepted, rejected;
    uint32_t payload_left;
    int owner, payload_id, reply_id;
    unsigned init_step, send_state, at_len, command_len[5], reply_len;
    bool ready, blocked, waiting, active, connected[5];
    char at_line[192], command[5][80], reply[192]; /* 静态存储，禁止置于小栈。 */
} esp_diag_t;
/* 功能：初始化无动力诊断协议；s/io及其中回调不可空，now为回绕毫秒时基；无返回。 */
void esp_diag_init(esp_diag_t *s, const esp_diag_io_t *io, uint32_t now);
/* 功能：处理串口一个字节；s不可空，byte为0..255，now为当前毫秒；无返回。 */
void esp_diag_rx(esp_diag_t *s, uint8_t byte, uint32_t now);
/* 功能：非阻塞服务AT、发包和保护；s不可空，now为当前毫秒；主循环持续调用，无返回。 */
void esp_diag_poll(esp_diag_t *s, uint32_t now);
/* 功能：锁存串口溢出/帧错误并禁用协议至复位；s不可空；无返回。 */
void esp_diag_transport_fault(esp_diag_t *s);
#endif
