#ifndef BX72_ROBOT_BSP_H
#define BX72_ROBOT_BSP_H
#include <stdbool.h>
#include <stdint.h>
#include "robot_open_loop.h"
/* 地址来源：官方SDK的ITCM/DTCM/WBUART/Open-WB映射；扩展窗口见robot_wb_bridge.sv。
 * 当前无厂家机器人HAL；仅在本BSP内执行有序MMIO，算法层继续使用回调。
 */
#define BX72_DRIVE_BASE UINT32_C(0x20000000)
#define BX72_SYSTEM_BASE UINT32_C(0x20001000)
#define BX72_UART_BASE UINT32_C(0x11000000)
/* 功能：校验协作桥、初始化UART并提供控制回调；bus不可空；成功true。 */
bool bx72_bsp_init(robot_bus_t *bus);
/* 功能：读取PL毫秒时钟，uint32自然回绕；无参数；返回启动后的毫秒数。 */
uint32_t bx72_now_ms(void);
/* 功能：提交严格前进心跳序号；无参数；返回总线成功true，失败false。 */
bool bx72_heartbeat(void);
/* 功能：查询诊断构建标志；无参数；true表示该顶层不连接电机且禁止软件arm。 */
bool bx72_diagnostic_only(void);
/* 功能：读取同步安全输入；无参数；返回bit0急停/1驱动/3传感/4障碍/5心跳。 */
uint32_t bx72_safety_inputs(void);
/* 功能：发布诊断阶段；stage为32位开发诊断值；无返回值，不影响运动许可。 */
void bx72_stage(uint32_t stage);
/* 功能：尝试读取一个串口字节；byte不可空；返回1字节、0无数据、-1参数错误。 */
int bx72_uart_get(uint8_t *byte);
/* 功能：尝试发送一字节；byte为0..255；true已写入，false暂忙；不阻塞。 */
bool bx72_uart_put(uint8_t byte);
#endif
