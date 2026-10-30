#include "robot_bsp.h"
#include <stddef.h>
/* CPU单线程且不开中断，无其他总线主机；错误寄存器与数据访问组成连续事务。 */
static uint32_t heartbeat_sequence; /* 自然回绕，PL仅接受新序号。 */
static bool diagnostic; /* 由PL能力位取得，禁止通过串口更改。 */
/* 功能：读取一个32位MMIO字；address为已验证全字地址；返回寄存器值。 */
static uint32_t read_word(uint32_t address)
{
    uint32_t value=*(volatile uint32_t *)(uintptr_t)address;
    __asm__ volatile("fence iorw,iorw" ::: "memory");
    return value;
}
/* 功能：完成一次32位MMIO写；address全字对齐，value任意32位；无返回值。 */
static void write_word(uint32_t address,uint32_t value)
{
    __asm__ volatile("fence iorw,iorw" ::: "memory");
    *(volatile uint32_t *)(uintptr_t)address=value;
    __asm__ volatile("fence iorw,iorw" ::: "memory");
}
/* 功能：机器人读回调；ctx不用可空，offset为0..0xffc全字偏移，value不可空；成功true。 */
static bool control_read(void *ctx,uint16_t offset,uint32_t *value)
{
    (void)ctx;
    if(value==NULL) return false;
    *value=0;
    if(offset>0xffcU || (offset&3U)!=0U) return false;
    uint32_t v=read_word(BX72_DRIVE_BASE+offset);
    if(read_word(BX72_SYSTEM_BASE+0x14U)!=0U) return false;
    *value=v;return true;
}
/* 功能：机器人写回调；ctx不用可空，offset为0..0xffc全字偏移；PL接受写入返回true。 */
static bool control_write(void *ctx,uint16_t offset,uint32_t value)
{
    (void)ctx;
    if(offset>0xffcU || (offset&3U)!=0U) return false;
    write_word(BX72_DRIVE_BASE+offset,value);
    return read_word(BX72_SYSTEM_BASE+0x14U)==0U;
}
/* 功能/参数/返回见头文件；总线在PL固定拍数结束，避免错误地址令CPU无限等待。 */
bool bx72_bsp_init(robot_bus_t *bus)
{
    if(bus==NULL || read_word(BX72_SYSTEM_BASE)!=UINT32_C(0x42585301)) return false;
    uint32_t clock_hz=read_word(BX72_SYSTEM_BASE+8U);
    if(clock_hz<1000000U) return false;
    diagnostic=(read_word(BX72_SYSTEM_BASE+4U)&1U)!=0U;
    heartbeat_sequence=read_word(BX72_SYSTEM_BASE+0x2cU);
    write_word(BX72_UART_BASE,clock_hz/115200U);
    bus->ctx=NULL;bus->read32=control_read;bus->write32=control_write;
    return bx72_heartbeat();
}
/* 功能/参数/返回见头文件。 */
uint32_t bx72_now_ms(void) { return read_word(BX72_SYSTEM_BASE+0x0cU); }
/* 功能/参数/返回见头文件；普通读状态或UART字符均不能为心跳续期。 */
bool bx72_heartbeat(void)
{
    write_word(BX72_SYSTEM_BASE+0x10U,++heartbeat_sequence);
    return read_word(BX72_SYSTEM_BASE+0x14U)==0U;
}
/* 功能/参数/返回见头文件。 */
bool bx72_diagnostic_only(void) { return diagnostic; }
/* 功能/参数/返回见头文件。 */
uint32_t bx72_safety_inputs(void) { return read_word(BX72_SYSTEM_BASE+0x24U); }
/* 功能/参数/返回见头文件。 */
void bx72_stage(uint32_t stage) { write_word(BX72_SYSTEM_BASE+0x20U,stage); }
/* 功能/参数/返回见头文件；沿用官方SDK的RX空位bit8，不推测未说明的错误位。 */
int bx72_uart_get(uint8_t *byte)
{
    if(byte==NULL) return -1;
    uint32_t rx=read_word(BX72_UART_BASE+8U);
    if((rx&0x100U)!=0U) return 0;
    *byte=(uint8_t)rx;return 1;
}
/* 功能/参数/返回见头文件；官方UART TX bit8表示忙。 */
bool bx72_uart_put(uint8_t byte)
{
    if((read_word(BX72_UART_BASE+0x0cU)&0x100U)!=0U) return false;
    write_word(BX72_UART_BASE+0x0cU,byte);return true;
}
