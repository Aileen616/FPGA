#include "jdy31_driver.h"
/* 来源：共享esp_diag_wb.sv；BX72软核无STM32 HAL，MMIO仅封装在本板级文件。 */
#define JDY_BASE UINT32_C(0x20002000)
static jdy31_driver_t bluetooth; /* 独立驱动实例，静态分配，不接整车模块。 */
/* 功能：读取诊断MMIO；ctx未使用可空，offset为已定义4字节对齐偏移；返回32位值。 */
static uint32_t rd(void *ctx,unsigned offset)
{ (void)ctx;uint32_t v=*(volatile uint32_t *)(uintptr_t)(JDY_BASE+offset);__asm__ volatile("fence iorw,iorw":::"memory");return v; }
/* 功能：写诊断MMIO；ctx未使用可空，offset为已定义偏移，v为合法值；无返回。 */
static void wr(void *ctx,unsigned offset,uint32_t v)
{ (void)ctx;__asm__ volatile("fence iorw,iorw":::"memory");*(volatile uint32_t *)(uintptr_t)(JDY_BASE+offset)=v; }
/* 功能：独立裸机诊断入口；无参数，正常不返回；任何情况下均无电机控制调用。 */
int main(void)
{
    const jdy31_bus_t bus={0,rd,wr};
    if(!jdy31_init(&bluetooth,&bus))for(;;) { }
    for(;;)jdy31_poll(&bluetooth);
}
