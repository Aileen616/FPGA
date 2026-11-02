#include "esp8266_diag.h"
/* 来源：esp_diag_wb.sv；BX72软核无STM32 HAL，MMIO仅封装在本板级文件。 */
#define ESP_BASE UINT32_C(0x20002000)
static esp_diag_t diag; /* 约1KB静态状态，不分配堆。 */
static char tx[512]; /* 有界发送队列；满时整个事务拒绝，不发送半条AT命令。 */
static unsigned head,tail;
/* 功能：读取诊断MMIO；offset为已定义4字节对齐偏移；返回32位值。 */
static uint32_t rd(unsigned offset)
{ uint32_t v=*(volatile uint32_t *)(uintptr_t)(ESP_BASE+offset);__asm__ volatile("fence iorw,iorw":::"memory");return v; }
/* 功能：写诊断MMIO；offset为已定义偏移，v为寄存器合法值；无返回。 */
static void wr(unsigned offset,uint32_t v)
{ __asm__ volatile("fence iorw,iorw":::"memory");*(volatile uint32_t *)(uintptr_t)(ESP_BASE+offset)=v; }
/* 功能：整包入队；ctx不用可空，data不可空，n为字节数；容量足够返回true。 */
static bool enqueue(void *ctx,const char *data,size_t n)
{
    (void)ctx;if(n>(tail+sizeof(tx)-head-1U)%sizeof(tx)) return false;
    for(size_t i=0;i<n;i++) {tx[head]=data[i];head=(head+1U)%sizeof(tx);}return true;
}
/* 功能：提交测试保护事件；ctx不用可空，event为枚举，seq为心跳序号；无返回。 */
static void guard(void *ctx,unsigned event,uint32_t seq)
{
    (void)ctx;
    if(event==ESP_G_KICK) wr(0x10,seq);
    else wr(0x0c,event==ESP_G_CLEAR?1U:event==ESP_G_ARM?2U:4U);
}
/* 功能：返回硬件测试状态；ctx不用可空；bit0有效、bit1故障。 */
static uint32_t state(void *ctx) { (void)ctx;return rd(8); }
/* 功能：独立裸机诊断入口；无参数，正常不返回；任何情况下均无电机控制调用。 */
int main(void)
{
    const esp_diag_io_t io={0,enqueue,guard,state};
    if(rd(0)!=UINT32_C(0x45535001)) for(;;) { }
    /* 模块启动阶段可能有74880波特率日志；先清空2秒，之后只接受115200的AT响应。 */
    uint32_t start=rd(4);
    while((uint32_t)(rd(4)-start)<2000U) (void)rd(0x14);
    /* UART错误锁存到FPGA复位：模块应先上电，再复位FPGA，避免启动乱码污染运行。 */
    esp_diag_init(&diag,&io,rd(4));
    for(;;) {
        uint32_t now=rd(4);
        if(rd(0x1c)) esp_diag_transport_fault(&diag);
        esp_diag_poll(&diag,now);
        uint32_t c=rd(0x14);
        if(!(c&0x100U)) esp_diag_rx(&diag,(uint8_t)c,now);
        if(tail!=head && !(rd(0x18)&0x100U)) {wr(0x18,(uint8_t)tx[tail]);tail=(tail+1U)%sizeof(tx);}
    }
}
