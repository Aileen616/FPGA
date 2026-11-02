#include "perf_monitor.h"
/* 来源：独立runtime_perf_wb外设；官方裸机平台无STM32 HAL，MMIO仅封装于本文件。 */
#define PERF_BASE UINT32_C(0x20006000)
static perf_write_t writer; /* 整包非阻塞发送回调，由主循环初始化。 */
static unsigned phase,index,used; /* 状态0空闲、1发起、2等快照、3逐字读回、4等待入队。 */
static uint32_t start_ms; /* 本次请求起点，单位毫秒，差值按uint32自然回绕。 */
static char report[272]; /* 27个32位十六进制字段及固定头尾，避免大栈数组。 */
static bool available; /* 只有读到PRF1版本ID才使能统计。 */
bool perf_monitor_pending; /* 对外仅表示需要推进诊断报告，不影响控制模式。 */

/* 功能：读取外设整字；offset为合法4字节对齐偏移；返回32位寄存器值。 */
static uint32_t read_reg(unsigned offset)
{uint32_t v=*(volatile uint32_t *)(uintptr_t)(PERF_BASE+offset);__asm__ volatile("":::"memory");return v;}
/* 功能：写统计寄存器；offset为合法偏移，value为整字值；无返回。 */
static void write_reg(unsigned offset,uint32_t value)
{__asm__ volatile("":::"memory");*(volatile uint32_t *)(uintptr_t)(PERF_BASE+offset)=value;}
/* 功能：追加固定8位十六进制字段；v为任意32位数；无返回，缓冲容量由固定字段数保证。 */
static void hex_word(uint32_t v)
{static const char digits[]="0123456789ABCDEF";report[used++]=' ';for(int i=7;i>=0;--i)report[used++]=digits[(v>>(i*4))&15U];}

void perf_monitor_init(perf_write_t write)
{writer=write;phase=0;index=0;used=0;perf_monitor_pending=false;available=read_reg(0)==UINT32_C(0x50524631);if(available)write_reg(0x1c,1);}
void perf_monitor_queue(uint32_t queued)
{if(available)write_reg(0x10,queued);}
void perf_monitor_tx_drop(void)
{if(available)write_reg(0x18,1);}
void perf_monitor_request(void)
{if(available&&writer&&phase==0){phase=1;perf_monitor_pending=true;}}
/* 功能：申请视频诊断快照；参数无；返回无，不改变控制状态。 */
void perf_monitor_video(void)
{if(available&&writer&&phase==0){phase=5;perf_monitor_pending=true;}}
void perf_monitor_service(uint32_t now_ms)
{
    if(phase==0)return;
    if(phase==5){
        *(volatile uint32_t *)(uintptr_t)0x20007010=1;
        report[0]='V';report[1]='I';report[2]='D';report[3]='1';used=4;index=0;phase=6;return;
    }
    if(phase==6){
        hex_word(*(volatile uint32_t *)(uintptr_t)(0x20007020+4*index));
        if(++index==16){report[used++]='\r';report[used++]='\n';phase=4;}return;
    }
    if(phase==1){
        /* 网络时钟停止时旧请求可能仍忙；不叠加toggle，保证旧负载不会被误当新快照。 */
        if(!(read_reg(4)&1U))write_reg(0x14,1);
        start_ms=now_ms;phase=2;return;
    }
    if(phase==2){
        uint32_t status=read_reg(4);
        if((status&1U)&&(uint32_t)(now_ms-start_ms)<10U)return;
        report[0]='P';report[1]='E';report[2]='R';report[3]='F';report[4]='1';used=5;
        hex_word(status);index=0;phase=3;return;
    }
    if(phase==3){
        /* 序号、标称频率、8个系统字段、16个网络字段；一次循环只做一次总线读取。 */
        unsigned offset=index==0?8:index==1?12:index<10?0x20+(index-2)*4:0x40+(index-10)*4;
        hex_word(read_reg(offset));
        if(++index==26){report[used++]='\r';report[used++]='\n';phase=4;}
        return;
    }
    /* 满队列直接放弃诊断报告，不能因重试性能日志拖延或干扰正常控制。 */
    (void)writer(0,report,used);phase=0;perf_monitor_pending=false;
}
