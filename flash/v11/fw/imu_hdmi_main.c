#include "imu_display.h"
#include <stddef.h>
/* 仅当前IMU显示固件使用的Wishbone外设页；不接电机、不初始化ToF。 */
#define IMU_BASE UINT32_C(0x20002000)
static bmi088_t imu; /* 裸机主循环唯一所有者，XYZ为芯片坐标轴。 */
static char screen[IMU_TEXT_BYTES]; /* 静态字符缓冲，避免占用函数栈。 */
/* 功能：读取PL全字寄存器；off为映射内4字节对齐偏移；返回32位值。 */
static uint32_t rd(unsigned off)
{
    uint32_t v=*(volatile uint32_t *)(uintptr_t)(IMU_BASE+off);
    __asm__ volatile("fence iorw,iorw" ::: "memory");return v;
}
/* 功能：写PL全字寄存器；off为合法偏移，v任意32位；无返回。 */
static void wr(unsigned off,uint32_t v)
{
    __asm__ volatile("fence iorw,iorw" ::: "memory");
    *(volatile uint32_t *)(uintptr_t)(IMU_BASE+off)=v;
    __asm__ volatile("fence iorw,iorw" ::: "memory");
}
/* 功能：按sensor_bus契约执行模式0全双工事务；ctx可空，其余参数见sensor_bus.h。
 * 返回：0成功，-1参数/PL错误/超时；失败时通过ABORT释放两路CS。
 */
static int spi_transfer(void *ctx,uint8_t chip,const uint8_t *tx,uint8_t *rx,size_t n,uint32_t timeout_us)
{
    (void)ctx;
    if(!tx || !rx || chip>1u || n<1u || n>16u || !timeout_us) return -1;
    if(rd(0x0c)&1u) {wr(0x10,0x100);return -1;}
    for(unsigned i=0;i<4u;i++) {
        uint32_t word=0;
        for(unsigned j=0;j<4u;j++) if(4u*i+j<n) word|=(uint32_t)tx[4u*i+j]<<(8u*j);
        wr(0x20u+4u*i,word);
    }
    uint32_t since=rd(8);
    wr(0x10,1u|((uint32_t)chip<<1)|((uint32_t)(n-1u)<<4));
    for(;;) {
        uint32_t status=rd(0x0c);
        if(status&4u) {wr(0x10,0x100);return -1;}
        if((status&3u)==2u) break;
        if(sensor_elapsed(rd(8),since,timeout_us)) {wr(0x10,0x100);return -1;}
    }
    for(unsigned i=0;i<4u;i++) {
        uint32_t word=rd(0x30u+4u*i);
        for(unsigned j=0;j<4u;j++) if(4u*i+j<n) rx[4u*i+j]=(uint8_t)(word>>(8u*j));
    }
    return 0;
}
/* 功能：原子消费DRDY快照；无参数；返回真实边沿及时间，非同步六轴快照。 */
static bmi088_events_t take_events(void)
{
    bmi088_events_t ev;
    wr(0x40,1);ev.mask=(uint8_t)rd(0x40);ev.acc_us=rd(0x44);ev.gyro_us=rd(0x48);
    return ev;
}
/* 功能：初始化BMI088并以100ms周期发布HDMI文本；无参数；正常运行不返回。
 * INIT错误锁存，按板上复位重新初始化；读错误可在两路新数据均到达后恢复显示。
 */
int main(void)
{
    sensor_bus_t bus={0};bus.spi=spi_transfer;
    if(rd(0)!=UINT32_C(0x424d4901)) for(;;) {} /* 无匹配PL时，显示端自主显示NO FRESH DATA。 */
    bmi088_start(&imu,&bus,rd(8),100000);
    uint32_t last_display=rd(8)-100000u;
    bool initialized=false;
    for(;;) {
        uint32_t now=rd(8);
        if(!bmi088_ready(&imu)) bmi088_init_step(&imu,now);
        else {
            bmi088_events_t ev=take_events();
            if(!initialized) {initialized=true;ev.mask=0;} /* 丢弃初始化过程中的历史边沿。 */
            bmi088_read(&imu,&ev,rd(8));
        }
        now=rd(8);
        sensor_validity_t validity=bmi088_health(&imu,now);
        imu_screen_state_t state=IMU_INIT;
        if(imu.error<0) state=IMU_ERROR;
        else if(bmi088_ready(&imu)) state=validity==SENSOR_VALID?IMU_LIVE:validity==SENSOR_STALE?IMU_STALE:IMU_WAIT;
        if(sensor_elapsed(now,last_display,100000) && !(rd(0x50)&1u)) {
            imu_format_screen(screen,&imu,state,now,rd(0x4c));
            for(unsigned i=0;i<IMU_TEXT_BYTES;i+=4u) {
                uint32_t v=(uint8_t)screen[i]|((uint32_t)(uint8_t)screen[i+1]<<8)|
                    ((uint32_t)(uint8_t)screen[i+2]<<16)|((uint32_t)(uint8_t)screen[i+3]<<24);
                wr(0x100u+i,v);
            }
            wr(0x54,1);last_display=now;
        }
    }
}
