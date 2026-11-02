#ifndef PS2_CONTROL_H
#define PS2_CONTROL_H
#include <stdint.h>
#include <stdbool.h>
enum {PS_SELECT=1,PS_L3=2,PS_R3=4,PS_START=8,PS_UP=16,PS_RIGHT=32,
      PS_DOWN=64,PS_LEFT=128,PS_L2=256,PS_R2=512,PS_L1=1024,PS_R1=2048,
      PS_TRIANGLE=4096,PS_CIRCLE=8192,PS_CROSS=16384,PS_SQUARE=32768};
typedef struct {
    uint32_t sequence,stamp_ms; /* 已消费快照与毫秒时刻，支持计时回绕。 */
    uint16_t previous; /* 上一有效帧的高有效按键。 */
    uint8_t task,selected,gear,page,result; /* 任务1..6；档位0..2；结果0待机/1拒绝/2运行。 */
    bool linked,neutral_seen,manual_armed,stop,start,confirm;
    int16_t forward,left,turn; /* 归一化目标，三轴绝对值之和不超过1200 counts/s；各轮组合亦不超过此值。 */
} ps2_control_t;
/* 功能：初始化交互状态；s非空；无返回，默认任务1和低档。 */
void ps2_control_init(ps2_control_t *s);
/* 功能：消费一个原子快照；s非空，seq序号、now毫秒、valid协议有效、buttons高有效、axes字节RX/RY/LX/LY；返回是否新快照。 */
bool ps2_control_update(ps2_control_t *s,uint32_t seq,uint32_t now,bool valid,uint16_t buttons,uint32_t axes,bool busy);
#endif
