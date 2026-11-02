#include "imu_display.h"
#include <string.h>
/* 功能：向指定行列写入文本；p非空，row<16、col<32、s非空；越过行尾截断，无返回。 */
static void put(char *p,unsigned row,unsigned col,const char *s)
{
    while(*s && col<32u) p[row*32u+col++]=*s++;
}
/* 功能：写8位十六进制无符号数；p至少8字节；v任意uint32；无返回。 */
static void hex8(char *p,uint32_t v)
{
    static const char digit[]="0123456789ABCDEF";
    for(unsigned i=0;i<8u;i++) p[i]=digit[(v>>(28u-4u*i))&15u];
}
/* 功能：写带符号的固定三位小数；p至少8字节；milli范围-500000..500000；无返回。 */
static void fixed3(char *p,int32_t milli)
{
    uint32_t n=(uint32_t)(milli<0?-milli:milli);
    p[0]=milli<0?'-':'+';p[4]='.';
    for(int i=7;i>=1;i--) if(i!=4) {p[i]=(char)('0'+n%10u);n/=10u;}
}
/* 功能：将原始计数转换为千分单位并四舍五入；raw为signed16，scale=6000/500000；返回千分单位。 */
static int32_t scaled(int16_t raw,int32_t scale)
{
    int64_t value=(int64_t)raw*scale;
    return (int32_t)((value+(value<0?-16384:16384))/32768);
}
/* 功能/参数/返回：见imu_display.h；无需printf、浮点格式库或动态分配。 */
void imu_format_screen(char text[IMU_TEXT_BYTES],const bmi088_t *d,
                       imu_screen_state_t state,uint32_t now_us,uint32_t overruns)
{
    static const char *const names[]={"INIT", "WAIT DRDY", "LIVE", "STALE", "ERROR"};
    memset(text,' ',IMU_TEXT_BYTES);
    put(text,0,0,"BMI088 SPI 1MHZ 200HZ");
    put(text,1,0,"STATE: ");put(text,1,7,names[state<=IMU_ERROR?state:IMU_ERROR]);
    put(text,2,0,"GYRO X:          DEG/S");
    put(text,3,0,"GYRO Y:          DEG/S");
    put(text,4,0,"GYRO Z:          DEG/S");
    put(text,5,0,"ACC  X:          G");
    put(text,6,0,"ACC  Y:          G");
    put(text,7,0,"ACC  Z:          G");
    for(unsigned i=0;i<6u;i++) {
        char *field=text+(i+2u)*32u+8u;
        if(state==IMU_LIVE) fixed3(field,scaled(i<3u?d->sample.gyro_raw[i]:d->sample.acc_raw[i-3u],i<3u?500000:6000));
        else memcpy(field,"---.----",8);
    }
    put(text,8,0,"SEQ A:");hex8(text+8u*32u+6u,d->sample.acc_seq);
    put(text,8,15,"G:");hex8(text+8u*32u+17u,d->sample.gyro_seq);
    put(text,9,0,"TIME MS:");hex8(text+9u*32u+8u,now_us/1000u);
    put(text,10,0,"ERROR:");hex8(text+10u*32u+6u,(uint32_t)(int32_t)d->error);
    put(text,11,0,"DRDY OVERRUN:");hex8(text+11u*32u+13u,overruns);
    put(text,12,0,"CHIP XYZ / UNCALIBRATED");
}
