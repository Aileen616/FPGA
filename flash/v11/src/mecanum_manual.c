#include "mecanum_manual.h"
#include <stddef.h>

bool mecanum_manual_mix(uint32_t key,uint32_t level,mecanum_output_t *output)
{
    /* 三档为12.5%/37.5%/50%的PWM上限；新高档尚待实测，不等于实测轮速。 */
    static const uint16_t limits[3]={4096U,12288U,16384U};
    static const mecanum_command_t commands[10]={
        {1000,0,0},{-1000,0,0},{0,1000,0},{0,-1000,0},{0,0,1000},{0,0,-1000},
        {1000,1000,0},{1000,-1000,0},{-1000,1000,0},{-1000,-1000,0}
    };
    if(output==NULL)return false;
    for(unsigned i=0;i<MECANUM_WHEEL_COUNT;i++)output->effort[i]=0;
    if(key<1||key>10||level<1||level>3)return false;
    const mecanum_config_t config={limits[level-1],0};
    return mecanum_mix(&config,&commands[key-1],output);
}
