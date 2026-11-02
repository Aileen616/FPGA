#include "ps2_control.h"
#include <string.h>
/* 功能：返回整数绝对值；v限于-32767..32767；返回非负值。 */
static int absolute(int v){return v<0?-v:v;}
/* 功能：摇杆死区和满量程映射；v为0..255，返回-100..100。 */
static int axis(unsigned v){int x=(int)v-128;if(absolute(x)<=20)return 0;return x>0?(x-20)*100/107:(x+20)*100/108;}
void ps2_control_init(ps2_control_t *s){memset(s,0,sizeof(*s));s->task=s->selected=1;}
bool ps2_control_update(ps2_control_t *s,uint32_t seq,uint32_t now,bool valid,uint16_t buttons,uint32_t axes,bool busy)
{
    s->stop=s->start=s->confirm=false;
    if(seq==s->sequence){
        if(s->linked&&(uint32_t)(now-s->stamp_ms)>=120){s->linked=false;s->neutral_seen=s->manual_armed=false;s->stop=true;}
        return false;
    }
    s->sequence=seq;s->stamp_ms=now;
    if(!valid){s->stop=s->linked;s->linked=s->neutral_seen=s->manual_armed=false;s->previous=buttons;return true;}
    uint16_t edge=buttons&~s->previous;
    if(!s->linked){s->linked=true;edge=0;s->neutral_seen=s->manual_armed=false;}
    uint16_t released=s->previous&~buttons;s->previous=buttons;
    int f=-axis(axes>>24),l=-axis((axes>>16)&255),r=-axis(axes&255);
    bool neutral=!f&&!l&&!r;
    s->forward=s->left=s->turn=0;
    if((buttons&PS_CROSS)||(edge&PS_SELECT)||(released&PS_L1)){
        s->stop=true;s->manual_armed=false;s->neutral_seen=false;s->gear=0;
        if(edge&PS_SELECT)s->page=0;
        return true;
    }
    if(edge&PS_TRIANGLE)s->page=1;
    if(edge&PS_SQUARE)s->page=2;
    if(edge&PS_L3)s->page=0;
    if(edge&PS_R3)s->page=3;
    if(!busy){
        if(edge&PS_UP)s->selected=s->selected==1?6:s->selected-1;
        if(edge&PS_DOWN)s->selected=s->selected==6?1:s->selected+1;
        if((edge&PS_L2)&&!(edge&PS_R2)&&s->gear)s->gear--;
        if((edge&PS_R2)&&!(edge&PS_L2)&&s->gear<2)s->gear++;
        if(edge&PS_CIRCLE){s->task=s->selected;s->confirm=true;s->manual_armed=false;}
        // START唤醒帧被消耗；只有释放再按且摇杆回中才产生启动事件。
        if((edge&PS_START)&&neutral&&!(buttons&PS_L1))s->start=true;
    }
    if(neutral&&!(buttons&PS_L1))s->neutral_seen=true;
    if((edge&PS_L1)&&neutral&&s->neutral_seen&&s->task==2&&!busy)s->manual_armed=true;
    if(s->manual_armed&&(buttons&PS_L1)&&s->task==2){
        /* 用户反馈能转但太慢：三档目标翻倍，R1仍保留100counts/s精细控制。 */
        int speed=(buttons&PS_R1)?100:(s->gear==0?400:s->gear==1?800:1200);
        int total=absolute(f)+absolute(l)+absolute(r);if(total<100)total=100;
        s->forward=(int16_t)(f*speed/total);s->left=(int16_t)(l*speed/total);s->turn=(int16_t)(r*speed/total);
    }
    return true;
}
