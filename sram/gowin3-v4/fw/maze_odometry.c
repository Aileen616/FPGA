#include "maze_odometry.h"
#include <limits.h>
#include <string.h>

/* 功能：计算有符号回绕计数差；a/b为32位计数；返回半周以内的有符号差。 */
static int64_t delta(uint32_t a,uint32_t b)
{
    uint32_t d=a-b;
    return d<=INT32_MAX?(int64_t)d:-(int64_t)(UINT32_MAX-d+1U);
}
/* 功能：定点计算航向正余弦；angle为毫度，s/c不可为空；返回无，结果为Q30。 */
static void rotation(int32_t angle,int32_t *s,int32_t *c)
{
    static const int32_t angles[16]={45000,26565,14036,7125,3576,1790,895,448,224,112,56,28,14,7,3,2};
    int32_t z=angle%360000,x=652032874,y=0;
    if(z>180000)z-=360000;
    if(z<-180000)z+=360000;
    if(z>90000){z-=180000;x=-x;}
    if(z<-90000){z+=180000;x=-x;}
    for(unsigned i=0;i<16;i++){
        int32_t old=x;
        if(z>=0){x-=y/(1<<i);y+=old/(1<<i);z-=angles[i];}
        else{x+=y/(1<<i);y-=old/(1<<i);z+=angles[i];}
    }
    *s=y;*c=x;
}
/* 功能：建立复用原系数的默认值；c可为空；返回无。 */
void mo_defaults(mo_config_t *c)
{if(c)for(unsigned i=0;i<4;i++)c->counts_per_m_milli[i]=5268945U;}
/* 功能：初始化旁路里程；o/c不可为空；返回参数有效性，不参与电机许可。 */
bool mo_init(mo_t *o,const mo_config_t *c)
{
    if(!o||!c)return false;
    memset(o,0,sizeof(*o));o->cfg=*c;
    for(unsigned i=0;i<4;i++)if(!c->counts_per_m_milli[i])return false;
    return true;
}
/* 功能：更新累计位置；o/in不可为空；返回是否消费新样本，异常只影响地图有效标记。 */
bool mo_update(mo_t *o,const mo_input_t *in)
{
    if(!o||!in)return false;
    if(!in->valid){o->valid=0;return false;}
    if(o->seen&&delta(in->seq,o->seq)<=0)return false;
    if(!o->seen){
        memcpy(o->previous,in->count,sizeof(o->previous));o->seq=in->seq;o->stamp_ms=in->stamp_ms;
        o->seen=o->valid=1;return true;
    }
    int64_t wheel[4];
    for(unsigned i=0;i<4;i++){
        if(!o->cfg.counts_per_m_milli[i]){o->valid=0;++o->rejected;return false;}
        int64_t numerator=delta(in->count[i],o->previous[i])*INT64_C(1000000000)+o->remainder[i];
        wheel[i]=numerator/o->cfg.counts_per_m_milli[i];
        o->remainder[i]=numerator%o->cfg.counts_per_m_milli[i];
    }
    memcpy(o->previous,in->count,sizeof(o->previous));o->seq=in->seq;o->stamp_ms=in->stamp_ms;
    int64_t fsum=wheel[0]+wheel[1]+wheel[2]+wheel[3]+o->forward_remainder;
    int64_t rsum=wheel[0]-wheel[1]-wheel[2]+wheel[3]+o->right_remainder;
    int64_t forward=fsum/4,right=rsum/4;
    o->forward_remainder=(int32_t)(fsum%4);o->right_remainder=(int32_t)(rsum%4);
    /* 表示范围检查只标记里程，不截断、停止或限幅运动。 */
    if(forward>INT32_MAX||forward<INT32_MIN||right>INT32_MAX||right<INT32_MIN){o->valid=0;++o->rejected;return false;}
    int32_t s,c;rotation(in->yaw_md,&s,&c);
    int64_t xsum=right*c+forward*s+o->rotate_remainder[0];
    int64_t ysum=forward*c-right*s+o->rotate_remainder[1];
    int64_t dx=xsum/INT64_C(1073741824),dy=ysum/INT64_C(1073741824);
    o->rotate_remainder[0]=xsum%INT64_C(1073741824);o->rotate_remainder[1]=ysum%INT64_C(1073741824);
    if(o->x_um+dx>(int64_t)INT32_MAX*1000||o->x_um+dx<(int64_t)INT32_MIN*1000||
       o->y_um+dy>(int64_t)INT32_MAX*1000||o->y_um+dy<(int64_t)INT32_MIN*1000){o->valid=0;++o->rejected;return false;}
    o->x_um+=dx;o->y_um+=dy;o->x_mm=(int32_t)(o->x_um/1000);o->y_mm=(int32_t)(o->y_um/1000);o->valid=1;
    return true;
}
