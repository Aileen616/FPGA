#include "robot_mission.h"
#include <string.h>
/* 功能：安全取绝对值；x不为INT32_MIN；返回非负值。 */
static int32_t mag(int32_t x){return x<0?-x:x;}
/* 功能：归一化角度；x范围±720000毫度；返回[-180000,180000)。 */
static int32_t wrap(int32_t x){while(x>=180000)x-=360000;while(x< -180000)x+=360000;return x;}
/* 功能：限制整数；x任意、limit正数；返回±limit内的值。 */
static int32_t limit(int32_t x,int32_t n){return x>n?n:x< -n?-n:x;}
/* 功能/参数/返回：见头文件；定长CORDIC避免浮点库和动态内存。 */
int32_t robot_atan2(int32_t y,int32_t x)
{
    static const int32_t a[16]={45000,26565,14036,7125,3576,1790,895,448,224,112,56,28,14,7,3,2};
    if(!x&&!y)return 0;
    if(!x)return y>0?90000:-90000;
    if(!y)return x>0?0:-180000;
    int32_t z=0;if(x<0){z=y>=0?180000:-180000;x=-x;y=-y;}
    x*=512;y*=512;
    for(unsigned i=0;i<16;i++){int32_t old=x;if(y>0){x+=y>>i;y-=old>>i;z+=a[i];}else if(y<0){x-=y>>i;y+=old>>i;z-=a[i];}}
    return wrap(z);
}
/* 功能/参数/返回：见头文件。 */
uint32_t robot_sqrt(uint32_t n)
{uint32_t r=0,b=1U<<30;while(b>n)b>>=2;while(b){if(n>=r+b){n-=r+b;r=(r>>1)+b;}else r>>=1;b>>=2;}return r;}
/* 功能：求旋转矩阵Q15；angle为±180000毫度，c/s不可空；无返回。 */
static void rotation(int32_t angle,int32_t *c,int32_t *s)
{
    static const int32_t a[15]={45000,26565,14036,7125,3576,1790,895,448,224,112,56,28,14,7,3};
    int32_t x=19898,y=0,z=wrap(angle),sign=1;
    if(z>90000){z-=180000;sign=-1;}if(z< -90000){z+=180000;sign=-1;}
    for(unsigned i=0;i<15;i++){int32_t old=x;if(z>=0){x-=y>>i;y+=old>>i;z-=a[i];}else{x+=y>>i;y-=old>>i;z+=a[i];}}
    *c=x*sign;*s=y*sign;
}
/* 功能/参数/返回：见头文件。 */
bool robot_active(const robot_mission_t *s){return s->mode>=R_MANUAL&&s->mode<=R_SEEK;}
/* 功能/参数/返回：见头文件。 */
void robot_init(robot_mission_t *s){memset(s,0,sizeof(*s));s->auto_slow_mm=200;s->auto_clear_mm=230;}
/* 功能/参数/返回：见头文件。 */
void robot_stop(robot_mission_t *s,robot_fault_t reason)
{s->mode=reason==R_OK?R_DONE:reason==R_CANCEL?R_IDLE:R_FAULT;s->fault=reason;s->forward=s->left=s->turn=0;for(unsigned i=0;i<4;i++)s->wheel_cps[i]=0;}
/* 功能/参数/返回：见头文件。 */
bool robot_geometry(robot_mission_t *s,const uint16_t off[4])
{if(robot_active(s))return false;for(unsigned i=0;i<4;i++)if(off[i]>1000)return false;memcpy(s->offsets,off,sizeof(s->offsets));s->configured=true;return true;}
/* 功能/参数/返回：见头文件。 */
bool robot_point(robot_mission_t *s,int32_t x,int32_t y)
{if(robot_active(s)||s->point_count>=8||x< -6000||x>6000||y< -6000||y>6000)return false;s->points[s->point_count++]=(robot_point_t){x,y};return true;}
/* 功能/参数/返回：见头文件。 */
bool robot_begin(robot_mission_t *s,const robot_input_t *in,robot_mode_t mode,uint32_t seq)
{
    if(robot_active(s)||!s->configured||!in->imu_valid||!seq||seq<=s->last_seq||
       (mode!=R_MANUAL&&mode!=R_ROUTE&&mode!=R_SEEK)||(mode==R_ROUTE&&!s->point_count))return false;
    if(mode==R_MANUAL&&(mag(s->manual_f)+mag(s->manual_l)+mag(s->manual_r)>600||s->manual_ms<200||s->manual_ms>2000))return false;
    for(unsigned i=0;i<4;i++)s->previous_encoder[i]=in->encoder[i];
    s->mode=mode;s->fault=R_OK;s->last_seq=seq;s->started_ms=s->renewed_ms=s->phase_ms=in->now_ms;
    s->x_q16=s->y_q16=0;s->reference_md=in->yaw_md;s->point_index=0;s->have_pose=true;
    s->target_valid=false;s->target_seq=0;s->clear_count=0;s->target_associated=false;s->target_seen=false;
    return true;
}
/* 功能/参数/返回：见头文件。 */
bool robot_keep(robot_mission_t *s,uint32_t seq,uint32_t now)
{if(!robot_active(s)||seq<=s->last_seq||!seq||(uint32_t)(now-s->renewed_ms)>=250)return false;s->last_seq=seq;s->renewed_ms=now;return true;}
/* 功能/参数/返回：见头文件。 */
bool robot_target(robot_mission_t *s,uint32_t seq,uint32_t now,bool valid,int32_t error,bool associated)
{if(s->mode!=R_SEEK||!seq||seq<=s->target_seq||error< -1000||error>1000)return false;s->target_seq=seq;s->target_ms=now;s->target_valid=valid;s->target_error=error;s->target_associated=valid&&associated;if(valid)s->target_seen=true;return true;}
/* 功能：计算车体边缘净距；s/in不可空，i=0..3；返回mm，负净距钳0。 */
static uint32_t clearance(const robot_mission_t *s,const robot_input_t *in,unsigned i)
{return in->range_mm[i]>s->offsets[i]?in->range_mm[i]-s->offsets[i]:0;}
/* 功能：把单轴车体运动映射到探头；f/l至少一个非0；返回前0/右1/左2/后3。 */
static unsigned direction(int32_t f,int32_t l){return f>0?0:f<0?3:l<0?1:2;}
/* 功能：更新里程；s/in不可空，单步每轮变化不得超过500counts；返回有效。 */
static bool odometry(robot_mission_t *s,const robot_input_t *in)
{
    int32_t d[4],c,t;for(unsigned i=0;i<4;i++){d[i]=(int32_t)(in->encoder[i]-s->previous_encoder[i]);s->previous_encoder[i]=in->encoder[i];if(d[i]< -500||d[i]>500)return false;}
    /* 正向已在平台按真实编码器极性修正；按麦克纳姆逆运动学求车体增量。 */
    int32_t f=d[0]+d[1]+d[2]+d[3],l=-d[0]+d[1]+d[2]-d[3];
    rotation(wrap(in->yaw_md-s->reference_md),&c,&t);
    s->x_q16+=((int64_t)f*c-(int64_t)l*t)*500000/5268945;
    s->y_q16+=((int64_t)f*t+(int64_t)l*c)*500000/5268945;
    return s->x_q16>=-INT64_C(20000)*65536&&s->x_q16<=INT64_C(20000)*65536&&s->y_q16>=-INT64_C(20000)*65536&&s->y_q16<=INT64_C(20000)*65536;
}
/* 功能：改变绕障阶段；s/in不可空、mode有效；无返回，阶段切换留200ms静止窗。 */
static void phase(robot_mission_t *s,const robot_input_t *in,robot_mode_t mode)
{s->mode=mode;s->phase_ms=in->now_ms;s->clear_count=0;s->clear_seq=0;s->forward=s->left=0;}
/* 功能：累计连续不同帧的净空；s/in不可空、i为测距索引；返回是否连续3帧满足距离。 */
static bool clear_frames(robot_mission_t *s,const robot_input_t *in,unsigned i,uint32_t mm)
{
    if(!in->range_valid[i]||clearance(s,in,i)<mm){s->clear_count=0;return false;}
    if(in->range_seq[i]!=s->clear_seq){s->clear_seq=in->range_seq[i];if(s->clear_count<3)s->clear_count++;}
    return s->clear_count>=3;
}
/* 功能/参数/返回：见头文件；所有输入决策在本地完成，路径模式无需外部续租。 */
void robot_step(robot_mission_t *s,const robot_input_t *in)
{
    if(!robot_active(s))return;
    s->forward=s->left=s->turn=0;
    if(!in->imu_valid){robot_stop(s,R_STALE);return;}
    if(!in->drive_ok||!odometry(s,in)){robot_stop(s,R_DRIVE);return;}
    if((uint32_t)(in->now_ms-s->started_ms)>=180000){robot_stop(s,R_TIMEOUT);return;}
    if(s->mode==R_MANUAL&&(uint32_t)(in->now_ms-s->renewed_ms)>=250){robot_stop(s,R_STALE);return;}
    int32_t x=(int32_t)(s->x_q16/65536),y=(int32_t)(s->y_q16/65536);
    int32_t yaw=wrap(in->yaw_md-s->reference_md);
    if(s->mode!=R_MANUAL&&s->mode!=R_SEEK&&mag(yaw)>15000){robot_stop(s,R_HEADING);return;}
    if(s->mode==R_MANUAL){
        if((uint32_t)(in->now_ms-s->started_ms)>=s->manual_ms){robot_stop(s,R_OK);return;}
        s->forward=s->manual_f;s->left=s->manual_l;s->turn=s->manual_r;
    }else if(s->mode==R_SEEK){
        if((uint32_t)(in->now_ms-s->started_ms)>=30000){robot_stop(s,R_TIMEOUT);return;}
        if(!s->target_valid||(uint32_t)(in->now_ms-s->target_ms)>=300){
            s->turn=0; /* 目标丢失由公共策略限时等待；不盲目旋转搜索。 */
        }else if(mag(s->target_error)>100){s->clear_count=0;s->turn=limit(-s->target_error/3,120);}
        else if(s->target_associated){
            if(!in->range_valid[0]){robot_stop(s,R_STALE);return;}
            if(clearance(s,in,0)<=500){
                if(in->range_seq[0]!=s->clear_seq){s->clear_seq=in->range_seq[0];s->clear_count++;}
                if(s->clear_count>=3){robot_stop(s,R_OK);return;}
            }else{s->clear_count=0;s->forward=200;}
        }else s->clear_count=0;
    }else if(s->mode==R_ROUTE){
        robot_point_t p=s->points[s->point_index];
        if(mag(p.x_mm-x)<=25&&mag(p.y_mm-y)<=25){if(++s->point_index==s->point_count){robot_stop(s,R_OK);return;}phase(s,in,R_ROUTE);}
        else if((uint32_t)(in->now_ms-s->phase_ms)>=200){
            if(mag(p.x_mm-x)>25)s->forward=limit((p.x_mm-x)*3,263);
            else s->left=limit((p.y_mm-y)*3,263);
            unsigned ahead=direction(s->forward,s->left);
            if(in->range_valid[ahead]&&clearance(s,in,ahead)<s->auto_slow_mm){
                int32_t f=s->forward>0?1:s->forward<0?-1:0,l=s->left>0?1:s->left<0?-1:0;
                unsigned ri=direction(l,-f),li=direction(-l,f);
                bool right=in->range_valid[ri]&&clearance(s,in,ri)>s->auto_clear_mm,left=in->range_valid[li]&&clearance(s,in,li)>s->auto_clear_mm;
                if(!right&&!left){s->forward=s->left=0;return;}
                s->bypass_sign=right?1:-1;s->bypass_f=f;s->bypass_l=l;s->bypass_x=x;s->bypass_y=y;phase(s,in,R_SHIFT);
            }
        }
    }else{
        int32_t df=s->bypass_l*s->bypass_sign,dl=-s->bypass_f*s->bypass_sign;
        int32_t side=(x-s->bypass_x)*df+(y-s->bypass_y)*dl,progress=(x-s->bypass_x)*s->bypass_f+(y-s->bypass_y)*s->bypass_l;
        if((uint32_t)(in->now_ms-s->phase_ms)>=60000||mag(side)>1500||mag(progress)>2000){robot_stop(s,R_TIMEOUT);return;}
        if((uint32_t)(in->now_ms-s->phase_ms)>=200){
            if(s->mode==R_SHIFT){
                if(side>=450&&clear_frames(s,in,direction(s->bypass_f,s->bypass_l),s->auto_clear_mm))phase(s,in,R_PASS);
                else {s->forward=df*200;s->left=dl*200;}
            }else if(s->mode==R_PASS){
                if(progress>=450&&clear_frames(s,in,direction(-df,-dl),s->auto_clear_mm))phase(s,in,R_REJOIN);
                else {s->forward=s->bypass_f*200;s->left=s->bypass_l*200;}
            }else if(mag(side)<=25)phase(s,in,R_ROUTE);
            else {int32_t speed=limit(-side*3,200);s->forward=df*speed;s->left=dl*speed;}
        }
    }
    /* 先按运动方向检查真实新鲜样本，再减速；转动要求四向均有效，避免侧面扫碰。 */
    for(unsigned i=0;i<4&&s->mode==R_MANUAL;i++){
        bool needed=i==0?s->forward>0:i==1?s->left<0:i==2?s->left>0:s->forward<0;
        if(s->turn)needed=true;
        if(!needed)continue;
        if(!in->range_valid[i]){robot_stop(s,R_STALE);return;}
        uint32_t mm=clearance(s,in,i);
        if(mm<=300){robot_stop(s,R_BLOCKED);return;}
        if(mm<800){s->forward=limit(s->forward,130);s->left=limit(s->left,130);s->turn=limit(s->turn,80);}
    }
    if(s->mode!=R_MANUAL&&s->mode!=R_SEEK&&(s->forward||s->left))s->turn=limit((-yaw*4-in->rate_mdps)/1000,70);
    s->wheel_cps[0]=s->forward-s->left-s->turn;s->wheel_cps[1]=s->forward+s->left+s->turn;
    s->wheel_cps[2]=s->forward+s->left-s->turn;s->wheel_cps[3]=s->forward-s->left+s->turn;
}
