/* 独立PWM60工程内部模块；依赖本文件上方的rd/wr、heading、straight、motor及通道表。
 * 基础幅值19660约为60%，纠偏后17694..21626；目标15807counts约为3m。
 * 本模块不供原低速工程包含，不修改低速工程的50%硬限制。 */
static bool snapshot(void *ctx,jdy_motor_snapshot_t *out);
/* 功能：按物理通道提交四轮PWM；step仅10ms控制节拍为真；成功true，失效false。 */
static bool pwm60_commit(bool step)
{
    int32_t unused[4];
    if(!straight_targets(&straight,&heading,step,unused)||hardware_seq==UINT32_MAX)return false;
    for(unsigned c=0;c<4;c++){
        int32_t value=19660*(1000+(c<2?straight.ratio:-straight.ratio))/1000;
        if(value<17694||value>21626)return false;
        wr(MOTOR_BASE,0x50+channel_to_bit[c]*4,(uint32_t)value);
    }
    uint32_t seq=++hardware_seq;wr(MOTOR_BASE,0x10,seq);wr(MOTOR_BASE,0x14,UINT32_C(0x4d4f5633));
    return (rd(MOTOR_BASE,4)&3U)==1U&&rd(MOTOR_BASE,0x44)==seq;
}
/* 功能：本地撤销运动并保留原因；reason为1距离、2超时、3编码器、4IMU/提交故障；无返回。 */
static void pwm60_halt(uint32_t reason)
{
    pwm60_done=true;pwm60_reason=reason;pwm60_armed=false;
    jdy_motor_rx(&motor,'~',rd(JDY_BASE,4));
    /* 即便通信已锁存也直接撤销硬件输出，不能依赖协议解析器处理停止字符。 */
    wr(MOTOR_BASE,0x08,4);wr(IMU_BASE,0x60,0);heading.running=false;heading.enabled=false;
}
/* 功能：10ms本地计数/故障/最长30秒检查；now为当前微秒模计数；无返回，必要时直接停车。 */
static void pwm60_tick(uint32_t now)
{
    if(!snapshot(0,&pwm60_feedback)){pwm60_halt(3);return;}
    if(pwm60_feedback.diagnostics&UINT32_C(0x00ffff00)){pwm60_halt(3);return;}
    bool reached=false;
    for(unsigned c=0;c<4;c++){
        int32_t delta=(int32_t)((uint32_t)pwm60_feedback.position[c]-(uint32_t)pwm60_origin[c]);
        int64_t distance=c>=2?-(int64_t)delta:delta;
        if(distance<0||distance>100000||pwm60_feedback.illegal[c]!=pwm60_illegal[c]){
            pwm60_halt(3);return;
        }
        pwm60_counts[c]=(int32_t)distance;
        if(distance>=15807)reached=true;
        /* 起步一秒后每轮必须有进展；慢轮不能靠其他轮的计数掩盖。 */
        if((uint32_t)(now-pwm60_start_us)>=1000000&&distance<20){pwm60_halt(3);return;}
    }
    if(reached){pwm60_halt(1);return;}
    if((uint32_t)(now-pwm60_start_us)>=30000000){pwm60_halt(2);return;}
    if(!imu.valid||heading.fault||!pwm60_commit(true))pwm60_halt(4);
}
/* 功能：仅全新前进手势允许启动或续租；ctx可空、seq新鲜、key/level必须1；成功true。 */
static bool drive(void *ctx,uint32_t seq,uint32_t key,uint32_t level)
{
    (void)ctx;
    if(!pwm60_armed||pwm60_done||key!=1||level!=1||!seq||motor.blocked||heading.calibrating||
       !heading.enabled||!imu.valid||heading.fault||motor.closed_loop||rd(MOTOR_BASE,0x74)!=0||
       (rd(JDY_BASE,8)&3U)!=1U)return false;
    if(!heading.running){
        if(!snapshot(0,&pwm60_feedback)||pwm60_feedback.diagnostics&UINT32_C(0x00ffff00))return false;
        for(unsigned c=0;c<4;c++){
            if(pwm60_feedback.cps[c]||pwm60_feedback.effort[c]||pwm60_feedback.target[c])return false;
            pwm60_origin[c]=pwm60_feedback.position[c];pwm60_illegal[c]=pwm60_feedback.illegal[c];pwm60_counts[c]=0;
        }
        const int32_t unit_base[4]={223,223,223,223};
        if(!straight_begin(&straight,&heading,unit_base))return false;
        wr(IMU_BASE,0x60,1);if(rd(IMU_BASE,0x60)!=1)return false;
        pwm60_start_us=last_heading_us=rd(IMU_BASE,8);
    }
    if(!pwm60_commit(false))return false;
    wr(JDY_BASE,0x10,seq);return (state(0)&3U)==1U;
}
