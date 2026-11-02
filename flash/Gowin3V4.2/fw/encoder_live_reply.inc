/* 依赖tof5_main.c的MMIO、任务状态和文本应答；仅用于独立诊断候选。 */
/* 功能：停车时读取单轮当前编码器与驱动快照；channel为0..3，对应RR/FR/RL/FL；无返回。 */
static void encoder_live_reply(unsigned channel)
{
    /* 活动或标定期间拒绝查询，避免串口长应答干扰控制节拍。 */
    if((rd(MOTOR_BASE,4)&1U)||bypass_active(&planner)||robot_active(&robot)||
       legacy.active||pad_owner||pad_cal_stage||heading.calibrating){
        reply("ELV V=1 BUSY=1\r\n");return;
    }
    unsigned bit=channel_to_bit[channel]; /* 软件轮序转换为已核对的物理位。 */
    wr(MOTOR_BASE,8,8); /* 仅锁存快照，不清计数、不武装、不提交输出、不续期。 */
    uint32_t snapshot=rd(MOTOR_BASE,0xd0),pins=rd(MOTOR_BASE,0xdc);
    uint32_t position=rd(MOTOR_BASE,0x20+bit*4),illegal=rd(MOTOR_BASE,0x30+bit*4);
    int32_t speed=(int32_t)rd(MOTOR_BASE,0xc0+bit*4);
    int32_t effort=(int32_t)rd(MOTOR_BASE,0xe0+bit*4),target=(int32_t)rd(MOTOR_BASE,0xf0+bit*4);
    char b[384],*p=text(b,"ELV V=1 BUSY=0"); /* 最长整数应答小于384字节。 */
    p=field(p," CH=",channel);p=field(p," BIT=",bit);p=field(p," SNAP=",snapshot);
    p=field(p," MS=",rd(JDY_BASE,4));p=field(p," ACTIVE=",rd(MOTOR_BASE,4)&1U);
    p=field(p," CFG=",rd(MOTOR_BASE,12));p=field(p," MODE=",rd(MOTOR_BASE,0x74));
    p=field(p," DIAG=",rd(MOTOR_BASE,0xd4));p=field(p," RAW=",position);p=field(p," ILL=",illegal);
    p=text(p," CPS=");p=signed_number(p,speed);p=text(p," PWM=");p=signed_number(p,effort);
    p=text(p," TARGET=");p=signed_number(p,target);
    /* AB是另一同步链读取时的电平，不是同拍历史波形，也不是引脚电压测量。 */
    p=field(p," A=",(pins>>bit)&1U);p=field(p," B=",(pins>>(bit+4))&1U);
    p=field(p," READY=",(pins>>(bit+8))&1U);p=text(p,"\r\n");*p=0;reply(b);
}
