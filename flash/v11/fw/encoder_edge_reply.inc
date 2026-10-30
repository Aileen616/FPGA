/* 依赖tof5_main.c中的状态、只读MMIO及文本输出；与边沿诊断RTL配套。 */
/* 功能：停车时读取首次非法边沿的冻结记录；channel为0..3，对应RR/FR/RL/FL；无返回。 */
static void encoder_edge_reply(unsigned channel)
{
    if((rd(MOTOR_BASE,4)&1U)||bypass_active(&planner)||robot_active(&robot)||
       legacy.active||pad_owner||pad_cal_stage||heading.calibrating){
        reply("EED V=1 BUSY=1\r\n");return;
    }
    unsigned bit=channel_to_bit[channel]; /* 映射到已核对的物理位，不能直接使用软件轮序寻址。 */
    uint32_t off=0x100U+bit*64U;
    char b[384],*p=text(b,"EED V=1 BUSY=0"); /* 最长十进制字段应答小于384字节。 */
    p=field(p," CH=",channel);p=field(p," BIT=",bit);
    p=field(p," META=",rd(MOTOR_BASE,off));
    p=field(p," TICK=",rd(MOTOR_BASE,off+4));
    p=field(p," ACCEPTAGE=",rd(MOTOR_BASE,off+8));
    p=field(p," SAMPLEAGE=",rd(MOTOR_BASE,off+12));
    p=field(p," PWMAGE=",rd(MOTOR_BASE,off+16));
    p=text(p," EFFORT=");p=signed_number(p,(int32_t)rd(MOTOR_BASE,off+20));
    p=field(p," ILL0=",rd(MOTOR_BASE,off+24));
    p=field(p," PREVDUR=",rd(MOTOR_BASE,off+28));
    p=field(p," EDGE=",rd(MOTOR_BASE,off+32));
    p=text(p,"\r\n");*p=0;reply(b);
}
