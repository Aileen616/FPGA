/* 迷宫开环执行器不消费编码器位置、速度或非法AB计数。 */
typedef struct {
    uint16_t forward_q8,right_q8,yaw_q8,limit;
    uint8_t verified,coverage_verified;
} maze_drive_config_t;
/* 系数单位为Q15输出/请求单位的Q8值，标定前全零；无串口解锁入口。
 * volatile保留执行路径，不能把尚未标定的请求速度当实测速度。 */
static volatile maze_drive_config_t maze_drive_config;
static int16_t maze_wheel[4]; /* FL/FR/RL/RR有符号Q15输出。 */
static uint32_t maze_commits,maze_started_ms;
/* 功能：检查标定和输出硬限；无参数；有效返回true，不写外设。 */
static bool maze_drive_ready(void)
{return maze_drive_config.verified&&maze_drive_config.forward_q8&&maze_drive_config.right_q8&&
 maze_drive_config.yaw_q8&&maze_drive_config.limit&&maze_drive_config.limit<=8192U;}
/* 功能：混合平移与航向请求；o不可空；无返回，四轮仅按有符号Q15接口范围处理。 */
static void maze_mix(const mt_output_t *o)
{
    int32_t limit=maze_trial?32767:maze_drive_config.limit;
    int32_t f=(int32_t)o->forward_mm_s*(maze_trial?23301:maze_drive_config.forward_q8)/256;
    int32_t l=-(int32_t)o->right_mm_s*(maze_trial?23301:maze_drive_config.right_q8)/256;
    /* 核心右转为正，原轮系混合器左转为正，必须在边界换号。 */
    int32_t t=-o->yaw_rate_md_s*(maze_trial?69:maze_drive_config.yaw_q8)/256;
    int32_t w[4]={f-l-t,f+l+t,f+l-t,f-l+t};uint32_t peak=0;
    for(unsigned i=0;i<4;i++)if(auto_abs(w[i])>peak)peak=auto_abs(w[i]);
    for(unsigned i=0;i<4;i++)maze_wheel[i]=(int16_t)(peak>(uint32_t)limit?
        (int64_t)w[i]*limit/peak:w[i]);
}
/* 功能：提交获准四轮帧并续本地租约；无参数；返回接受结果，不读取编码器。 */
static bool maze_commit(void)
{
    if(!maze_owned||(!maze_trial&&!maze_drive_ready())||hw_sequence==UINT32_MAX||local_lease==UINT32_MAX||
       rd(MOTOR_BASE,0x74)!=0||rd(MOTOR_BASE,12)!=0x5f)return false;
    for(unsigned i=0;i<4;i++)wr(MOTOR_BASE,0x50+4*canonical_bit[i],(uint32_t)(int32_t)maze_wheel[i]);
    wr(MOTOR_BASE,0x10,++hw_sequence);wr(MOTOR_BASE,0x14,UINT32_C(0x4d4f5633));
    if((rd(MOTOR_BASE,4)&3U)!=1U||rd(MOTOR_BASE,0x44)!=hw_sequence)return false;
    wr(JDY_BASE,0x10,++local_lease);++maze_commits;return (rd(JDY_BASE,8)&3U)==1U;
}
