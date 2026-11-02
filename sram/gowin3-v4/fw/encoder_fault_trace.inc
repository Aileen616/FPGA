/* 首次触发分支先停车，再冻结同一控制拍的反馈；读取记录不刷新运动租约。 */
enum { EF_ILLEGAL=1U,EF_STALL=2U,EF_REVERSE=4U,EF_RANGE=8U,EF_SPREAD=16U,EF_RETURN=32U,EF_UNKNOWN=64U };
typedef struct {
    uint32_t id,valid,mask,wheels,phase,ms,elapsed,invert,returned;
    int32_t requested;
    uint32_t position[4],baseline[4],illegal[4],illegal_base[4],idle_ms[4];
    int32_t delta[4],actual[4];
} encoder_fault_trace_t;
static encoder_fault_trace_t encoder_trace; /* 固定152字节；保留到下一次编码器故障或复位。 */

/* 功能：按本段方向归一编码器差值；v不可空，channel为0..3；返回有符号counts。 */
static int32_t encoder_delta_at(const tof5_input_t *v,unsigned channel)
{
    uint32_t raw=(uint32_t)v->position[channel]-mission.baseline[channel];
    if(mission.raw_invert&(1U<<channel))raw=0U-raw;
    return (int32_t)raw;
}

/* 功能：立即停车后冻结诊断；mask为触发位，wheels为通道位图，v不可空，effort为停机前指令；无返回。 */
static void encoder_stop_trace(uint32_t mask,uint32_t wheels,const tof5_input_t *v,int32_t effort)
{
    uint32_t phase=planner.phase;
    halt(T5_ENCODER); /* 先撤销输出，禁止为了记录或串口打印而延迟停车。 */
    if(encoder_trace.id<UINT32_MAX)++encoder_trace.id;
    encoder_trace.valid=1;encoder_trace.mask=mask;encoder_trace.wheels=wheels;
    encoder_trace.phase=phase;encoder_trace.ms=v->now_ms;
    encoder_trace.elapsed=v->now_ms-mission.started_ms;encoder_trace.invert=mission.raw_invert;
    encoder_trace.returned=right_settled_counts;encoder_trace.requested=effort;
    for(unsigned c=0;c<4;c++){
        unsigned bit=channel_to_bit[c];
        encoder_trace.position[c]=(uint32_t)v->position[c];encoder_trace.baseline[c]=mission.baseline[c];
        encoder_trace.delta[c]=encoder_delta_at(v,c);
        encoder_trace.illegal[c]=rd(MOTOR_BASE,0x30+bit*4);encoder_trace.illegal_base[c]=illegal_start[c];
        encoder_trace.idle_ms[c]=v->now_ms-stall_stamp[c];
        encoder_trace.actual[c]=(int32_t)rd(MOTOR_BASE,0xe0+bit*4);
    }
}

/* 功能：为已触发的原计数保护细分原因；v不可空，effort为停机前指令；无返回，不放宽原判据。 */
static void encoder_trace_count_fault(const tof5_input_t *v,int32_t effort)
{
    int32_t minimum=INT32_MAX,maximum=INT32_MIN;uint32_t mask=0,wheels=0;
    for(unsigned c=0;c<4;c++){
        int32_t delta=encoder_delta_at(v,c);
        if(delta<minimum)minimum=delta;
        if(delta>maximum)maximum=delta;
        if(delta< -100){mask|=EF_REVERSE;wheels|=1U<<c;}
        if(delta>40000){mask|=EF_RANGE;wheels|=1U<<c;}
    }
    if((int64_t)maximum-minimum>790){
        mask|=EF_SPREAD;
        for(unsigned c=0;c<4;c++){
            int32_t delta=encoder_delta_at(v,c);
            if(delta==minimum||delta==maximum)wheels|=1U<<c;
        }
    }
    encoder_stop_trace(mask?mask:EF_UNKNOWN,wheels,v,effort);
}
