/* 仅保存最近四次已消费的START入口尝试；重启清空，不补造旧版本历史。 */
typedef struct {
    uint32_t id,ms,mask,front,right,left,imu_ok,cal,fault,race,accepted;
} pad_start_record_t;
static pad_start_record_t pad_history[4]; /* 固定176字节，无堆，距离为探头原始mm。 */
static uint32_t pad_attempts; /* 启动尝试编号，极限饱和；槽位索引独立循环。 */
static unsigned pad_history_next,pad_history_count; /* 写入位置0..3，有效数量0..4。 */

/* 功能：记录按键时的输入；无参数；返回本次槽位供入口填写结果，永不为空。 */
static pad_start_record_t *pad_record_start(void)
{
    pad_start_record_t *row=&pad_history[pad_history_next];
    pad_history_next=(pad_history_next+1U)%4U;
    if(pad_history_count<4U)++pad_history_count;
    if(pad_attempts<UINT32_MAX)++pad_attempts;
    *row=(pad_start_record_t){.id=pad_attempts,.ms=rd(JDY_BASE,4),
        .front=joint_tof.sample[0].distance_mm,.right=joint_tof.sample[1].distance_mm,
        .left=joint_tof.sample[2].distance_mm,.imu_ok=imu_ready(),.cal=heading.calibrated,
        .fault=heading.fault,.race=irq_races};
    return row;
}

/* 功能：只读输出一条启动记录；index为0..3，0最新；无返回，不续期或改变电机状态。 */
static void pad_history_reply(unsigned index)
{
    char b[320],*p=text(b,"P3H V=1");p=field(p," SLOT=",index);
    bool exists=index<pad_history_count;p=field(p," EXISTS=",exists);
    if(exists){
        const pad_start_record_t *r=&pad_history[(pad_history_next+3U-index)%4U];
        p=field(p," ID=",r->id);p=field(p," MS=",r->ms);p=field(p," ACCEPT=",r->accepted);
        p=field(p," MASK=",r->mask);p=field(p," FRONT=",r->front);p=field(p," RIGHT=",r->right);
        p=field(p," LEFT=",r->left);p=field(p," IMUOK=",r->imu_ok);p=field(p," CAL=",r->cal);
        p=field(p," FAULT=",r->fault);p=field(p," RACE=",r->race);
    }
    p=text(p,"\r\n");*p=0;reply(b);
}

/* 功能：选择最近拒绝的首要提示；无参数；返回固定文本，完整原因仍在位图中。 */
static const char *pad_reject_text(void)
{
    if(start_reject_mask&SG_IMU)return "NOT READY / IMU CHECK";
    if(start_reject_mask&SG_FRONT_INVALID)return "NOT READY / FRONT INVALID";
    if(start_reject_mask&SG_RIGHT_INVALID)return "NOT READY / RIGHT INVALID";
    if(start_reject_mask&SG_LEFT_INVALID)return "NOT READY / LEFT INVALID";
    if(start_reject_mask&SG_FRONT_NEAR)return "NOT READY / DIRECTION CLEARANCE";
    if(start_reject_mask&SG_RIGHT_NEAR)return "NOT READY / RIGHT CLEARANCE";
    if(start_reject_mask&SG_LEFT_NEAR)return "NOT READY / LEFT CLEARANCE";
    return "NOT READY / CHECK STATE";
}
