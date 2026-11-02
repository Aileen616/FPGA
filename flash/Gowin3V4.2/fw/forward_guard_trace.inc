/* 781前进诊断：只记录首次异常，不改变采集结果、时效门槛或电机保护。 */
static uint32_t tof_error_status,tof_error_addr,tof_error_reg,tof_error_stage;
static uint32_t guard_bus_errors,guard_bus_addr,guard_bus_reg,guard_bus_stage,guard_bus_status,guard_bus_elapsed,guard_bus_max_us;
static uint32_t guard_mask,guard_stamp,guard_age,guard_seq,guard_dist,guard_validity,guard_error,guard_raw,guard_range,guard_phase;
static uint32_t guard_channel,guard_count,guard_wait,guard_clear,guard_pending,guard_poll_age;
/* 功能：清空诊断窗口；无参数/返回，仅停止态命令或获准新任务调用。 */
static void guard_reset(void)
{
    guard_mask=guard_count=0;guard_bus_errors=guard_bus_max_us=0;
    guard_bus_addr=guard_bus_reg=guard_bus_stage=guard_bus_status=guard_bus_elapsed=0;
}
/* 功能：检查前向首个异常；无参数/返回，不操作传感器或电机，不覆盖首次快照。 */
static void guard_observe(void)
{
    if(!tof_running||!vl53l1x_array_ready(&joint_tof))return;
    unsigned selected=bypass_tof_channel(planner.phase);
    /* 零输出换向期允许等待新探头，首次故障快照只抓运动段和静态前向异常。 */
    if(selected>=3||planner.phase==B_WAIT_FRONT||planner.phase==B_WAIT_SHIFT||planner.phase==B_WAIT_PASS||planner.phase==B_FAULT||planner.phase==B_DONE)return;
    const tof_sample_t *t=&joint_tof.sample[selected];
    if(!t->received)return;
    uint32_t now=rd(TOF_BASE,8),age=now-t->timestamp_us;
    unsigned mask=(t->validity!=SENSOR_VALID?1U:0U)|(t->error!=SENSOR_OK?2U:0U)|
        (t->range_status?4U:0U)|(age>=200000U?8U:0U)|(!t->distance_mm?16U:0U);
    if(!mask)return;
    ++guard_count;if(guard_mask)return;
    guard_channel=selected;guard_mask=mask;guard_stamp=now;guard_age=age;guard_seq=t->sequence;
    guard_dist=t->distance_mm;guard_validity=t->validity;guard_error=(uint32_t)(-(int)t->error);
    guard_raw=t->raw_status;guard_range=t->range_status;guard_phase=planner.phase;
    guard_wait=joint_tof.wait_ready_low[selected];guard_clear=joint_tof.clear_confirmed[selected];
    guard_pending=joint_tof.clear_still_ready[selected];guard_poll_age=now-joint_tof.last_poll_us[selected];
}
