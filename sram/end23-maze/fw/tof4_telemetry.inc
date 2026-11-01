/* 功能：只读输出一通道完整诊断；不续期、不切换运动模式、不访问I2C。
 * 参数：i为0前/1右/2左/3后；返回：无。MM为原始距离，必须同时检查VALID与AGE。
 */
static void tof4_read_reply(unsigned i)
{
    if(i>=4)return;
    const tof_sample_t *t=&joint_tof.sample[i];
    uint32_t now=rd(TOF_BASE,8),age=t->received?now-t->timestamp_us:UINT32_MAX;
    bool valid=i<joint_tof.cfg.count&&tof_running&&vl53l1x_array_ready(&joint_tof)&&t->received&&
        t->validity==SENSOR_VALID&&t->error==SENSOR_OK&&t->range_status==0&&age<joint_tof.cfg.fresh_us;
    char b[512],*p=text(b,"T4 V=3");
    p=field(p," CH=",i);p=field(p," X=",i==3?1:i+2);p=field(p," ADDR=",joint_tof.cfg.address7[i]);
    p=field(p," MM=",t->distance_mm);p=field(p," VALID=",valid);p=field(p," SEQ=",t->sequence);
    p=field(p," AGE=",age);p=field(p," RAW=",t->raw_status);p=field(p," RANGE=",t->range_status);
    p=text(p," ERR=");p=signed_number(p,t->error);p=field(p," STREAM=",t->stream_count);
    p=field(p," POLLS=",tof_poll_count[i]);p=field(p," GAP=",tof_poll_max_us[i]);
    p=field(p," POLLAGE=",now-joint_tof.last_poll_us[i]);p=field(p," CLEAR=",joint_tof.clear_confirmed[i]);
    p=field(p," WAIT=",joint_tof.wait_ready_low[i]);p=field(p," PENDING=",joint_tof.clear_still_ready[i]);
    p=field(p," MAXAGE=",joint_tof.max_age_us[i]);p=field(p," PGAP=",joint_tof.max_publish_gap_us[i]);
    p=field(p," STALES=",joint_tof.stale_events[i]);
    p=field(p," BUSERR=",guard_bus_errors);p=field(p," ACTIVE=",rd(MOTOR_BASE,4)&1U);
    p=text(p,"\r\n");*p=0;reply(b);
}
