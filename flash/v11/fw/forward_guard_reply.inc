/* 功能：输出固定容量的首次异常快照；无参数/返回，不清记录、不续期运动。 */
static void guard_reply(void)
{
    char b[512],*p=b;p=text(p,"G783");p=field(p," CHANNEL=",guard_channel);
    p=field(p," MASK=",guard_mask);p=field(p," T=",guard_stamp);p=field(p," AGE=",guard_age);
    p=field(p," SEQ=",guard_seq);p=field(p," MM=",guard_dist);p=field(p," VAL=",guard_validity);
    p=field(p," ERRN=",guard_error);p=field(p," RAW=",guard_raw);p=field(p," RANGE=",guard_range);
    p=field(p," PHASE=",guard_phase);p=field(p," COUNT=",guard_count);
    p=field(p," WAIT=",guard_wait);p=field(p," CLEAR=",guard_clear);p=field(p," PENDING=",guard_pending);
    p=field(p," POLLAGE=",guard_poll_age);p=field(p," BUSERR=",guard_bus_errors);
    p=field(p," ADDR=",guard_bus_addr);p=field(p," REG=",guard_bus_reg);p=field(p," STAGE=",guard_bus_stage);
    p=field(p," STATUS=",guard_bus_status);p=field(p," ELAPSED=",guard_bus_elapsed);p=field(p," MAXUS=",guard_bus_max_us);
    p=text(p,"\r\n");*p=0;reply(b);
}
