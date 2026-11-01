/* 功能：输出指定槽位的最新距离；slot=0/1/2对应前X2/右X3/左X4；无返回，不刷新运动租约。 */
static void range_reply(unsigned slot)
{
    char b[240],*p=b;const tof_sample_t *t=&joint_tof.sample[slot];
    p=text(p,"T3");p=field(p," CH=",slot+2U);p=field(p," DM=",TOF5_DISTANCE_MODE);
    p=field(p," MM=",t->distance_mm);p=field(p," VALID=",t->validity);
    p=field(p," STATUS=",t->range_status);p=field(p," RAW=",t->raw_status);
    p=field(p," SEQ=",t->sequence);p=field(p," AGE=",t->received?rd(TOF_BASE,8)-t->timestamp_us:UINT32_MAX);
    p=field(p," OK=",t->error==SENSOR_OK);p=text(p,"\r\n");*p=0;reply(b);
}
