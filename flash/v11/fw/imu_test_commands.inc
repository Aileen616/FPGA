    /* 本段嵌入imu_command，复用其b/p/idle；测试身份包含是否通信锁存，便于直接辨识。 */
    if(!strcmp(line,"imu test")){
        p=imu_text(p,"ITEST V=1");p=imu_field(p," PROFILE=",IMU_TEST_STATIC?1:2);
        p=imu_field(p," BLOCK=",motor.blocked);p=imu_field(p," UNIT=",1);
        p=imu_field(p," KP=",4);p=imu_field(p," KD=",2);p=imu_field(p," LIMIT=",100);
        p=imu_field(p," SLEW=",2);imu_send(b,p);return;
    }
    if(!strcmp(line,"imu caldiag")){
        /* 只读保留失败窗口的极值；先定位静止判据，不能靠反复标定掩盖失败。 */
        p=imu_text(p,"ICAL");p=imu_field(p," COUNT=",(int32_t)heading.samples);
        p=imu_field(p," CAL=",heading.calibrated);p=imu_field(p," BUSY=",heading.calibrating);
        p=imu_field(p," FAULT=",heading.fault);
        p=imu_field(p," LX=",heading.lo[0]);p=imu_field(p," HX=",heading.hi[0]);
        p=imu_field(p," LY=",heading.lo[1]);p=imu_field(p," HY=",heading.hi[1]);
        p=imu_field(p," LZ=",heading.lo[2]);p=imu_field(p," HZ=",heading.hi[2]);
        p=imu_field(p," BX=",heading.bias_q16[0]);p=imu_field(p," BY=",heading.bias_q16[1]);
        p=imu_field(p," BZ=",heading.bias_q16[2]);imu_send(b,p);return;
    }
    if(!strcmp(line,"imu health")){
        p=imu_text(p,"IH");p=imu_field(p," N=",(int32_t)imu.gyro_seq);
        p=imu_field(p," T=",(int32_t)heading.stamp_us);p=imu_field(p," DT=",(int32_t)heading.dt_us);
        p=imu_field(p," OK=",imu.valid&&heading.feedback_valid&&(uint32_t)(rd(IMU_BASE,8)-heading.stamp_us)<20000);
        p=imu_field(p," ERR=",imu.error);p=imu_field(p," DROP=",(int32_t)imu.rejected_frames);
        p=imu_field(p," MISS=",(int32_t)(irq_missed_acc+irq_missed_gyro));p=imu_field(p," RACE=",(int32_t)irq_races);
        p=imu_field(p," Y=",(int32_t)(heading.yaw_q16_md/65536));p=imu_field(p," R=",heading.rate_mdps);
        p=imu_field(p," U=",heading.correction);p=imu_field(p," RUN=",heading.running);
        p=imu_field(p," FAULT=",heading.fault);imu_send(b,p);return;
    }
