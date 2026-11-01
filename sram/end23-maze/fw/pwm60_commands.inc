    /* 独立PROFILE3专用；普通手机key仍须先有本次显式航向授权才能产生输出。 */
    if(!strcmp(line,"imu hold 1")){
        if(!idle||motor.blocked||motor.closed_loop||rd(MOTOR_BASE,0x74)!=0||
           !motor.configured||rd(MOTOR_BASE,0x0c)!=0x5f||!imu.valid||
           !heading.calibrated||!heading.axis_confirmed||heading.axis!=2||heading.sign!=1||
           heading.fault||heading.calibrating){imu_reject("NOT_READY");return;}
        /* 编码器极性仅在停止态设置，不需要开启速度PI。 */
        /* 寄存器按RTL位3/2/4/1，左侧通道3/4对应bit0/bit2；协议回读才是ENCINV12。 */
        wr(MOTOR_BASE,0x78,5);if(rd(MOTOR_BASE,0x78)!=5){imu_reject("ENCODER");return;}
        pwm60_done=false;pwm60_reason=0;pwm60_armed=true;heading.enabled=true;
        for(unsigned c=0;c<4;c++)pwm60_counts[c]=0;
        p=imu_text(p,"IMU HOLD=1");imu_send(b,p);return;
    }
    if(!strcmp(line,"imu pwm60")){
        p=imu_text(p,"P60 V=1");p=imu_field(p," ARM=",pwm60_armed);p=imu_field(p," DONE=",pwm60_done);
        p=imu_field(p," WHY=",(int32_t)pwm60_reason);p=imu_field(p," N=",(int32_t)pwm60_feedback.sequence);
        const char *dl[4]={" D1="," D2="," D3="," D4="};
        const char *sl[4]={" S1="," S2="," S3="," S4="};
        const char *pl[4]={" P1="," P2="," P3="," P4="};
        for(unsigned c=0;c<4;c++)p=imu_field(p,dl[c],pwm60_counts[c]);
        for(unsigned c=0;c<4;c++)p=imu_field(p,sl[c],pwm60_feedback.cps[c]);
        for(unsigned c=0;c<4;c++)p=imu_field(p,pl[c],pwm60_feedback.effort[c]);
        p=imu_field(p," U=",straight.ratio);imu_send(b,p);return;
    }
