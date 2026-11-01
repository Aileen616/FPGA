/* 功能：追加有符号十进制；p至少12字节，n任意int32；返回末端。 */
static char *signed_number(char *p,int32_t n)
{if(n<0){*p++='-';return decimal(p,0U-(uint32_t)n);}return decimal(p,(uint32_t)n);}
/* 功能：处理IMU白名单；s不可空且NUL结尾；无返回，初始化和标定只能停机执行。 */
static void imu_command(const char *s)
{
    char b[512],*p=b;
    if(!strcmp(s,"imu")||!strcmp(s,"imu health")){
        p=text(p,"I783");p=field(p," OK=",imu_ready());p=field(p," VALID=",imu.valid);
        p=text(p," ERR=");p=signed_number(p,imu.error);p=field(p," CAL=",heading.calibrated);
        p=field(p," BUSY=",heading.calibrating);p=field(p," FAULT=",heading.fault);
        p=field(p," AXIS=",heading.axis_confirmed?heading.axis:99);p=text(p," SIGN=");p=signed_number(p,heading.sign);
        p=field(p," N=",imu.gyro_seq);p=field(p," T=",heading.stamp_us);p=field(p," DT=",heading.dt_us);
        p=field(p," HZM=",imu_rate_millihz);p=field(p," MAXUS=",imu_max_service_us);
        p=field(p," DROP=",imu.rejected_frames);p=field(p," AMISS=",irq_missed_acc);
        p=field(p," GMISS=",irq_missed_gyro);p=field(p," RACE=",irq_races);
        p=text(p," YAW=");p=signed_number(p,(int32_t)(heading.yaw_q16_md/65536));
        p=text(p," REF=");p=signed_number(p,heading.reference_md);
        p=text(p," RATE=");p=signed_number(p,heading.rate_mdps);
        p=text(p," U=");p=signed_number(p,straight.ratio);
    }else if(!strcmp(s,"imu calstats")){
        p=text(p,"ICSTAT V=2");p=field(p," N=",calibration.collected);
        p=field(p," WHY=",calibration.reason);p=field(p," AXES=",calibration.axis_mask);
        p=field(p," MAXDT=",calibration.max_dt_us);
        static const char *const v[3]={" VX="," VY="," VZ="},*const d[3]={" DX="," DY="," DZ="};
        for(unsigned i=0;i<3;i++)p=field(p,v[i],calibration.variance_milli[i]);
        for(unsigned i=0;i<3;i++)p=field(p,d[i],calibration.drift_centi[i]);
    }else if(bypass_active(&planner)||(rd(MOTOR_BASE,4)&1U)){
        reply("I783 REJECTED=ACTIVE\r\n");return;
    }else if(!strncmp(s,"imu calpage ",12)){
        uint32_t page;
        if(heading.calibrating||!number(s+12,&page)||page>=100||page*10>=calibration.collected){reply("I783 REJECTED=PAGE\r\n");return;}
        unsigned count=calibration.collected-page*10;if(count>10)count=10;
        p=text(p,"ICDATA");p=field(p," PAGE=",page);p=field(p," COUNT=",count);p=text(p," DATA=");
        for(unsigned j=0;j<count;j++)for(unsigned i=0;i<3;i++){
            if(j||i)*p++=',';
            p=signed_number(p,calibration.raw[page*10+j][i]);
        }
    }else if(!strncmp(s,"imu trace ",10)){
        uint32_t page,status=rd(IMU_BASE,0x70),used=status&2047U;
        if(!number(s+10,&page)||page>=32){reply("ITR REJECTED=PAGE\r\n");return;}
        if(!(status&0x80000000U)){reply("ITR WAIT=1\r\n");return;}
        unsigned count=page*32U<used?used-page*32U:0;if(count>32)count=32;
        p=text(p,"ITR V=1");p=field(p," PAGE=",page);p=field(p," COUNT=",count);
        p=field(p," STATUS=",status);p=field(p," RX=",rd(IMU_BASE,0x74));
        p=field(p," TOTAL=",rd(IMU_BASE,0x80));p=text(p," DATA=");
        static const char hex[]="0123456789ABCDEF";
        for(unsigned i=0;i<count;i++){
            wr(IMU_BASE,0x78,page*32U+i);uint32_t value=rd(IMU_BASE,0x7c);
            *p++=hex[(value>>4)&15U];*p++=hex[value&15U];
        }
    }else if(!strcmp(s,"imu start")){
        halt(T5_CANCEL);uint32_t div=(rd(IMU_BASE,4)+999999U)/1000000U;
        wr(IMU_BASE,0x10,0x100);wr(IMU_BASE,0x14,div);
        imu_start();imu_display_manual_reset();p=text(p,"I783 START=1");
    }else if(!strcmp(s,"imu bias")){
        if(!imu.valid||imu.error!=SENSOR_OK||heading.fault||
           (uint32_t)(rd(IMU_BASE,8)-heading.stamp_us)>=20000U||
           (uint32_t)(rd(JDY_BASE,4)-last_motion_ms)<200U){reply("I783 REJECTED=BIAS\r\n");return;}
        halt(T5_CANCEL);imu_quality_reset();heading_calibration_start(&calibration,&heading);p=text(p,"I783 CALIBRATING=1000");
    }else if(!strcmp(s,"imu axis 2 1")){
        if(!heading.calibrated||heading.calibrating||heading.fault){reply("I783 REJECTED=CAL\r\n");return;}
        heading.axis=2;heading.sign=1;heading.axis_confirmed=true;heading.have_yaw_previous=false;
        heading.yaw_q16_md=0;p=text(p,"I783 AXIS_OK=1");
    }else{halt(T5_CANCEL);reply("I783 REJECTED=SYNTAX\r\n");return;}
    p=text(p,"\r\n");*p=0;reply(b);
}
