    /* 本段只加入独立PWM工程，旧ICAL接口不变，新增整窗失败原因和全速原始窗口。 */
    if(!strcmp(line,"imu calstats")){
        p=imu_text(p,"ICSTAT V=2");p=imu_field(p," N=",calibration.collected);
        p=imu_field(p," WHY=",calibration.reason);p=imu_field(p," AXES=",calibration.axis_mask);
        p=imu_field(p," MAXDT=",calibration.max_dt_us);
        const char *vl[3]={" VX="," VY="," VZ="},*dl[3]={" DX="," DY="," DZ="},*hl[3]={" HX="," HY="," HZ="};
        for(unsigned i=0;i<3;i++)p=imu_field(p,vl[i],calibration.variance_milli[i]);
        for(unsigned i=0;i<3;i++)p=imu_field(p,dl[i],calibration.drift_centi[i]);
        for(unsigned i=0;i<3;i++)p=imu_field(p,hl[i],calibration.half_centi[i]);
        imu_send(b,p);return;
    }
    if(!strncmp(line,"imu calpage ",12)){
        if(!idle||heading.calibrating){imu_reject("BUSY");return;}
        unsigned page=0,n=0;
        const char *s=line+12;
        while(*s>='0'&&*s<='9'&&n<2){page=page*10+(unsigned)(*s++-'0');n++;}
        if(!n||*s||page*10>=calibration.collected){imu_reject("PAGE");return;}
        unsigned count=calibration.collected-page*10;if(count>10)count=10;
        p=imu_text(p,"ICDATA");p=imu_field(p," PAGE=",page);p=imu_field(p," COUNT=",count);
        p=imu_text(p," DATA=");
        for(unsigned j=0;j<count;j++)for(unsigned i=0;i<3;i++){
            if(j||i)*p++=',';
            p=imu_number(p,calibration.raw[page*10+j][i]);
        }
        imu_send(b,p);return;
    }
