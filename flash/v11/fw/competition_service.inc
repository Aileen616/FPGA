/* 依赖：tof5_main.c中的rd/wr、IMU、ToF、字符串和停止函数；由准备工具包含。 */
static const uint8_t canonical_bit[4]={2,1,0,3}; /* FL/FR/RL/RR到已核对物理位。 */
static uint32_t local_lease; /* CPU完成本地健康检查后才递增，不能由显示任务续期。 */
static bool pi_configured;
static const uint32_t reply_port=JDY_BASE;
static char osd_page[512];static unsigned osd_word=128;static uint32_t osd_stamp,osd_bank;
/* 仅供显示的上次有效毫米值；上电全0，seen低4位对应前/右/左/后，绝不回灌运动输入。 */
static uint16_t osd_tof_mm[4];
static uint8_t osd_tof_seen;
static int32_t roll_md,pitch_md;static bool tilt_valid;static uint32_t tilt_stamp;
/* 功能：整包入队性能报告；ctx可空、s非空、n不超过272；返回空间足够，不触发运动。 */
static bool perf_enqueue(void *ctx,const char *s,size_t n)
{(void)ctx;if(n>(tail+sizeof(tx)-head-1U)%sizeof(tx))return false;for(size_t i=0;i<n;i++){tx[head]=s[i];head=(head+1U)%sizeof(tx);}return true;}
/* 功能：填充真实控制输入；v非空；无返回，编码器计数先按物理位0/2反向归一。 */
static void robot_inputs(robot_input_t *v)
{
    memset(v,0,sizeof(*v));v->now_ms=rd(JDY_BASE,4);v->imu_valid=imu_ready();
    v->yaw_md=(int32_t)(heading.yaw_q16_md/65536);v->rate_mdps=heading.rate_mdps;
    v->drive_ok=(rd(MOTOR_BASE,4)&3U)==1U&&(rd(JDY_BASE,8)&3U)==1U;
    wr(MOTOR_BASE,8,8);
    for(unsigned i=0;i<4;i++){
        unsigned bit=canonical_bit[i];uint32_t n=rd(MOTOR_BASE,0x20+4*bit);
        v->encoder[i]=(5U&(1U<<bit))?0U-n:n;
        const tof_sample_t *t=&joint_tof.sample[i];
        v->range_mm[i]=t->distance_mm;v->range_seq[i]=t->sequence;
        v->range_valid[i]=i<joint_tof.cfg.count&&tof_running&&vl53l1x_array_ready(&joint_tof)&&t->received&&t->validity==SENSOR_VALID&&
            t->error==SENSOR_OK&&t->range_status==0&&(uint32_t)(rd(TOF_BASE,8)-t->timestamp_us)<200000U;
    }
}
/* 功能：仅提交完整四轮速度；s非空，目标由有界任务生成；返回PL接受结果。 */
static bool robot_commit(const robot_mission_t *s)
{
    if(hw_sequence==UINT32_MAX||local_lease==UINT32_MAX)return false;
    for(unsigned i=0;i<4;i++)wr(MOTOR_BASE,0xb0+4*canonical_bit[i],(uint32_t)s->wheel_cps[i]);
    wr(MOTOR_BASE,0x10,++hw_sequence);wr(MOTOR_BASE,0x14,UINT32_C(0x434c5331));
    wr(JDY_BASE,0x10,++local_lease);
    return (rd(MOTOR_BASE,4)&3U)==1U&&rd(MOTOR_BASE,0x44)==hw_sequence;
}
/* 功能：10ms本地闭环调度；无参数/返回，I2C字节间也调用，不执行传感器事务。 */
static void robot_tick(void)
{
    if(autonomy_owned){autonomy_tick();return;}
    if(!robot_active(&robot))return;
    robot_input_t in;robot_inputs(&in);robot_step(&robot,&in);
    if(!robot_active(&robot)){halt(robot.fault==R_OK?T5_DISTANCE:T5_SENSOR);return;}
    if(!robot_commit(&robot)){robot_stop(&robot,R_DRIVE);halt(T5_DRIVE);}
}
/* 功能：停机配置已验证低输出PI；limit为4096/12288/16384，返回完整回读成功。 */
static bool robot_pi(uint32_t limit)
{
    if(robot_active(&robot)||bypass_active(&planner)||(rd(MOTOR_BASE,4)&1U)||
       (limit!=4096&&limit!=12288&&limit!=16384))return false;
    pi_configured=false;halt(T5_CANCEL);wr(MOTOR_BASE,12,0x5f);wr(MOTOR_BASE,0x78,5);
    for(unsigned i=0;i<4;i++){
        wr(MOTOR_BASE,0x80+4*i,393216);wr(MOTOR_BASE,0x90+4*i,23593);wr(MOTOR_BASE,0xa0+4*i,limit);
        if(rd(MOTOR_BASE,0x80+4*i)!=393216||rd(MOTOR_BASE,0x90+4*i)!=23593||rd(MOTOR_BASE,0xa0+4*i)!=limit)return false;
    }
    wr(MOTOR_BASE,0x74,1);
    pi_configured=rd(MOTOR_BASE,12)==0x5f&&rd(MOTOR_BASE,0x78)==5&&rd(MOTOR_BASE,0x74)==1&&rd(MOTOR_BASE,0x7c)==15;
    return pi_configured;
}
/* 功能：启动显式任务；mode/seq合法；返回准入及第一次零速提交结果。 */
static bool robot_start(robot_mode_t mode,uint32_t seq)
{
    if(mode!=R_MANUAL)return autonomy_begin(seq,mode==R_ROUTE?4:5);
    if(!pi_configured||bypass_active(&planner)||(uint32_t)(rd(JDY_BASE,4)-last_motion_ms)<200U)return false;
    robot_input_t in;robot_inputs(&in);
    if(!robot_begin(&robot,&in,mode,seq))return false;
    heading.enabled=true;wr(IMU_BASE,0x60,1);wr(MOTOR_BASE,8,1);wr(JDY_BASE,12,1);wr(JDY_BASE,12,2);
    local_lease=0;for(unsigned i=0;i<4;i++)robot.wheel_cps[i]=0;
    if(!robot_commit(&robot)){robot_stop(&robot,R_DRIVE);halt(T5_DRIVE);return false;}return true;
}
/* 功能：解析严格空格分隔的有符号整数；s/v非空，n=1..6；返回正好n项且无尾随字符。 */
static bool robot_numbers(const char *s,int32_t *v,unsigned n)
{
    for(unsigned i=0;i<n;i++){
        bool neg=*s=='-';if(neg)s++;uint32_t x=0;unsigned digits=0;
        while(*s>='0'&&*s<='9'){if(x>214748364U||(x==214748364U&&(unsigned)(*s-'0')>7U))return false;x=x*10+(unsigned)(*s++-'0');digits++;}
        if(!digits)return false;
        v[i]=neg?-(int32_t)x:(int32_t)x;
        if(i+1<n){if(*s++!=' ')return false;}else if(*s)return false;
    }return true;
}
/* 功能：处理统一协议；s非空；返回是否属于robot命令，未知robot命令立即停车。 */
static bool robot_command(const char *s)
{
    if(strncmp(s,"robot",5))return false;
    bool ok=false;int32_t v[6];uint32_t now=rd(JDY_BASE,4);
    if(!strcmp(s,"robot video")){perf_monitor_video();return true;}
    if(!strcmp(s,"robot perf")){perf_monitor_request();return true;}
    if(!strcmp(s,"robot state")){
        char b[64],*p=field(text(b,"R1 MODE="),"",robot.mode);p=field(p," ACTIVE=",rd(MOTOR_BASE,4)&1U);p=field(p," FAULT=",robot.fault);
        p=text(p,"\r\n");*p=0;reply(b);return true;
    }
    if(!strcmp(s,"robot")){
        char b[256],*p=text(b,"ROBOT V=1 PROFILE=MONO_DDR");p=field(p," MODE=",robot.mode);p=field(p," FAULT=",robot.fault);
        p=field(p," ACTIVE=",rd(MOTOR_BASE,4)&1U);p=field(p," IMU=",imu_ready());p=field(p," TOF=",vl53l1x_array_ready(&joint_tof));
        p=field(p," SEQ=",robot.last_seq);
        p=field(p," POINT=",robot.point_index);p=field(p," POINTS=",robot.point_count);p=text(p," X=");p=signed_number(p,(int32_t)(robot.x_q16/65536));
        p=text(p," Y=");p=signed_number(p,(int32_t)(robot.y_q16/65536));p=text(p," YAW=");p=signed_number(p,(int32_t)(heading.yaw_q16_md/65536));
        p=text(p,"\r\n");*p=0;reply(b);return true;
    }
    if(!strcmp(s,"robot clear")&&!robot_active(&robot)&&!bypass_active(&planner)){robot.point_count=0;ok=true;}
    else if(!strncmp(s,"robot config ",13)&&robot_numbers(s+13,v,4)){
        uint16_t off[4];ok=true;for(unsigned i=0;i<4;i++){if(v[i]<0||v[i]>1000)ok=false;off[i]=(uint16_t)v[i];}
        if(ok)ok=robot_geometry(&robot,off)&&robot_pi(4096);
    }else if(!strncmp(s,"robot limit ",12)&&robot_numbers(s+12,v,1))ok=robot_pi((uint32_t)v[0]);
    else if(!strncmp(s,"robot point ",12)&&robot_numbers(s+12,v,2))ok=robot_point(&robot,v[0],v[1]);
    else if(!strncmp(s,"robot route ",12)&&robot_numbers(s+12,v,1)&&v[0]>0)ok=robot_start(R_ROUTE,(uint32_t)v[0]);
    else if(!strncmp(s,"robot seek6 ",12)&&robot_numbers(s+12,v,1)&&v[0]>0)ok=autonomy_begin((uint32_t)v[0],6);
    else if(!strncmp(s,"robot seek ",11)&&robot_numbers(s+11,v,1)&&v[0]>0)ok=robot_start(R_SEEK,(uint32_t)v[0]);
    else if(!strncmp(s,"robot drive ",12)&&robot_numbers(s+12,v,5)&&!robot_active(&robot)&&v[4]>0){
        if(v[0]>=-600&&v[0]<=600&&v[1]>=-600&&v[1]<=600&&v[2]>=-600&&v[2]<=600&&v[3]>=200&&v[3]<=2000){
            robot.manual_f=v[0];robot.manual_l=v[1];robot.manual_r=v[2];robot.manual_ms=(uint32_t)v[3];ok=robot_start(R_MANUAL,(uint32_t)v[4]);
        }
    }else if(!strncmp(s,"robot keep ",11)&&robot_numbers(s+11,v,1)&&v[0]>0)ok=!robot_active(&robot)||robot_keep(&robot,(uint32_t)v[0],now);
    else if(!strncmp(s,"robot target ",13)&&robot_numbers(s+13,v,4)&&v[0]>0&&(v[1]==0||v[1]==1)&&(v[3]==0||v[3]==1))
        ok=!robot_active(&robot)||robot_target(&robot,(uint32_t)v[0],now,v[1]!=0,v[2],v[3]!=0);
    if(!ok){robot_stop(&robot,R_CANCEL);halt(T5_CANCEL);reply("ROBOT REJECTED\r\n");}
    else if(strncmp(s,"robot keep ",11)&&strncmp(s,"robot target ",13))reply("ROBOT OK\r\n");
    return true;
}
/* 功能：向OSD的指定行写短文本；row<13，s不超过31字节；无返回。 */
static void osd_line(unsigned row,const char *s)
{
    unsigned n=0;memset(osd_page+row*32,' ',32);while(*s&&n<31)osd_page[row*32+n++]=*s++;
    /* 每行结束让出采样时间，避免整页格式化把200Hz事件积压到下一轮。 */
    if(irq_armed)imu_service();
}
/* 功能：生成姿态和任务页；now为毫秒；无返回；四路数值常驻，缓存失效时必须带状态标记。 */
static void osd_format(uint32_t now)
{
    char b[64],*p;memset(osd_page,' ',sizeof(osd_page));osd_line(0,"BX72 MONO DDR ROBOT V1");
    bool live=imu_display.ready&&imu.valid&&imu.error==SENSOR_OK&&(uint32_t)(rd(IMU_BASE,8)-imu.gyro_us)<20000U;
    if(live){
        /* 先冻结显示样本；中途服务IMU不会把不同帧的轴混入同一次姿态计算。 */
        int32_t ax=imu.acc[0],ay=imu.acc[1],az=imu.acc[2];
        int32_t gx=heading.gyro_mdps[0],gy=heading.gyro_mdps[1];
        int32_t ar=robot_atan2(ay,az);if(irq_armed)imu_service();
        uint32_t norm=robot_sqrt((uint32_t)(ay*ay)+(uint32_t)(az*az));
        int32_t ap=robot_atan2(-ax,(int32_t)norm);if(irq_armed)imu_service();
        uint32_t dt=now-tilt_stamp;
        if(!tilt_valid||dt>200){roll_md=ar;pitch_md=ap;tilt_valid=true;}
        else {roll_md+=(int32_t)((int64_t)gx*dt/1000);if(irq_armed)imu_service();
            pitch_md+=(int32_t)((int64_t)gy*dt/1000);if(irq_armed)imu_service();
            int64_t total=(int64_t)ax*ax+(int64_t)ay*ay+(int64_t)az*az;
            if(total>24159191&&total<36089604){roll_md+=(ar-roll_md)/4;pitch_md+=(ap-pitch_md)/4;}}
        tilt_stamp=now;p=text(b,"ROLL MD ");p=signed_number(p,roll_md);*p=0;osd_line(1,b);
        p=text(b,"PITCH MD ");p=signed_number(p,pitch_md);*p=0;osd_line(2,b);
    }else{tilt_valid=false;osd_line(1,"ROLL -- STALE");osd_line(2,"PITCH -- STALE");}
    if(imu_ready()){p=text(b,"YAW REL MD ");p=signed_number(p,(int32_t)(heading.yaw_q16_md/65536));*p=0;osd_line(3,b);}
    else osd_line(3,"YAW -- CAL REQUIRED");
    p=field(text(b,"MODE "),"",robot.mode);p=field(p," FAULT ",robot.fault);*p=0;osd_line(4,b);
    p=field(text(b,"WAYPOINT "),"",robot.point_index);p=field(p," TOTAL ",robot.point_count);*p=0;osd_line(5,b);
    p=text(b,"X MM ");p=signed_number(p,(int32_t)(robot.x_q16/65536));p=text(p," Y ");p=signed_number(p,(int32_t)(robot.y_q16/65536));*p=0;osd_line(6,b);
    static const char *const names[4]={"FRONT MM ","RIGHT MM ","LEFT MM ","REAR MM "};
    robot_input_t in;robot_inputs(&in);
    for(unsigned i=0;i<4;i++){
        if(in.range_valid[i]){osd_tof_mm[i]=in.range_mm[i];osd_tof_seen|=(uint8_t)(1U<<i);}
        p=text(b,names[i]);p=decimal(p,osd_tof_mm[i]);
        /* 数值栏常驻，但状态必须区分未出首帧、过期及其他无效测量。 */
        if(!in.range_valid[i]){
            if(!(osd_tof_seen&(1U<<i)))p=text(p," INIT");
            else if(joint_tof.sample[i].validity==SENSOR_STALE||
                (uint32_t)(rd(TOF_BASE,8)-joint_tof.sample[i].timestamp_us)>=200000U)p=text(p," STALE");
            else p=text(p," INVALID");
        }
        *p=0;osd_line(7+i,b);
    }
    p=field(text(b,"VIDEO STATUS "),"",rd(0x20007000,8));*p=0;osd_line(11,b);
    osd_line(12,robot.mode==R_DONE?"TASK DONE VERIFY FIELD":"ENCODER PI / LOCAL WATCHDOG");
}
/* 功能：分块更新OSD；无参数/返回，每次最多8个字避免阻塞200Hz IMU。 */
static void display_service(void)
{
    uint32_t now=rd(JDY_BASE,4),ack=rd(0x20007000,4);
    if(osd_word>=128){if((uint32_t)(now-osd_stamp)<100U||(ack&1U)!=((ack>>1)&1U))return;osd_stamp=now;osd_bank=(ack&1U)^1U;osd_format(now);pad_osd();osd_word=0;}
    for(unsigned n=0;n<8&&osd_word<128;n++,osd_word++){
        unsigned i=osd_word*4;uint32_t w=(uint8_t)osd_page[i]|((uint32_t)(uint8_t)osd_page[i+1]<<8)|((uint32_t)(uint8_t)osd_page[i+2]<<16)|((uint32_t)(uint8_t)osd_page[i+3]<<24);
        wr(0x20007000,0x400+osd_bank*512+osd_word*4,w);
    }
    if(osd_word==128)wr(0x20007000,4,osd_bank);
}
