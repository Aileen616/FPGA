static bool endpoint_image_owned(void);
#define VISION_BASE UINT32_C(0x2000a000)
static fpga_vision_t task3_vision; /* PL像素统计的本地解释。 */
static bool vision_profile_verified=VISION_PROFILE_VERIFIED!=0;
static bool vision_geometry_verified=VISION_GEOMETRY_VERIFIED!=0;
static uint16_t vision_depth_mm=VISION_OBSTACLE_DEPTH_MM;

/* 功能：读取硬件原子快照；无参数/返回；不续驱动租约、不发运动。 */
static void vision_service(void)
{
    uint32_t w[8];
    if(rd(VISION_BASE,0)!=UINT32_C(0x46563331)){task3_vision.frame_valid=false;return;}
    wr(VISION_BASE,4,1);
    for(unsigned i=0;i<8;i++)w[i]=rd(VISION_BASE,0x10U+4*i);
    fv_observe(&task3_vision,w,rd(JDY_BASE,4),vision_profile_verified);
}
/* 功能：处理视觉模式及只读查询；s不可空；命中返回true，不接受外部识别结果。 */
static bool vision_command(const char *s)
{
    uint32_t n,now=rd(JDY_BASE,4);char b[256],*p;
    if(!strcmp(s,"vision snap")){
        if(endpoint_image_owned()){reply("IMG OWNED ENDPOINT_PAUSE\r\n");return true;}
        if(rd(VISION_BASE,0x100)!=UINT32_C(0x494D4731)){reply("IMG NOHW\r\n");return true;}
        if(rd(VISION_BASE,0x104)&2U){reply("IMG BUSY\r\n");return true;}
        wr(VISION_BASE,0x100,1);reply("IMG REQUESTED\r\n");return true;
    }
    if(!strcmp(s,"vision image")){
        if(endpoint_image_owned()){reply("IMG OWNED ENDPOINT_PAUSE\r\n");return true;}
        if(!(rd(VISION_BASE,0x104)&1U)){reply("IMG NOTREADY\r\n");return true;}
        p=field(text(b,"IMG V=1 W=100 H=60 RGB565=1 SEQ="),"",rd(VISION_BASE,0x110));
        p=text(p,"\r\n");*p=0;reply(b);return true;
    }
    if(!strncmp(s,"vision row ",11)){
        if(endpoint_image_owned()){reply("IMG OWNED ENDPOINT_PAUSE\r\n");return true;}
        static char row[410]; /* 每次只发一行，主循环排空队列后主机才请求下一行。 */
        static const char hex[]="0123456789abcdef";
        if(!number(s+11,&n)||n>=60||!(rd(VISION_BASE,0x104)&1U)){reply("IMG ROW_REJECT\r\n");return true;}
        for(uint32_t x=0;x<100;x++){
            wr(VISION_BASE,0x108,n*100+x);
            uint32_t v=rd(VISION_BASE,0x10c);
            for(uint32_t k=0;k<4;k++)row[x*4+k]=hex[(v>>(12-4*k))&15U];
        }
        row[400]='\r';row[401]='\n';row[402]=0;reply(row);return true;
    }
    if(!strcmp(s,"vision raw")){
        vision_service();now=rd(JDY_BASE,4);
        p=field(text(b,"FVR V=2 SEQ="),"",task3_vision.sequence);
        p=field(p," FLAGS=",task3_vision.raw_flags);
        p=field(p," AGE=",(uint32_t)(now-task3_vision.stamp_ms));
        p=field(p," RAW_OK=",fv_raw_fresh(&task3_vision,now));
        p=field(p," CAND=",task3_vision.target_candidate&&fv_raw_fresh(&task3_vision,now));
        p=field(p," VETO=",task3_vision.block_latched);
        p=field(p," POS=",fv_raw_fresh(&task3_vision,now)?task3_vision.position:FV_UNKNOWN);
        p=field(p," OBJ=",task3_vision.obstacle_pixels);p=field(p," OCC=",task3_vision.corridor_obstacle);
        p=field(p," X0=",task3_vision.x0);p=field(p," Y0=",task3_vision.y0);
        p=field(p," X1=",task3_vision.x1);p=field(p," Y1=",task3_vision.y1);
        p=text(p,"\r\n");*p=0;reply(b);return true;
    }
    if(!strcmp(s,"vision status")){
        vision_service();now=rd(JDY_BASE,4);p=field(text(b,"FV3 ENABLE="),"",task3_vision.enabled);
        p=field(p," PROFILE=",vision_profile_verified);p=field(p," GEOMETRY=",vision_geometry_verified);
        p=field(p," SEQ=",task3_vision.sequence);p=field(p," VALID=",fv_fresh(&task3_vision,now));
        p=field(p," TARGET=",task3_vision.target_visible&&fv_fresh(&task3_vision,now));
        p=field(p," VETO=",!fv_released(&task3_vision,now));p=field(p," ALG=",2);
        p=field(p," OBJ=",task3_vision.obstacle_pixels);
        p=field(p," OCC=",task3_vision.corridor_obstacle);p=field(p," X0=",task3_vision.x0);
        p=field(p," Y0=",task3_vision.y0);p=field(p," X1=",task3_vision.x1);p=field(p," Y1=",task3_vision.y1);
        p=text(p,"\r\n");*p=0;reply(b);return true;
    }
    if(!strncmp(s,"vision mode ",12)){
        if(!number(s+12,&n)||n>1||autonomy_owned||bypass_active(&planner)||robot_active(&robot)||pad_owner||
           pad_cal_stage||heading.calibrating||(rd(MOTOR_BASE,4)&1U)||(uint32_t)(now-last_motion_ms)<200U||
           (n&&(!vision_profile_verified||!vision_geometry_verified||!vision_depth_mm))){
            reply("FV3 MODE_OK=0\r\n");return true;
        }
        task3_vision.enabled=n!=0;task3_vision.seen_target=false;
        task3_vision.release_frames=0;task3_vision.block_latched=true;
        reply("FV3 MODE_OK=1\r\n");return true;
    }
    if(!strncmp(s,"vision",6)){reply("FV3 REJECT\r\n");return true;}
    return false;
}
