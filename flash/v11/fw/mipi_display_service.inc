/* 独立静态显示适配；包含在生成的mipi_display_main.c中，复用同一编译单元的传感器状态。
 * 对外调用display_service，每次只写8个字；display_receive只允许imu/tof诊断命令。
 */
#include "sensor_display.h"
#define DISPLAY_BASE UINT32_C(0x20006000)
static char display_text[SENSOR_DISPLAY_BYTES]; /* 512字节静态页面，不占主循环栈。 */
static sensor_display_snapshot_t display_snapshot;
static uint32_t display_last_us,display_camera_stamp,display_camera_frame;
static unsigned display_word=128,display_bank;
static char display_command[80];
static unsigned display_length;
static bool display_drop;

/* 功能：生成单个显示量；v单位由字段定义，stamp/fresh为us，state为独立有效性；返回完整量。 */
static sensor_display_value_t display_value(int32_t v,uint32_t stamp,uint32_t fresh,sensor_display_state_t state)
{sensor_display_value_t out={v,stamp,fresh,state};return out;}

/* 功能：取得同主循环的一致快照，分批写不可见页；无参数/返回，100ms刷新且等待帧边界确认。 */
static void display_service(void)
{
    uint32_t now=rd(IMU_BASE,8),handshake=rd(DISPLAY_BASE,4);
    if((handshake&1U)!=((handshake>>1)&1U))return;
    if(display_word>=128){
        if((uint32_t)(now-display_last_us)<100000U)return;
        display_last_us=now;
        uint32_t camera=rd(DISPLAY_BASE,8),frame=camera&65535U;
        if(frame!=display_camera_frame){display_camera_frame=frame;display_camera_stamp=now;}
        display_snapshot.camera_frame=frame;
        display_snapshot.camera_state=!(camera&65536U)?DISPLAY_WAIT:
            ((uint32_t)(now-display_camera_stamp)>500000U?DISPLAY_STALE:DISPLAY_VALID);
        display_snapshot.calibrated=heading.calibrated;
        sensor_display_state_t is=imu.error<0?DISPLAY_ERROR:(imu.valid?DISPLAY_VALID:DISPLAY_WAIT);
        for(unsigned i=0;i<3;i++){
            /* 本工程已配置陀螺±500dps、加速度±6g；与器件复位默认值不同。 */
            display_snapshot.gyro[i]=display_value((int32_t)imu.gyro[i]*15625/1024,imu.gyro_us,50000,is);
            display_snapshot.acc[i]=display_value((int32_t)imu.acc[i]*6000/32768,imu.acc_us,50000,is);
        }
        display_snapshot.yaw=display_value((int32_t)(heading.yaw_q16_md/65536),imu.gyro_us,50000,
            heading.calibrated&&heading.feedback_valid?is:DISPLAY_WAIT);
        display_snapshot.tof[0]=display_value(0,now,500000,DISPLAY_OFF);
        for(unsigned i=0;i<3;i++){
            const tof_sample_t *t=&joint_tof.sample[i];
            sensor_display_state_t st=!tof_running?DISPLAY_OFF:
                (joint_tof.error<0||t->error<0?DISPLAY_ERROR:
                (t->validity==SENSOR_STALE?DISPLAY_STALE:(t->validity==SENSOR_VALID?DISPLAY_VALID:
                (t->received?DISPLAY_ERROR:DISPLAY_WAIT))));
            display_snapshot.tof[i+1]=display_value(t->distance_mm,t->timestamp_us,500000,st);
        }
        (void)sensor_display_format(display_text,sizeof(display_text),&display_snapshot,now);
        display_bank=1U-(handshake&1U);display_word=0;
    }
    for(unsigned n=0;n<8&&display_word<128;n++,display_word++){
        const unsigned char *p=(const unsigned char *)display_text+display_word*4;
        uint32_t v=(uint32_t)p[0]|((uint32_t)p[1]<<8)|((uint32_t)p[2]<<16)|((uint32_t)p[3]<<24);
        wr(DISPLAY_BASE,0x400+display_bank*512+display_word*4,v);
    }
    if(display_word==128)wr(DISPLAY_BASE,4,display_bank);
}

/* 功能：只接收显示、IMU和ToF诊断行；c任意字节，now为ms；无返回，不分发运动协议。 */
static void display_receive(uint8_t c,uint32_t now)
{
    (void)now;
    if(c=='\r'||c=='\n'){
        if(!display_drop&&display_length){
            display_command[display_length]=0;
            if(!strcmp(display_command,"display")){
                char b[180],*p=imu_text(b,"MIPI V=1");
                p=imu_field(p," CAM=",(int32_t)rd(DISPLAY_BASE,8));
                p=imu_field(p," STATUS=",(int32_t)rd(DISPLAY_BASE,12));
                p=imu_field(p," OSD=",(int32_t)rd(DISPLAY_BASE,4));imu_send(b,p);
            }else if(!strncmp(display_command,"imu",3))imu_command(display_command);
            else if(!strncmp(display_command,"tof",3))tof_command(display_command);
        }
        display_length=0;display_drop=false;
    }else if(c>=32&&c<127&&!display_drop){
        if(display_length+1<sizeof(display_command))display_command[display_length++]=(char)c;
        else display_drop=true;
    }
}
