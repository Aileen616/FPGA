/* 功能：配置完成后将三路测距交给硬件；保留软件样本结构供既有控制算法使用。 */
static bool tof_auto_started;

/* 功能：推进初始化或读取三路原子快照；无参数/返回；运行阶段无I2C操作。 */
static void tof_frame_service(void)
{
    if(!tof_running){tof_auto_started=false;return;}
    uint32_t now=rd(TOF_BASE,8);
    if(!tof_auto_started){
        if((uint32_t)(now-last_tof)<1000U)return;
        last_tof=now;(void)vl53l1x_array_step(&joint_tof,now);
        if(joint_tof.error<0){halt(T5_SENSOR);tof_stop();return;}
        if(vl53l1x_array_ready(&joint_tof)){wr(TOF_BASE,0x80,1);tof_auto_started=true;}
        return;
    }
    if((uint32_t)(now-last_tof)<1000U)return;
    last_tof=now;
    if(rd(TOF_BASE,0x84)){joint_tof.error=SENSOR_E_IO;halt(T5_SENSOR);tof_stop();return;}
    wr(TOF_BASE,0x88,1);
    for(unsigned i=0;i<3;i++){
        uint32_t base=0xa0+32*i,seq=rd(TOF_BASE,base);
        tof_sample_t *sample=&joint_tof.sample[i];
        if(!seq||seq==sample->sequence)continue;
        uint32_t stamp=rd(TOF_BASE,base+4),flags=rd(TOF_BASE,base+8);
        uint8_t raw[20];
        for(unsigned j=0;j<5;j++){
            uint32_t word=rd(TOF_BASE,base+12+4*j);
            raw[j*4]=(uint8_t)word;raw[j*4+1]=(uint8_t)(word>>8);
            raw[j*4+2]=(uint8_t)(word>>16);raw[j*4+3]=(uint8_t)(word>>24);
        }
        sample->sequence=seq;sample->timestamp_us=stamp;sample->received=true;
        sample->raw_status=raw[0]&31;sample->range_status=sample->raw_status==9?0:255;
        sample->stream_count=raw[2];sample->distance_mm=((uint16_t)raw[13]<<8)|raw[14];
        sample->error=SENSOR_OK;sample->validity=(flags==1)?SENSOR_VALID:SENSOR_INVALID;
    }
    vl53l1x_array_health(&joint_tof,rd(TOF_BASE,8));
}
