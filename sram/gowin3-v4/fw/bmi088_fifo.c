#include "bmi088_fifo.h"
#include <string.h>
/* 功能：读取1..9字节寄存器；d/out不可空，chip0加速度/1陀螺，reg<128；返回0或负错误。 */
static sensor_result_t read_reg(bmi088_fifo_t *d,unsigned chip,unsigned reg,uint8_t *out,unsigned n)
{
    uint8_t tx[16]={0},rx[16]={0};unsigned skip=chip?1:2;tx[0]=(uint8_t)(reg|0x80);
    if(d->core.bus.spi(d->core.bus.ctx,(uint8_t)chip,tx,rx,n+skip,2000))return SENSOR_E_IO;
    memcpy(out,rx+skip,n);return SENSOR_OK;
}
/* 功能：写陀螺单字节寄存器；d不可空，reg/value为8位值；成功0，否则负错误。 */
static sensor_result_t write_gyro(bmi088_fifo_t *d,uint8_t reg,uint8_t value)
{uint8_t tx[2]={reg,value},rx[2];return d->core.bus.spi(d->core.bus.ctx,1,tx,rx,2,2000)?SENSOR_E_IO:SENSOR_OK;}
/* 功能：解码小端二补码；p不可空且至少2字节；返回signed16。 */
static int16_t decode(const uint8_t *p)
{int32_t v=p[0]+256*(int32_t)p[1];return (int16_t)(v>=32768?v-65536:v);}
/* 功能：锁存故障并撤销新样本；d不可空，rc为负错误；返回rc。 */
static sensor_result_t fault(bmi088_fifo_t *d,sensor_result_t rc)
{d->error=rc;d->valid=false;d->new_gyro=false;return rc;}
/* 功能/参数/返回：见头文件；重新初始化须由停止态显式请求。 */
sensor_result_t bmi088_fifo_start(bmi088_fifo_t *d,const sensor_bus_t *bus,uint32_t now)
{
    if(!d||!bus||!bus->spi)return SENSOR_E_ARG;
    sensor_bus_t copy=*bus;memset(d,0,sizeof(*d));d->last_poll_us=now;
    return bmi088_start(&d->core,&copy,now,20000);
}
/* 功能/参数/返回：见头文件。Bosch DS1.9：0x0A脉冲不可低速轮询，以0x0E帧数和0x3F完整6字节FIFO替代。 */
sensor_result_t bmi088_fifo_step(bmi088_fifo_t *d,uint32_t now)
{
    uint8_t a,g,status,data[9];sensor_result_t rc;
    if(!d)return SENSOR_E_ARG;
    d->new_gyro=false;
    if(d->error<0)return d->error;
    if(!bmi088_ready(&d->core)){
        rc=bmi088_init_step(&d->core,now);if(rc<0)return fault(d,rc);return SENSOR_PENDING;
    }
    if(!d->phase){
        d->fault_stage=1;
        /* 明确关闭外部事件标签，固定每帧XYZ六字节；配置写入同时清空旧FIFO。 */
        rc=write_gyro(d,0x3d,0);if(rc<0)return fault(d,rc);
        /* 按Bosch BMI08x API保留FIFO_CONFIG_1其余位（复位值含bit3），仅改模式/XYZ选择。 */
        rc=read_reg(d,1,0x3e,&g,1);if(rc<0)return fault(d,rc);
        rc=write_gyro(d,0x3e,(uint8_t)((g&0x3c)|0x80));if(rc<0)return fault(d,rc);
        d->phase=1;d->last_poll_us=now;return SENSOR_PENDING;
    }
    if((uint32_t)(now-d->last_poll_us)<1000)return SENSOR_PENDING;
    d->poll_gap_us=now-d->last_poll_us;d->last_poll_us=now;
    if(d->poll_gap_us>d->max_poll_gap_us)d->max_poll_gap_us=d->poll_gap_us;
    /* 主循环长时间失去服务后，不能给FIFO中的旧数据补一个新时间戳。 */
    d->fault_stage=7;
    if(d->gyro_seq&&d->poll_gap_us>=20000)return fault(d,SENSOR_E_TIMEOUT);
    d->fault_stage=1;
    rc=read_reg(d,1,0x3d,&g,1);if(rc<0)return fault(d,rc);d->config0=g;
    if(g&0x80)return fault(d,SENSOR_E_CONFIG);
    rc=read_reg(d,1,0x3e,&g,1);if(rc<0)return fault(d,rc);
    d->config1=g;
    if((g&0xc3)!=0x80)return fault(d,SENSOR_E_CONFIG);
    d->fault_stage=2;
    rc=read_reg(d,0,0,&a,1);if(rc<0)return fault(d,rc);
    rc=read_reg(d,1,0,&g,1);if(rc<0)return fault(d,rc);
    d->acc_id=a;d->gyro_id=g;
    if(a!=0x1e||g!=0x0f)return fault(d,SENSOR_E_ID);
    d->fault_stage=3;
    rc=read_reg(d,0,0x03,&status,1);if(rc<0)return fault(d,rc);
    if(status&0x80){
        rc=read_reg(d,0,0x12,data,9);if(rc<0)return fault(d,rc);
        for(unsigned i=0;i<3;i++)d->acc[i]=decode(data+2*i);
        d->acc_us=now;++d->acc_seq;
    }
    d->fault_stage=4;
    rc=read_reg(d,1,0x0e,&status,1);if(rc<0)return fault(d,rc);d->fifo_before=status;
    d->fifo_status=status;if((status&0x7f)>d->max_fifo)d->max_fifo=status&0x7f;
    /* 20ms以上积压或溢出不能用读取时刻给旧样本补新时间，须停止态重新初始化。 */
    if((status&0x80)||(status&0x7f)>4)return fault(d,SENSOR_E_TIMEOUT);
    if(status&0x7f){
        d->fault_stage=5;
        rc=read_reg(d,1,0x3f,data,6);if(rc<0)return fault(d,rc);
        /* 完整帧读出后复核状态，故障样本不参与零偏与航向积分。 */
        d->fault_stage=6;
        rc=read_reg(d,1,0x0e,&status,1);if(rc<0)return fault(d,rc);d->fifo_after=status;
        d->fifo_status=status;
        if((status&0x80)||(status&0x7f)>4)return fault(d,SENSOR_E_TIMEOUT);
        /* 实板抓到全轴0x8000，PX4 BMI088驱动也排除此模式。不能当作三个-500deg/s发布。
           只丢弃本帧并显式计数，不补零、不递增有效序号、不刷新旧样本时间；持续异常仍失鲜。 */
        if(data[0]==0&&data[1]==0x80&&data[2]==0&&data[3]==0x80&&data[4]==0&&data[5]==0x80){
            if(d->rejected_frames!=UINT32_MAX)++d->rejected_frames;
            d->fault_stage=8;
            d->valid=d->gyro_seq&&d->acc_seq&&(uint32_t)(now-d->gyro_us)<20000&&(uint32_t)(now-d->acc_us)<20000;
            return SENSOR_PENDING;
        }
        for(unsigned i=0;i<3;i++)d->gyro[i]=decode(data+2*i);
        d->gyro_us=now;++d->gyro_seq;d->new_gyro=true;
    }
    d->valid=d->gyro_seq&&d->acc_seq&&(uint32_t)(now-d->gyro_us)<20000&&(uint32_t)(now-d->acc_us)<20000;
    d->fault_stage=0;
    return d->new_gyro?SENSOR_OK:SENSOR_PENDING;
}

/* 功能/参数/返回：见头文件。独立中断路径保留旧FIFO实现作为历史回归，不混用时间戳。 */
sensor_result_t bmi088_irq_step(bmi088_fifo_t *d,const bmi088_events_t *events,uint32_t now)
{
    uint8_t id,data[9],cfg; sensor_result_t rc;
    if(!d||!events||(events->mask&~3u))return SENSOR_E_ARG;
    d->new_gyro=false;
    if(d->error<0)return d->error;
    if(!bmi088_ready(&d->core)){
        rc=bmi088_init_step(&d->core,now);d->acc_id=d->core.acc_id;d->gyro_id=d->core.gyro_id;
        if(rc<0)return fault(d,rc);
        return SENSOR_PENDING;
    }
    if(!d->phase){
        d->fault_stage=1;
        rc=read_reg(d,1,0x3e,&cfg,1);if(rc<0)return fault(d,rc);
        rc=write_gyro(d,0x3e,(uint8_t)(cfg&0x3c));if(rc<0)return fault(d,rc);
        rc=read_reg(d,1,0x3e,&cfg,1);if(rc<0)return fault(d,rc);
        d->config1=cfg;if(cfg&0xc3)return fault(d,SENSOR_E_CONFIG);
        d->phase=1;d->fault_stage=0;return SENSOR_PENDING;
    }
    for(unsigned chip=0;chip<2;chip++){
        uint32_t stamp=chip?events->gyro_us:events->acc_us;
        uint32_t previous=chip?d->gyro_us:d->acc_us,seq=chip?d->gyro_seq:d->acc_seq;
        if(!(events->mask&(1u<<chip))||(uint32_t)(now-stamp)>=20000u||
           (seq&&(stamp==previous||(uint32_t)(stamp-previous)>=0x80000000u)))continue;
        d->fault_stage=2;
        rc=read_reg(d,chip,0,&id,1);if(rc<0)return fault(d,rc);
        if(chip)d->gyro_id=id;else d->acc_id=id;
        if(id!=(chip?0x0f:0x1e))return fault(d,SENSOR_E_ID);
        d->fault_stage=chip?5:3;
        rc=read_reg(d,chip,chip?0x02:0x12,data,chip?6:9);if(rc<0)return fault(d,rc);
        if(chip&&data[0]==0&&data[1]==0x80&&data[2]==0&&data[3]==0x80&&data[4]==0&&data[5]==0x80){
            if(d->rejected_frames!=UINT32_MAX)++d->rejected_frames;
            d->fault_stage=8;continue;
        }
        if(chip){
            for(unsigned i=0;i<3;i++)d->gyro[i]=decode(data+2*i);
            d->gyro_us=stamp;++d->gyro_seq;d->new_gyro=true;
        }else{
            for(unsigned i=0;i<3;i++)d->acc[i]=decode(data+2*i);
            d->acc_us=stamp;++d->acc_seq;
        }
        d->fault_stage=0;
    }
    d->valid=d->gyro_seq&&d->acc_seq&&(uint32_t)(now-d->gyro_us)<20000&&(uint32_t)(now-d->acc_us)<20000;
    return d->new_gyro?SENSOR_OK:SENSOR_PENDING;
}
