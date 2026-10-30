#include "fpga_vision.h"
#include "vision_profile.h"
#include <string.h>

/* 功能：清状态并锁定视觉阻挡；v不可空；无返回。 */
void fv_init(fpga_vision_t *v)
{memset(v,0,sizeof(*v));v->block_latched=true;}

/* 功能：判断采集新鲜度；v不可空，now为ms；完整且帧龄<250ms返回true。 */
bool fv_raw_fresh(const fpga_vision_t *v,uint32_t now)
{return v->raw_valid&&(uint32_t)(now-v->stamp_ms)<250U;}

/* 功能：判断已标定帧新鲜度；v不可空，now为ms；可供控制使用返回true。 */
bool fv_fresh(const fpga_vision_t *v,uint32_t now)
{return v->frame_valid&&fv_raw_fresh(v,now);}

/* 功能：查询附加阻挡是否解除；v不可空，now为ms；true仍必须经过ToF和几何准入。 */
bool fv_released(const fpga_vision_t *v,uint32_t now)
{return fv_fresh(v,now)&&!v->block_latched&&v->release_frames>=3;}

/* 功能：接收新PL快照并筛选蓝色目标；v/w不可空，now为ms，verified为现场确认；无返回。 */
void fv_observe(fpga_vision_t *v,const uint32_t w[8],uint32_t now,bool verified)
{
    if(w[0]==v->sequence)return; /* 重读不能续期，也不能充当三个新帧。 */
    bool continuous=fv_raw_fresh(v,now)&&w[0]==v->sequence+1U;
    v->sequence=w[0];v->stamp_ms=now-w[7];v->raw_flags=w[1];
    v->obstacle_pixels=w[2];v->corridor_obstacle=w[3];
    /* w[4]为旧RTL的白色统计；相机看不到地面，因此完全不参与本版决策。 */
    v->x0=(uint16_t)w[5];v->y0=(uint16_t)(w[5]>>16);
    v->x1=(uint16_t)w[6];v->y1=(uint16_t)(w[6]>>16);
    uint32_t box=0; /* 包围框面积，单位为下采样像素，最大24000。 */
    bool bounds=v->x0<=v->x1&&v->x1<200&&v->y0<=v->y1&&v->y1<120;
    if(bounds)box=(uint32_t)(v->x1-v->x0+1U)*(v->y1-v->y0+1U);
    v->raw_valid=(w[1]&1U)&&w[7]<250U&&w[2]<=24000U&&w[3]<=VISION_ROI_AREA&&w[3]<=w[2];
    v->frame_valid=verified&&v->raw_valid;
    v->target_candidate=v->raw_valid&&bounds&&w[2]>=VISION_MIN_PIXELS&&box&&w[2]<=box&&
        w[2]*100U>=box*VISION_MIN_FILL&&(uint32_t)(v->x1-v->x0+1U)>=VISION_MIN_WIDTH&&
        (uint32_t)(v->y1-v->y0+1U)>=VISION_MIN_HEIGHT;
    v->target_visible=verified&&v->target_candidate;
    v->position=FV_UNKNOWN;
    if(v->target_candidate){
        uint32_t center_twice=(uint32_t)v->x0+v->x1; /* 双倍横坐标，避免除法和小数。 */
        v->position=center_twice<100U?FV_LEFT:center_twice>298U?FV_RIGHT:FV_CENTER;
    }
    if(v->frame_valid&&v->target_visible)v->seen_target=true;
    /* 出现蓝色立即阻挡；未见蓝色只撤销附加否决，绝不产生“空地有效”结论。 */
    if(!v->raw_valid||w[3]>VISION_BLUE_VETO_PIXELS){
        v->release_frames=0;v->block_latched=true;
    }else{
        if(!continuous){v->release_frames=0;v->block_latched=true;}
        if(v->release_frames<3)v->release_frames++;
        if(v->release_frames>=3)v->block_latched=false;
    }
}
