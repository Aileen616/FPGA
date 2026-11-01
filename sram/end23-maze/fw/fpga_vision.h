#ifndef FPGA_VISION_H
#define FPGA_VISION_H
#include <stdbool.h>
#include <stdint.h>
/* 相机原图下采样坐标200×120；位置为图像左/中/右，未标定时不等同车体方位。 */
typedef enum { FV_UNKNOWN=0,FV_LEFT=1,FV_CENTER=2,FV_RIGHT=3 } fv_position_t;
typedef struct {
    uint32_t sequence,stamp_ms,raw_flags,obstacle_pixels,corridor_obstacle;
    uint16_t x0,y0,x1,y1;
    uint8_t release_frames; /* 连续新帧计数，0..3；仅用于解除视觉阻挡。 */
    bool enabled,raw_valid,frame_valid,target_candidate,target_visible,seen_target,block_latched;
    fv_position_t position;
} fpga_vision_t;
/* 功能：初始化并锁定视觉阻挡；v不可空；无返回，辅助默认关闭。 */
void fv_init(fpga_vision_t *v);
/* 功能：解释8字PL快照；v/w不可空，now单位ms，verified为蓝板标定确认；无返回，忽略旧白地计数w[4]。 */
void fv_observe(fpga_vision_t *v,const uint32_t w[8],uint32_t now,bool verified);
/* 功能：检查原始帧完整及250ms新鲜度；v不可空，now为ms；合格返回true，不代表识别正确。 */
bool fv_raw_fresh(const fpga_vision_t *v,uint32_t now);
/* 功能：检查已标定视觉帧新鲜度；参数同fv_raw_fresh；合格返回true。 */
bool fv_fresh(const fpga_vision_t *v,uint32_t now);
/* 功能：检查三帧后是否解除视觉附加阻挡；参数同fv_raw_fresh；true仅撤销视觉否决，不提供通行授权。 */
bool fv_released(const fpga_vision_t *v,uint32_t now);
#endif
