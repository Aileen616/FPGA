#ifndef ENDPOINT_BLACK_H
#define ENDPOINT_BLACK_H
#include <stdint.h>
#include <stdbool.h>
#define EP_PIXELS 6000U
#define EP_RESULTS 16U
/* 坐标为100x60原始缩略图像素；area为开运算后的连通域像素数；support的bit0..16对应16..80阈值。 */
typedef struct {
    uint8_t x,y,w,h,threshold,kernel;
    uint16_t area;
    uint32_t support;
} ep_box_t;
/* error=0正常、1组容量、2成员容量、3候选容量、4结果容量、5队列容量；溢出不是无目标。 */
typedef struct {
    ep_box_t box[EP_RESULTS];
    uint32_t steps;
    uint16_t members_peak,groups_peak;
    uint8_t count,error;
    bool done;
} ep_result_t;
/* 功能：复位单实例工作区元数据，不清图像；无参数；无返回，终止未完成计算。 */
void ep_reset(void);
/* 功能：装载一个RGB565像素；index为0..5999、pixel为原始16位值；无返回，忙时或越界忽略。 */
void ep_pixel(uint32_t index,uint16_t pixel);
/* 功能：开始处理已完整装载的图像；无参数；成功true，忙时false。 */
bool ep_begin(void);
/* 功能：推进一行形态学、一个游程或一个有界候选步骤；无参数；完成或错误时true，否则false。 */
bool ep_step(void);
/* 功能：读取单实例结果；无参数；返回只读非空指针，done之前不可使用候选。 */
const ep_result_t *ep_result(void);
/* 功能：返回静态工作区字节数；无参数；返回不含调用栈的字节数。 */
uint32_t ep_workspace_bytes(void);
#endif
