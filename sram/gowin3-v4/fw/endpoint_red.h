#ifndef ENDPOINT_RED_H
#define ENDPOINT_RED_H
#include <stdbool.h>
#include <stdint.h>
#define EP_PIXELS 6000U
#define EP_RESULTS 16U
#define EP_RED_AREA_MIN 16U
/* 保留原服务的数据布局；坐标单位为100×60图像像素，area为红色8连通域面积。
 * 红色检测只输出最大合格区域；threshold=70表示最低饱和度，kernel/support固定为0。 */
typedef struct {
    uint8_t x,y,w,h,threshold,kernel;
    uint16_t area;
    uint32_t support;
} ep_box_t;
/* count只取0/1，表示是否存在合格红域；error=0正常、5队列容量异常、6图像装载不完整。 */
typedef struct {
    ep_box_t box[EP_RESULTS];
    uint32_t steps;
    uint16_t members_peak,groups_peak;
    uint8_t count,error;
    bool done;
} ep_result_t;
/* 功能：判断单个像素是否为红色。参数：pixel为任意RGB565原始值。返回：true为通过颜色条件，false为未通过。 */
bool ep_red_pixel(uint16_t pixel);
/* 功能：终止旧计算并准备完整新帧。参数：无。返回：无；装载新帧之前必须调用。 */
void ep_reset(void);
/* 功能：装载一个像素。参数：index为0～5999，pixel为RGB565；均为值参数。返回：无，忙时或越界忽略。 */
void ep_pixel(uint32_t index,uint16_t pixel);
/* 功能：开始检测完整帧。参数：无。返回：true已启动；false为仍忙或装载不完整，后者发布error=6。 */
bool ep_begin(void);
/* 功能：推进一次有界扫描或一个像素的邻域处理。参数：无。返回：true已完成或出错，false需继续调用。 */
bool ep_step(void);
/* 功能：读取静态结果。参数：无。返回：永不为空的只读指针，仅done=true后可用，不可由调用方修改。 */
const ep_result_t *ep_result(void);
/* 功能：查询静态工作区大小。参数：无。返回：工作区字节数，不包括调用栈及代码常量。 */
uint32_t ep_workspace_bytes(void);
#endif
