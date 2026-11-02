#ifndef TOF_POLL_SCHEDULE_H
#define TOF_POLL_SCHEDULE_H
#include <stdint.h>
/* 功能：选择下次测距通道，避免方向优先使其它通道长期不被服务。
 * 参数：last为非空的count个上次服务时间；count为1..4；preferred为优先索引，>=count表示静态公平轮询；now单位us。
 * 返回：0..count-1；非法参数返回0。60ms是抢占优先权的门限，不承诺阻塞总线下的绝对延迟。
 */
static unsigned tof_poll_choose(const uint32_t *last, unsigned count, unsigned preferred, uint32_t now)
{
    if(!last||!count||count>4)return 0;
    unsigned oldest=0;
    for(unsigned i=1;i<count;i++)if((uint32_t)(now-last[i])>(uint32_t)(now-last[oldest]))oldest=i;
    /* 静态只按等待时间选择；运动时也先偿还等待过久的通道，200ms失鲜保护不变。 */
    if(preferred>=count||(uint32_t)(now-last[oldest])>=60000U)return oldest;
    if((uint32_t)(now-last[preferred])>=20000U)return preferred;
    return oldest;
}
#endif
