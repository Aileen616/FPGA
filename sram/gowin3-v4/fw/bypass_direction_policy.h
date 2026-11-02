#ifndef BYPASS_DIRECTION_POLICY_H
#define BYPASS_DIRECTION_POLICY_H
#include "bypass_return.h"
/* 功能：选择本阶段用于放行的ToF；phase为合法阶段。
 * 返回：0前、1右、2左、3无需测距。等待换向时检查下一段，终点等待不再依赖测距。
 */
static inline unsigned bypass_tof_channel(bypass_phase_t phase)
{
    switch(phase){
    case B_WAIT_FRONT:case B_SHIFT_RIGHT:return 1;
    case B_WAIT_PASS:case B_RETURN_LEFT:return 2;
    case B_WAIT_END:return 3;
    default:return 0; /* 初始/终止状态查询按下一次前进展示，前进及越障只看前方。 */
    }
}
/* 功能：获取对应车体边缘净距；in不可空，channel为0..2；返回毫米。 */
static inline uint16_t bypass_direction_distance(const bypass_input_t *in,unsigned channel)
{
    return channel==1?in->right_mm:channel==2?in->left_mm:in->front_mm;
}
#endif
