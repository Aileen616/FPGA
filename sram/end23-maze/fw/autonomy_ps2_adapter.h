#ifndef AUTONOMY_PS2_ADAPTER_H
#define AUTONOMY_PS2_ADAPTER_H
#include "autonomy_policy.h"
#include "ps2_control.h"

/* 功能：把手柄新帧的明确停止请求传入自主策略；in/pad不可空，fresh来自真实解析器。
 * 返回：无。仅供自主任务3..6使用；直接驾驶继续使用原pad.stop，不经过此转换。
 * 来源：ps2_control_t为现有手柄解析器，调用顺序为解析快照→本函数→ap_step。
 */
void ap_apply_ps2(ap_input_t *in, const ps2_control_t *pad, bool fresh);
#endif
