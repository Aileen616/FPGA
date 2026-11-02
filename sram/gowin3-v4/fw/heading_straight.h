#ifndef HEADING_STRAIGHT_H
#define HEADING_STRAIGHT_H
#include "heading_hold.h"
/* 四轮物理通道1..4；ratio为千分比，限制±100，每10ms最多变化2。 */
typedef struct { int32_t base[4],ratio; } heading_straight_t;
/* 功能：锁定本次四轮基础速度和起步航向；s/h/base不可空，base各50..1000cps；成功true。 */
bool straight_begin(heading_straight_t *s,heading_hold_t *h,const int32_t base[4]);
/* 功能：产生按各轮基速缩放的P+角速度阻尼目标；s/h/out不可空；tick仅每10ms置true；有效返回true。
 * P=4千分比/度，D=2千分比/(度/秒)，最大修正10%；未实车整定。
 */
bool straight_targets(heading_straight_t *s,heading_hold_t *h,bool tick,int32_t out[4]);
#endif
