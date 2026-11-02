/* 启动条件使用屏幕所示探头原始距离；运动期仍使用原车体净距和保护。 */
#define PS2_SIDE_START_MM 400U /* 用户指定左右均严格大于400mm，等于400mm拒绝。 */
enum {
    SG_CONFIG=1U,SG_SEQUENCE=2U,SG_SETTLE=4U,SG_IMU=8U,
    SG_FRONT_INVALID=16U,SG_RIGHT_INVALID=32U,SG_LEFT_INVALID=64U,
    SG_FRONT_NEAR=128U,SG_RIGHT_NEAR=256U,SG_LEFT_NEAR=512U,
    SG_BUSY=1024U,SG_INTERNAL=2048U
};
static uint32_t start_reject_mask; /* 最近一次入口拒绝位图；0表示入口条件满足。 */

/* 功能：无副作用检查本次启动；seq为新序号，0非法；返回拒绝原因位图，0为通过。 */

