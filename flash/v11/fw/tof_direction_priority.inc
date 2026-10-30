#include "tof_poll_schedule.h"
/* 功能：按静态公平或有界方向优先选择通道并记录服务间隔。
 * 参数：now为当前微秒计数，允许32位回绕；返回：无。不改变样本和200ms失鲜门限。
 */
static void tof_select_priority(uint32_t now)
{
    if(!vl53l1x_array_ready(&joint_tof))return;
    unsigned preferred=joint_tof.cfg.count;
    if(bypass_active(&planner))preferred=bypass_tof_channel(planner.phase);
    if(robot_active(&robot))preferred=robot.forward<0?3:robot.left<0?1:robot.left>0?2:0;
    unsigned i=tof_poll_choose(joint_tof.last_poll_us,joint_tof.cfg.count,preferred,now);
    uint32_t gap=now-joint_tof.last_poll_us[i];
    if(tof_poll_count[i]&&gap>tof_poll_max_us[i])tof_poll_max_us[i]=gap;
    tof_poll_count[i]++;joint_tof.poll_index=(uint8_t)i;
}
