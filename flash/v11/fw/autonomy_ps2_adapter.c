#include "autonomy_ps2_adapter.h"

/* 功能：区分自主运行时的断链通知和有效按键停车；参数/返回值见头文件。 */
void ap_apply_ps2(ap_input_t *in, const ps2_control_t *pad, bool fresh)
{
    in->external_linked=pad->linked;
    /* 不覆盖其他来源的停车请求；旧帧、坏帧、断链合成stop不解释成人工指令。 */
    if(fresh&&pad->linked){
        if(pad->previous&PS_CROSS)in->estop=true;
        else if(pad->stop)in->cancel=true;
    }
}
