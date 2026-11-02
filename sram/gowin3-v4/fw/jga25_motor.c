#include "jga25_motor.h"
#include <stddef.h>
#include <limits.h>
/* 功能：产生12V JGA25的初次开环配置；无参数；返回限幅8192、无极性翻转。
 * 独立配置入口使后续标定不改变通用麦轮算法；无外部句柄或全局状态。 */
mecanum_config_t jga25_open_loop_config(void)
{
    const mecanum_config_t config={8192U,0U};
    return config;
}
/* 功能：使用真实输出轴分辨率转换四轮目标；cal/mrpm/cps不可空，单位及范围见头文件。
 * 返回true表示整组有效；false时整组输出为0；64位乘法避免溢出，无浮点依赖。 */
bool jga25_targets_cps(const jga25_calibration_t *cal, const int32_t mrpm[4], int32_t cps[4])
{
    int32_t result[4]={0,0,0,0};
    bool valid=cal!=NULL && mrpm!=NULL;
    if(cps==NULL) return false;
    /* 先在局部保存结果，允许输入输出同址；每一轮通过后才发布整组目标。 */
    for(unsigned w=0U;valid && w<4U;++w) {
        int64_t speed=mrpm[w];
        int64_t magnitude=speed<0?-speed:speed;
        if(cal->counts_per_rev[w]==0U || cal->counts_per_rev[w]>INT32_MAX ||
           cal->max_mrpm[w]==0U || cal->max_mrpm[w]>JGA25_LABEL_SPEED_MRPM ||
           magnitude>(int64_t)cal->max_mrpm[w]) {valid=false;break;}
        /* 一分钟60秒且每RPM有1000毫RPM：毫RPM*counts/rev/60000；采用对称四舍五入。 */
        int64_t value=(magnitude*(int64_t)cal->counts_per_rev[w]+30000)/60000;
        if(value>INT32_MAX) {valid=false;break;}
        result[w]=(int32_t)(speed<0?-value:value);
    }
    for(unsigned w=0U;w<4U;++w) cps[w]=valid?result[w]:0;
    return valid;
}
