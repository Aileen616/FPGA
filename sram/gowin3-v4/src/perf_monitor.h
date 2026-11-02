#ifndef BX72_PERF_MONITOR_H
#define BX72_PERF_MONITOR_H
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
/* 功能：整包入队回调；ctx可空，data不可空，n为字节数；成功返回true，队列满返回false。 */
typedef bool (*perf_write_t)(void *ctx,const char *data,size_t n);
/* 功能：初始化只读性能报告服务；write不可空；无返回，不修改电机或通信租约。 */
void perf_monitor_init(perf_write_t write);
extern bool perf_monitor_pending; /* 有诊断请求时为true；主循环仅在该标志置位时进入报告服务。 */
/* 功能：记录整包入队后的队列长度；queued为0..511字节；无返回，不插入每轮循环。 */
void perf_monitor_queue(uint32_t queued);
/* 功能：记录发送队列入队失败一次；无参数；无返回。 */
void perf_monitor_tx_drop(void);
/* 功能：申请一次异步快照；无参数；无返回，已有请求或报告时忽略重复命令。 */
void perf_monitor_request(void);
/* 功能：推进快照和非阻塞报告；now_ms为自然回绕毫秒；无返回，每次至多读一个统计字。 */
void perf_monitor_service(uint32_t now_ms);
#endif
