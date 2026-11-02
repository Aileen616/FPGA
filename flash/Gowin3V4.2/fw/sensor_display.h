#ifndef BX72_SENSOR_DISPLAY_H
#define BX72_SENSOR_DISPLAY_H
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#define SENSOR_DISPLAY_COLS 32u
#define SENSOR_DISPLAY_ROWS 16u
#define SENSOR_DISPLAY_BYTES (SENSOR_DISPLAY_COLS * SENSOR_DISPLAY_ROWS)

/* 显示状态互相独立；关闭、等待、错误时不得显示上一次数值。 */
typedef enum {
    DISPLAY_OFF, DISPLAY_WAIT, DISPLAY_VALID, DISPLAY_STALE, DISPLAY_ERROR
} sensor_display_state_t;

/* 单个量的只读快照；value由调用者按下列单位换算，不修改传感器状态。
 * stamp_us和now_us须同源，fresh_us取1..INT32_MAX；快照不可跨完整计时器回绕保留。
 */
typedef struct {
    int32_t value;
    uint32_t stamp_us, fresh_us;
    sensor_display_state_t state;
} sensor_display_value_t;

typedef struct {
    sensor_display_value_t gyro[3]; /* 芯片XYZ轴，毫度/秒；未校准时为原始换算值。 */
    sensor_display_value_t acc[3];  /* 芯片XYZ轴，mg；与陀螺仪分别检查新鲜度。 */
    sensor_display_value_t yaw;     /* 相对参考航向，毫度；不是绝对方位角。 */
    sensor_display_value_t tof[4];  /* 物理X1后/X2前/X3右/X4左，单位mm；不是驱动槽位。 */
    bool calibrated;               /* 仅描述陀螺零偏标定，不代表完整姿态解算通过。 */
    uint32_t camera_frame;         /* 用于调试的相机帧号；不表示与传感器曝光同步。 */
    sensor_display_state_t camera_state; /* 由相机接收层判断帧完整性和失鲜。 */
} sensor_display_snapshot_t;

/* 功能：生成32列16行ASCII信息页，可送入双页OSD的不可见页。
 * 参数：text不可空且capacity>=512；s不可空且不能与text重叠；now_us为同源微秒计时。
 * 返回：true完成，false表示空指针或空间不足且不写text；输出512字节，不附加NUL。
 * 调用：主循环复制一致快照后调用；提交OSD后须等换页确认再复用页面；禁止中断内格式化。
 */
bool sensor_display_format(char *text, size_t capacity,
                           const sensor_display_snapshot_t *s, uint32_t now_us);
#endif
