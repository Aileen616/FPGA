#include "sensor_display.h"
#include <string.h>

/* 功能：写入一行中的字符串并在行尾截断。
 * 参数：p不可空且至少512字节；row<16，col<32，s为不可空NUL结尾字符串。
 * 返回：无；不会向下一行溢出。
 */
static void put(char *p, unsigned row, unsigned col, const char *s)
{
    while (*s && col < SENSOR_DISPLAY_COLS) p[row * SENSOR_DISPLAY_COLS + col++] = *s++;
}

/* 功能：格式化有符号整数或三位定点数，不依赖printf及浮点库。
 * 参数：out不可空且至少16字节；value覆盖完整int32范围；fraction为0或3。
 * 返回：无；输出以NUL结尾，安全处理INT32_MIN。
 */
static void number(char out[16], int32_t value, unsigned fraction)
{
    char reverse[14]; /* 最大十位整数、一个小数点和符号，不使用大栈数组。 */
    unsigned count = 0, digits = 0, pos = 0;
    uint32_t magnitude = value < 0 ? 0u - (uint32_t)value : (uint32_t)value;
    do {
        if (fraction && digits == fraction) reverse[count++] = '.';
        reverse[count++] = (char)('0' + magnitude % 10u);
        magnitude /= 10u;
        ++digits;
    } while (magnitude || digits <= fraction);
    if (value < 0) out[pos++] = '-';
    while (count) out[pos++] = reverse[--count];
    out[pos] = '\0';
}

/* 功能：检查一个读数的显示有效期，不改变其原始状态。
 * 参数：v不可空，now_us与时间戳同源；有效期须小于半个32位计时周期。
 * 返回：显示状态；错误配置视为ERROR，达到有效期边界视为STALE。
 */
static sensor_display_state_t state_at(const sensor_display_value_t *v, uint32_t now_us)
{
    if ((unsigned)v->state > DISPLAY_ERROR) return DISPLAY_ERROR;
    if (v->state != DISPLAY_VALID) return v->state;
    if (!v->fresh_us || v->fresh_us > INT32_MAX) return DISPLAY_ERROR;
    return (uint32_t)(now_us - v->stamp_us) < v->fresh_us ? DISPLAY_VALID : DISPLAY_STALE;
}

/* 功能：向一行写状态和数值，屏蔽失效的旧值。
 * 参数：p不可空，row<16，label/unit不可空，v不可空，fraction为0或3，now_us同源微秒。
 * 返回：无；ToF负距离视为ERROR，不能当成有效测距。
 */
static void value_row(char *p, unsigned row, const char *label, const char *unit,
                      const sensor_display_value_t *v, unsigned fraction, uint32_t now_us)
{
    static const char *const names[] = {"OFF", "WAIT", "LIVE", "STALE", "ERROR"};
    char formatted[16];
    sensor_display_state_t state = state_at(v, now_us);
    if (!fraction && v->value < 0 && state == DISPLAY_VALID) state = DISPLAY_ERROR;
    put(p, row, 0, label);
    put(p, row, 25, names[state]);
    put(p, row, 21, unit);
    if (state == DISPLAY_VALID) {
        number(formatted, v->value, fraction);
        put(p, row, 8, formatted);
    } else put(p, row, 8, "---");
}

/* 功能/参数/返回：见头文件；每行包含独立状态，不以刷新界面代替刷新传感器时间戳。 */
bool sensor_display_format(char *text, size_t capacity,
                           const sensor_display_snapshot_t *s, uint32_t now_us)
{
    static const char *const gyro_labels[] = {"GX:", "GY:", "GZ:"};
    static const char *const acc_labels[] = {"AX:", "AY:", "AZ:"};
    /* 按用户已确认的物理通道方向显示，不能直接套用三路驱动的0..2槽位。 */
    static const unsigned channel_order[] = {1, 3, 2, 0};
    static const char *const tof_labels[] = {"FR X2:", "LT X4:", "RT X3:", "BK X1:"};
    static const char *const names[] = {"OFF", "WAIT", "LIVE", "STALE", "ERROR"};
    static const char hex[] = "0123456789ABCDEF";
    if (!text || !s || capacity < SENSOR_DISPLAY_BYTES) return false;
    memset(text, ' ', SENSOR_DISPLAY_BYTES);
    put(text, 0, 0, "BX72 CAMERA / IMU / TOF");
    put(text, 1, 0, "FRAME:");
    for (unsigned i = 0; i < 8; ++i) text[32 + 7 + i] = hex[(s->camera_frame >> (28u - i * 4u)) & 15u];
    put(text, 1, 25, names[(unsigned)s->camera_state <= DISPLAY_ERROR ? s->camera_state : DISPLAY_ERROR]);
    put(text, 2, 0, s->calibrated ? "GYRO BIAS: CALIBRATED" : "GYRO BIAS: UNCALIBRATED");
    for (unsigned i = 0; i < 3; ++i) {
        value_row(text, 3 + i, gyro_labels[i], "D/S", &s->gyro[i], 3, now_us);
        value_row(text, 6 + i, acc_labels[i], "G", &s->acc[i], 3, now_us);
    }
    sensor_display_value_t yaw = s->yaw;
    if (!s->calibrated && yaw.state == DISPLAY_VALID) yaw.state = DISPLAY_WAIT;
    value_row(text, 9, "REL YAW", "DEG", &yaw, 3, now_us);
    for (unsigned i = 0; i < 4; ++i)
        value_row(text, 10 + i, tof_labels[i], "MM", &s->tof[channel_order[i]], 0, now_us);
    put(text, 14, 0, "CHIP XYZ / RELATIVE YAW");
    put(text, 15, 0, "LATEST DATA / NOT EXPOSURE SYNC");
    return true;
}
