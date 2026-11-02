/* 显示恢复状态与诊断跨传感器初始化保留；不恢复旧零偏或动作。 */
typedef enum {IDR_WARM=0,IDR_WATCH=1,IDR_WAIT=2,IDR_LOCKED=3} imu_display_phase_t;
typedef struct {
    imu_display_phase_t phase;
    uint32_t since_ms,last_acc,last_gyro,good_pairs,attempts,errors,recovered;
    uint32_t first_ms,last_ms,first_seq,last_seq;
    int32_t first_error,last_error;
    uint8_t first_acc_id,first_gyro_id,last_acc_id,last_gyro_id,last_stage;
    bool ready,control_fault;
} imu_display_recovery_t;
static imu_display_recovery_t imu_display; /* 上电清零；最多3次自动初始化，连续20对新鲜样本准入。 */
static void imu_display_manual_reset(void);
