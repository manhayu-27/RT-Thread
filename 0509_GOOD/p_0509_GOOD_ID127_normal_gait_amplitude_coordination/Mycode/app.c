#include "app.h"
#include "app_config.h"

#include "main.h"
#include "motor_control.h"
#include <stdio.h>
#include <string.h>
#include <math.h>
#include "stm32f4xx_hal.h"
#include <stdint.h>
#include <stdarg.h>

/* --- CAN 电机底层驱动 --- */
#include "can_driver.h"       
#include "control.h"
#include "getdata.h"
#include "ankle_can_config.h"

/* --- 硬件 ID 与控制参数 --- */
#define KNEE_MOTOR_ID              0U        // 膝关节 RS485 ID
#define HIP_MOTOR_ID               1U        // 髋关节 RS485 ID
#define ANKLE_MOTOR_ID             ANKLE_CAN_NODE_ID  // 踝关节 CAN ID，仅使用节点 1

/* 重要调试开关：0=正常步态，1=固定角度测试 */
#define MOTOR_TEST_MODE            0

#define CONTROL_PERIOD_MS          10U
#define PRINT_PERIOD_MS            10U
#define VOFA_ENABLE                1U

/* ================= Node127 融合控制参数 =================
 * Node127 通过 CAN2（PB5/PB6）通信；主控每 10 ms 发送一次 0x010 同步帧。
 * ID=127 发送三路大腿肌电，ID=227 发送同序号俯仰角、X 轴角速度和运动状态。
 */
#define NODE127_SYNC_CAN_ID        0x010U
#define NODE127_SYNC_PERIOD_MS     10U
#define NODE127_DATA_TIMEOUT_MS    100U
#define NODE127_WARMUP_MS           3000U
#define NODE127_EMG_TIMEOUT_MS     100U
#define NODE127_EMG_LPF_ALPHA      0.18f   /* legacy default, kept for compatibility with older notes */
#define NODE127_EMG_ATTACK_ALPHA   0.80f   /* fast envelope attack: human intent should start the gait quickly */
#define NODE127_EMG_RELEASE_ALPHA  0.10f   /* slower release: filters single-sample dropouts without long motor overrun */
#define NODE127_EMG_BASELINE_ALPHA 0.0008f
#define NODE127_EMG_BASELINE_TRACK_RAW 80.0f  /* uV MAV: only track baseline while idle. */
#define NODE127_EMG_FULL_SCALE_RAW 300.0f /* uV active range; tune after subject calibration. */
#define NODE127_EMG_ON_SIGMA       6.0f
#define NODE127_EMG_OFF_SIGMA      3.0f
#define NODE127_EMG_ON_MIN_RAW     25.0f
#define NODE127_EMG_OFF_MIN_RAW    12.0f
#define NODE127_EMG_ON_COUNT       3U
#define NODE127_EMG_OFF_COUNT      20U
#define NODE127_INTENT_WAIT_IMU_MS 300U
#define NODE127_GYRO_DEADBAND_RAW  320.0f  /* Q6: 5 degrees/s. */
#define NODE127_GYRO_FULL_RAW      9600.0f /* Q6: another 150 degrees/s reaches full scale. */
#define NODE127_MOTION_HOLD_MS     80U     /* short anti-jitter hold only */
#define NODE127_GAIT_SCALE         1.0f
#define NODE127_PITCH_Q            64.0f
#define NODE127_IMU_MOVING_FLAG    0x0001U
#define NODE127_CALIB_MS           1000U     /* Rest calibration; all motors hold during this time. */
#define NODE127_INTENT_ATTACK_ALPHA 0.80f
#define NODE127_INTENT_RELEASE_ALPHA 0.28f
#define NODE124_START_IGNORE_MS     1200U

/* =============== 三关节协调与幅度限制参数 ===============
 * 肌电决定是否允许运动，IMU 决定实际运动状态。
 * 髋关节直接跟随大腿俯仰，膝、踝相位按大腿累计摆角推进。
 * 所有关节从上电当前位置开始，避免首次启动时目标角度突然跳变。
 */
#define GAIT_AMP_MIN_SCALE          0.35f    /* 启动时使用 35% 正常步态幅值，避免突然跳变。 */
#define GAIT_AMP_MAX_SCALE          1.00f    /* 最大输出为正常步态幅值。 */
#define GAIT_AMP_RAMP_UP_PER_S      2.20f    /* 步态幅值渐变速度，从小幅逐步过渡到正常幅值。 */
#define GAIT_AMP_DECAY_PER_S        1.60f    /* decay commanded gait amplitude quickly after human intent disappears */
#define KNEE_JOINT_ANGLE_SCALE      0.92f    /* 膝关节步态角度缩放系数。 */
#define HIP_JOINT_ANGLE_SCALE       0.85f    /* 髋关节步态角度缩放系数。 */
#define ANKLE_JOINT_ANGLE_SCALE     0.95f    /* 踝关节步态角度缩放系数。 */
#define THIGH_PITCH_SIGN            1.0f     /* Set to -1 if forward swing commands the wrong direction. */
#define THIGH_PITCH_GAIN            1.0f     /* Prosthesis hip angle / measured thigh angle. */
#define THIGH_PHASE_PCT_PER_DEG     1.0f     /* Tune until one full thigh cycle advances about 100 percent. */
#define THIGH_PHASE_NOISE_DEG       0.08f
#define THIGH_PHASE_MAX_DELTA_DEG   8.0f
#define KNEE_MAX_CMD_SPEED_DEG_S    320.0f   /* 膝关节目标角速度上限，防止目标变化过快。 */
#define HIP_MAX_CMD_SPEED_DEG_S     240.0f

#define INIT_HOLD_MS               1500U
#define GAIT_PERIOD_MS             1000U

/* 物理换算常量。 */
#define DEG_TO_RAD                 0.01745329252f
#define RAD_TO_DEG                 57.2957795131f
#define KNEE_REDUCTION_RATIO       6.33f
#define HIP_REDUCTION_RATIO        6.33f
#define ANKLE_WAVE_SCALE           1.0f
#define ANKLE_TURN_PER_DEG         (ANKLE_OUTPUT_REDUCTION_RATIO / 360.0f)

/* 按步态曲线的正负方向运动，零点取上电时的当前位置。 */
#define KNEE_CMD_MIN_DEG           (-5.0f)
#define KNEE_CMD_MAX_DEG           (66.0f)

#define HIP_CMD_MIN_DEG            (-24.0f)
#define HIP_CMD_MAX_DEG            (24.0f)

/* 踝关节按照步态曲线运行。
 * 角度是相对于上电时脚部位置的输出轴角度。
 * 程序会减去起始相位的曲线值，避免上电瞬间冲向限位。
 */
#define ANKLE_CMD_MIN_DEG          (-22.0f)  /* 踝关节跖屈角度下限，保留电机余量。 */
#define ANKLE_CMD_MAX_DEG          (12.0f)   /* 踝关节背屈角度上限，保留电机余量。 */
#define ANKLE_GAIT_OFFSET_DEG      (0.0f)
#define ANKLE_MAX_CMD_SPEED_DEG_S  160.0f

/* 从步态相位 0% 开始运行踝关节曲线。 */
#define GAIT_START_PHASE_PCT       0.0f

#define KNEE_DIR_SIGN              1.0f
#define HIP_DIR_SIGN               1.0f
/* 踝关节方向。如果实际动作相反，将此值改为 -1.0f。 */
#define ANKLE_CMD_DIR              (1.0f)
#define ANKLE_FEEDBACK_TIMEOUT_MS  1000U
#define ANKLE_FEEDBACK_STALE_MS    300U
#define ANKLE_CLOSED_LOOP_DELAY_MS 120U
#define ANKLE_START_RAMP_MS        1000U

#define KP_TARGET                  0.52f    /* 正常步态时的位置刚度。 */
#define KW_TARGET                  0.18f
#define KP_HOLD_BEFORE_EMG         0.08f    /* 检测到肌电前仅轻度保持，避免上电锁定造成抖动。 */
#define KW_HOLD_BEFORE_EMG         0.08f

/* --- 算法控制参数 --- */
#define PHASE_ADVANCE_PCT          0.0f      /* 三个关节共用同一个相位，保证步态同步。 */
#define POS_SLEW_BASE_RAD_S_INIT   6.0f      
#define WFF_LPF_ALPHA_INIT         0.18f     
#define CURVE_TENSION              0.62f
#define PHASE_SMOOTH_PCT           1.2f

typedef struct {
    float phase_pct;
    float joint_deg;
} JointCurvePoint_t;

typedef struct {
    const char *name; uint16_t motor_id; float dir_sign; float amp_scale; float reduction_ratio;
    const JointCurvePoint_t *curve; uint32_t point_count;
    MOTOR_send cmd; MOTOR_recv data;
    float center_pos; float soft_kp; float soft_kw; float traj_scale;
    float cmd_pos_state; float prev_cmd_pos_state; float err_abs_lpf;
    float wff_state; float slew_base_state; float wff_alpha_state;
    float err_peak_abs_cycle; float err_peak_abs_last;
    float err_sum_cycle; uint32_t err_cnt_cycle;
    float phase_adv_state; float fb_pos_state; uint8_t fb_valid_state;
    HAL_StatusTypeDef last_servo_status; uint8_t traj_started;
    float target_ref_deg; float target_curve_deg; float target_slope_deg_per_pct;
    uint8_t is_calibrated;
} JointController_t;

/* --- 膝关节与髋关节步态数据 ---
 * 膝关节在摆动期屈曲，髋关节在支撑期和摆动期使用对应曲线。
 * 程序会统一减去起始相位的曲线值，避免肌电触发瞬间跳变。
 */
static const JointCurvePoint_t kKneeCurve[] = {
    {0.0f,   0.0f},
    {6.0f,   1.0f},
    {12.0f, 12.0f},
    {20.0f,  7.0f},
    {30.0f,  0.0f},
    {40.0f,  1.0f},
    {48.0f,  5.0f},
    {55.0f, 18.0f},
    {62.0f, 42.0f},
    {68.0f, 62.0f},
    {74.0f, 65.0f},
    {80.0f, 52.0f},
    {86.0f, 28.0f},
    {92.0f,  4.0f},
    {96.0f, -2.0f},
    {100.0f, 0.0f}
};
static const JointCurvePoint_t kHipCurve[] = {
    {0.0f,  23.0f},
    {8.0f,  20.0f},
    {18.0f, 10.0f},
    {28.0f,  0.0f},
    {40.0f, -14.0f},
    {55.0f, -23.0f},
    {62.0f, -18.0f},
    {68.0f,  -6.0f},
    {74.0f,   8.0f},
    {80.0f,  19.0f},
    {86.0f,  22.0f},
    {94.0f,  19.0f},
    {100.0f, 23.0f}
};
#define KNEE_POINT_COUNT  ((uint32_t)(sizeof(kKneeCurve) / sizeof(kKneeCurve[0])))
#define HIP_POINT_COUNT   ((uint32_t)(sizeof(kHipCurve) / sizeof(kHipCurve[0])))

/* --- 全局句柄定义 --- */
/* Peripheral handles are owned by CubeMX-generated main.c. */
extern CAN_HandleTypeDef hcan1; // 由 CubeMX 生成的 can.c 提供。
extern CAN_HandleTypeDef hcan2; // 127模块 CAN2

typedef struct {
    uint8_t accel_chip_id;
    uint8_t gyro_chip_id;
    uint8_t accel_ready;
    uint8_t gyro_ready;
    uint8_t init_ok;
    int16_t accel_raw[3];
    int16_t gyro_raw[3];
    float accel_g[3];
    float gyro_dps[3];
} BMI088_t;

typedef struct {
    uint32_t frame24;
    uint16_t angle_raw;
    float angle_deg;
    uint8_t mag_field_state;
    uint8_t push_state;
    uint8_t track_loss;
    uint8_t crc_rx;
    uint8_t crc_calc;
    uint8_t read_ok;
} MT6701_t;

static volatile float g_ankle_cmd_deg = 0.0f;
static volatile float g_ankle_cmd_turn = 0.0f;
static volatile uint8_t g_ankle_gait_enable = 0U;
static float g_ankle_cmd_turn_state = 0.0f;
static uint8_t g_ankle_cmd_turn_state_valid = 0U;

/* 上电阶段码，用于判断程序停在哪一步。 */
volatile uint32_t g_boot_stage_code = 0U;
volatile uint32_t g_hardfault_count = 0U;

/* Node127 运动状态：无运动时不推进相位，三个关节保持上一目标。 */
static float g_node127_emg_baseline = 0.0f;
static float g_node127_emg_norm = 0.0f;
static float g_node127_emg_baseline_ch[3] = {0.0f, 0.0f, 0.0f};
static float g_node127_emg_env_ch[3] = {0.0f, 0.0f, 0.0f};
static float g_node127_emg_norm_ch[3] = {0.0f, 0.0f, 0.0f};
static float g_node127_emg_calib_m2_ch[3] = {0.0f, 0.0f, 0.0f};
static float g_node127_emg_on_delta_ch[3] = {0.0f, 0.0f, 0.0f};
static float g_node127_emg_off_delta_ch[3] = {0.0f, 0.0f, 0.0f};
static uint8_t g_node127_emg_intent = 0U;
static uint8_t g_node127_emg_on_count = 0U;
static uint8_t g_node127_emg_off_count = 0U;
static uint8_t g_node127_emg_wait_release = 0U;
static uint32_t g_node127_emg_intent_tick = 0U;
static uint8_t g_node127_emg_baseline_valid = 0U;
static uint8_t g_node127_motion_active = 0U;
static float g_node127_motion_level = 0.0f;
static float g_node127_intent_level = 0.0f;
static uint32_t g_node124_ignore_until_tick = 0U;
static float g_node127_phase_pct = GAIT_START_PHASE_PCT;
static float g_gait_amp_state = 0.0f;          /* 统一的步态幅值系数，供所有关节使用。 */
static uint8_t g_node127_first_motion_seen = 0U;
static int32_t g_node127_pitch_bias_sum = 0;
static float g_node127_pitch_bias = 0.0f;
static float g_node127_last_phase_pitch_deg = 0.0f;
static uint16_t g_node127_last_phase_sequence = 0U;
static uint32_t g_node127_last_imu_motion_tick = 0U;

typedef enum {
    NODE127_STATE_BOOT = 0,
    NODE127_STATE_CALIB = 1,
    NODE127_STATE_IDLE = 2,
    NODE127_STATE_MOVE = 3,
    NODE127_STATE_HOLD = 4,
    NODE127_STATE_LOST = 5,
    NODE127_STATE_INTENT_PENDING = 6
} Node127ControlState_t;

static volatile uint8_t g_node127_ctrl_state = NODE127_STATE_BOOT;
static uint8_t g_node127_calibrated = 0U;
static uint32_t g_node127_calib_start_tick = 0U;
static uint32_t g_node127_calib_count = 0U;
static uint32_t g_node127_calib_last_sample_tick = 0U;
static int32_t g_node127_gx_bias_sum = 0;
static float g_node127_gx_bias = 0.0f;


#define BMI088_SPI_TIMEOUT_MS      10U
#define BMI088_GYRO_CHIP_ID_VAL    0x0FU
#define BMI088_ACCEL_CHIP_ID_VAL   0x1EU

#define BMI088_REG_ACCEL_CHIP_ID   0x00U
#define BMI088_REG_ACCEL_X_LSB     0x12U
#define BMI088_REG_ACCEL_CONF      0x40U
#define BMI088_REG_ACCEL_RANGE     0x41U
#define BMI088_REG_ACCEL_PWR_CONF  0x7CU
#define BMI088_REG_ACCEL_PWR_CTRL  0x7DU

#define BMI088_REG_GYRO_CHIP_ID    0x00U
#define BMI088_REG_GYRO_X_LSB      0x02U
#define BMI088_REG_GYRO_RANGE      0x0FU
#define BMI088_REG_GYRO_BW         0x10U
#define BMI088_REG_GYRO_LPM1       0x11U

#define BMI088_ACCEL_SENS_6G       5460.0f
#define BMI088_GYRO_SENS_2000DPS   16.384f

static float interp_array_periodic_hermite(const float *x_key, const float *y_key, uint32_t point_num, float phase_pct);
static HAL_StatusTypeDef BMI088_Init(BMI088_t *imu);
static HAL_StatusTypeDef BMI088_Update(BMI088_t *imu);
static HAL_StatusTypeDef MT6701_Init(MT6701_t *enc);
static HAL_StatusTypeDef MT6701_Update(MT6701_t *enc);

static float clampf_local(float x, float x_min, float x_max) {
    if (x < x_min) return x_min;
    if (x > x_max) return x_max;
    return x;
}

static float Joint_ClampCurveDeg(const JointController_t *j, float deg);
static float ankle_wave_safe_deg_from_percent(float gait_percent);
static uint8_t Ankle_WaitForFeedback(uint8_t node_id, uint32_t timeout_ms);
static uint8_t Ankle_InitCANPositionMode(uint8_t node_id);
static uint8_t Ankle_HasFault(uint8_t node_id);
static void Ankle_HoldCurrent(uint8_t node_id);

/* --- 踝关节步态插值 --- */
static float ankle_wave_raw_from_percent(float gait_percent) {
    static const float gait_key[] = {0.0f, 6.0f, 10.0f, 16.0f, 22.0f, 28.0f, 35.0f, 45.0f, 52.0f, 56.0f, 62.0f, 68.0f, 75.0f, 82.0f, 90.0f, 96.0f, 100.0f};
    /*
 * 踝关节步态曲线：承重期轻度跖屈，支撑中期逐渐背屈，蹬地期明显跖屈，
 * 摆动期回到接近中立位。程序仍会减去 0% 相位值，避免首次触发跳变。
 */
    static const float ankle_key[] = {-2.0f, -10.0f, -15.0f, -9.0f, -4.0f, 1.0f, 6.0f, 10.0f, 8.0f, 0.0f, -14.0f, -20.0f, -13.0f, -7.0f, -3.0f, -1.0f, -2.0f};
    const uint32_t point_num = (uint32_t)(sizeof(gait_key) / sizeof(gait_key[0]));
    return interp_array_periodic_hermite(gait_key, ankle_key, point_num, gait_percent);
}

static float ankle_wave_deg_from_percent(float gait_percent) {
    float y_c = ankle_wave_raw_from_percent(gait_percent);
    float y_m = ankle_wave_raw_from_percent(gait_percent - PHASE_SMOOTH_PCT);
    float y_p = ankle_wave_raw_from_percent(gait_percent + PHASE_SMOOTH_PCT);
    return 0.25f * (y_m + 2.0f * y_c + y_p);
}

static float ankle_wave_safe_deg_from_percent(float gait_percent) {
    float cmd_deg = ANKLE_GAIT_OFFSET_DEG + ANKLE_WAVE_SCALE * ankle_wave_deg_from_percent(gait_percent);
    return clampf_local(cmd_deg, ANKLE_CMD_MIN_DEG, ANKLE_CMD_MAX_DEG);
}

/* --- 函数声明 --- */
static void Vofa_SendCanFrame(void);
static void Diag_SendAlive(uint32_t now, uint32_t stage_code);
static void Debug_WriteBufAll(const uint8_t *buf, uint16_t len, uint32_t timeout_ms);
static void Debug_Uart3Printf(const char *fmt, ...);
static void Debug_Uart3WriteRaw(const char *s);
static void Node127_SendSyncRequest(void);

/* ================= 运行所需的插值、关节和传感器函数 =================
 * 这些函数由应用层使用，必须在本文件中提供实现或声明。
 */
static void MT6701_Select(void)       { HAL_GPIO_WritePin(MT6701_CS_PORT, MT6701_CS_PIN, GPIO_PIN_RESET); }
static void MT6701_Deselect(void)     { HAL_GPIO_WritePin(MT6701_CS_PORT, MT6701_CS_PIN, GPIO_PIN_SET); }
static void MT6701_SckHigh(void)      { HAL_GPIO_WritePin(MT6701_SCK_PORT, MT6701_SCK_PIN, GPIO_PIN_SET); }
static void MT6701_SckLow(void)       { HAL_GPIO_WritePin(MT6701_SCK_PORT, MT6701_SCK_PIN, GPIO_PIN_RESET); }
static void MT6701_DelayShort(void)   { for (volatile uint32_t i = 0; i < 48U; ++i) { __NOP(); } }

static uint8_t MT6701_CRC6_FromData18(uint32_t data18)
{
#define MT6701_BIT(n) ((uint8_t)((data18 >> (n)) & 0x1U))
    uint8_t c0 = (uint8_t)(MT6701_BIT(17) ^ MT6701_BIT(16) ^ MT6701_BIT(15) ^ MT6701_BIT(12) ^ MT6701_BIT(10) ^ MT6701_BIT(6) ^ MT6701_BIT(5) ^ MT6701_BIT(0));
    uint8_t c1 = (uint8_t)(MT6701_BIT(15) ^ MT6701_BIT(13) ^ MT6701_BIT(12) ^ MT6701_BIT(11) ^ MT6701_BIT(10) ^ MT6701_BIT(7) ^ MT6701_BIT(5) ^ MT6701_BIT(1) ^ MT6701_BIT(0));
    uint8_t c2 = (uint8_t)(MT6701_BIT(16) ^ MT6701_BIT(14) ^ MT6701_BIT(13) ^ MT6701_BIT(12) ^ MT6701_BIT(11) ^ MT6701_BIT(8) ^ MT6701_BIT(6) ^ MT6701_BIT(2) ^ MT6701_BIT(1));
    uint8_t c3 = (uint8_t)(MT6701_BIT(17) ^ MT6701_BIT(15) ^ MT6701_BIT(14) ^ MT6701_BIT(13) ^ MT6701_BIT(12) ^ MT6701_BIT(9) ^ MT6701_BIT(7) ^ MT6701_BIT(3) ^ MT6701_BIT(2));
    uint8_t c4 = (uint8_t)(MT6701_BIT(16) ^ MT6701_BIT(15) ^ MT6701_BIT(14) ^ MT6701_BIT(13) ^ MT6701_BIT(10) ^ MT6701_BIT(8) ^ MT6701_BIT(4) ^ MT6701_BIT(3));
    uint8_t c5 = (uint8_t)(MT6701_BIT(17) ^ MT6701_BIT(16) ^ MT6701_BIT(15) ^ MT6701_BIT(14) ^ MT6701_BIT(11) ^ MT6701_BIT(9) ^ MT6701_BIT(5) ^ MT6701_BIT(4));
#undef MT6701_BIT
    return (uint8_t)((c5 << 5) | (c4 << 4) | (c3 << 3) | (c2 << 2) | (c1 << 1) | c0);
}

static uint32_t MT6701_ReadFrame_Falling(void)
{
    uint32_t frame = 0U;
    MT6701_SckHigh();
    MT6701_DelayShort();
    MT6701_Select();
    MT6701_DelayShort();
    for (uint8_t i = 0U; i < 24U; ++i) {
        MT6701_SckLow();
        MT6701_DelayShort();
        frame <<= 1;
        if (HAL_GPIO_ReadPin(MT6701_MISO_PORT, MT6701_MISO_PIN) == GPIO_PIN_SET) frame |= 0x01U;
        MT6701_SckHigh();
        MT6701_DelayShort();
    }
    MT6701_Deselect();
    MT6701_DelayShort();
    return frame;
}

static uint32_t MT6701_ReadFrame_Rising(void)
{
    uint32_t frame = 0U;
    MT6701_SckHigh();
    MT6701_DelayShort();
    MT6701_Select();
    MT6701_DelayShort();
    for (uint8_t i = 0U; i < 24U; ++i) {
        MT6701_SckLow();
        MT6701_DelayShort();
        MT6701_SckHigh();
        MT6701_DelayShort();
        frame <<= 1;
        if (HAL_GPIO_ReadPin(MT6701_MISO_PORT, MT6701_MISO_PIN) == GPIO_PIN_SET) frame |= 0x01U;
    }
    MT6701_Deselect();
    MT6701_DelayShort();
    return frame;
}

static uint32_t MT6701_ReadFrame_HighPhase(void)
{
    uint32_t frame = 0U;
    MT6701_SckHigh();
    MT6701_DelayShort();
    MT6701_Select();
    MT6701_DelayShort();
    for (uint8_t i = 0U; i < 24U; ++i) {
        frame <<= 1;
        if (HAL_GPIO_ReadPin(MT6701_MISO_PORT, MT6701_MISO_PIN) == GPIO_PIN_SET) frame |= 0x01U;
        MT6701_SckLow();
        MT6701_DelayShort();
        MT6701_SckHigh();
        MT6701_DelayShort();
    }
    MT6701_Deselect();
    MT6701_DelayShort();
    return frame;
}

static uint8_t MT6701_DecodeFrame(uint32_t frame, MT6701_t *enc)
{
    uint32_t data18;
    uint8_t status;
    uint8_t crc_rx;
    uint8_t crc_calc;

    if (enc == NULL) return 0U;

    data18 = (frame >> 6) & 0x3FFFFU;
    status = (uint8_t)(data18 & 0x0FU);
    crc_rx = (uint8_t)(frame & 0x3FU);
    crc_calc = MT6701_CRC6_FromData18(data18);

    enc->frame24 = frame;
    enc->angle_raw = (uint16_t)((data18 >> 4) & 0x3FFFU);
    enc->angle_deg = ((float)enc->angle_raw * 360.0f) / 16384.0f;
    enc->mag_field_state = (uint8_t)(status & 0x03U);
    enc->push_state = (uint8_t)((status >> 2) & 0x01U);
    enc->track_loss = (uint8_t)((status >> 3) & 0x01U);
    enc->crc_rx = crc_rx;
    enc->crc_calc = crc_calc;
    enc->read_ok = (uint8_t)((crc_rx == crc_calc) ? 1U : 0U);
    return enc->read_ok;
}

static HAL_StatusTypeDef MT6701_Init(MT6701_t *enc)
{
    if (enc == NULL) return HAL_ERROR;
    memset(enc, 0, sizeof(*enc));
    MT6701_Deselect();
    MT6701_SckHigh();
    HAL_Delay(5);
    return MT6701_Update(enc);
}

static HAL_StatusTypeDef MT6701_Update(MT6701_t *enc)
{
    uint32_t frame_falling;
    uint32_t frame_rising;
    uint32_t frame_highphase;

    if (enc == NULL) return HAL_ERROR;

    frame_falling = MT6701_ReadFrame_Falling();
    if (MT6701_DecodeFrame(frame_falling, enc) != 0U) return HAL_OK;

    frame_highphase = MT6701_ReadFrame_HighPhase();
    if (MT6701_DecodeFrame(frame_highphase, enc) != 0U) return HAL_OK;

    frame_rising = MT6701_ReadFrame_Rising();
    (void)MT6701_DecodeFrame(frame_rising, enc);

    return (enc->read_ok != 0U) ? HAL_OK : HAL_ERROR;
}

static void BMI088_SelectGyro(void)    { HAL_GPIO_WritePin(BMI088_GYRO_CS_PORT,  BMI088_GYRO_CS_PIN,  GPIO_PIN_RESET); }
static void BMI088_DeselectGyro(void)  { HAL_GPIO_WritePin(BMI088_GYRO_CS_PORT,  BMI088_GYRO_CS_PIN,  GPIO_PIN_SET); }
static void BMI088_SelectAccel(void)   { HAL_GPIO_WritePin(BMI088_ACCEL_CS_PORT, BMI088_ACCEL_CS_PIN, GPIO_PIN_RESET); }
static void BMI088_DeselectAccel(void) { HAL_GPIO_WritePin(BMI088_ACCEL_CS_PORT, BMI088_ACCEL_CS_PIN, GPIO_PIN_SET); }

static HAL_StatusTypeDef BMI088_GyroWriteReg(uint8_t reg, uint8_t value)
{
    uint8_t tx[2] = { (uint8_t)(reg & 0x7FU), value };
    HAL_StatusTypeDef status;
    BMI088_SelectGyro();
    status = HAL_SPI_Transmit(&hspi1, tx, 2U, BMI088_SPI_TIMEOUT_MS);
    BMI088_DeselectGyro();
    return status;
}

static HAL_StatusTypeDef BMI088_AccelWriteReg(uint8_t reg, uint8_t value)
{
    uint8_t tx[2] = { (uint8_t)(reg & 0x7FU), value };
    HAL_StatusTypeDef status;
    BMI088_SelectAccel();
    status = HAL_SPI_Transmit(&hspi1, tx, 2U, BMI088_SPI_TIMEOUT_MS);
    BMI088_DeselectAccel();
    return status;
}

static HAL_StatusTypeDef BMI088_GyroReadRegs(uint8_t reg, uint8_t *data, uint16_t len)
{
    uint8_t tx[16] = {0};
    uint8_t rx[16] = {0};
    HAL_StatusTypeDef status;

    if ((data == NULL) || (len == 0U) || (len > 14U)) return HAL_ERROR;
    tx[0] = (uint8_t)(reg | 0x80U);
    for (uint16_t i = 0; i < len; ++i) tx[i + 1U] = 0xFFU;
    BMI088_SelectGyro();
    status = HAL_SPI_TransmitReceive(&hspi1, tx, rx, (uint16_t)(len + 1U), BMI088_SPI_TIMEOUT_MS);
    BMI088_DeselectGyro();
    if (status != HAL_OK) return status;
    memcpy(data, &rx[1], len);
    return HAL_OK;
}

static HAL_StatusTypeDef BMI088_AccelReadRegs(uint8_t reg, uint8_t *data, uint16_t len)
{
    uint8_t tx[17] = {0};
    uint8_t rx[17] = {0};
    HAL_StatusTypeDef status;

    if ((data == NULL) || (len == 0U) || (len > 14U)) return HAL_ERROR;
    tx[0] = (uint8_t)(reg | 0x80U);
    for (uint16_t i = 0; i < (uint16_t)(len + 1U); ++i) tx[i + 1U] = 0xFFU;
    BMI088_SelectAccel();
    status = HAL_SPI_TransmitReceive(&hspi1, tx, rx, (uint16_t)(len + 2U), BMI088_SPI_TIMEOUT_MS);
    BMI088_DeselectAccel();
    if (status != HAL_OK) return status;
    memcpy(data, &rx[2], len);
    return HAL_OK;
}

static HAL_StatusTypeDef BMI088_Init(BMI088_t *imu)
{
    uint8_t dummy = 0U;

    if (imu == NULL) return HAL_ERROR;
    memset(imu, 0, sizeof(*imu));

    BMI088_DeselectGyro();
    BMI088_DeselectAccel();
    HAL_Delay(5);

    /* BMI088 上电默认按 I2C 模式工作，需要一次空读切换到 SPI。 */
    (void)BMI088_AccelReadRegs(BMI088_REG_ACCEL_CHIP_ID, &dummy, 1U);
    HAL_Delay(1);

    (void)BMI088_AccelReadRegs(BMI088_REG_ACCEL_CHIP_ID, &imu->accel_chip_id, 1U);
    (void)BMI088_GyroReadRegs(BMI088_REG_GYRO_CHIP_ID, &imu->gyro_chip_id, 1U);

    imu->accel_ready = (imu->accel_chip_id == BMI088_ACCEL_CHIP_ID_VAL) ? 1U : 0U;
    imu->gyro_ready  = (imu->gyro_chip_id  == BMI088_GYRO_CHIP_ID_VAL)  ? 1U : 0U;

    if (imu->accel_ready != 0U) {
        (void)BMI088_AccelWriteReg(BMI088_REG_ACCEL_PWR_CONF, 0x00U);
        HAL_Delay(1);
        (void)BMI088_AccelWriteReg(BMI088_REG_ACCEL_PWR_CTRL, 0x04U);
        HAL_Delay(2);
        (void)BMI088_AccelWriteReg(BMI088_REG_ACCEL_CONF, 0xA8U);
        (void)BMI088_AccelWriteReg(BMI088_REG_ACCEL_RANGE, 0x01U);
        HAL_Delay(2);
    }

    if (imu->gyro_ready != 0U) {
        HAL_Delay(30);
        (void)BMI088_GyroWriteReg(BMI088_REG_GYRO_RANGE, 0x00U);
        (void)BMI088_GyroWriteReg(BMI088_REG_GYRO_BW, 0x02U);
        (void)BMI088_GyroWriteReg(BMI088_REG_GYRO_LPM1, 0x00U);
        HAL_Delay(2);
    }

    imu->init_ok = (uint8_t)(((imu->accel_ready != 0U) && (imu->gyro_ready != 0U)) ? 1U : 0U);
    if (imu->init_ok != 0U) {
        (void)BMI088_Update(imu);
        return HAL_OK;
    }
    return HAL_ERROR;
}

static HAL_StatusTypeDef BMI088_Update(BMI088_t *imu)
{
    uint8_t raw[6] = {0};

    if ((imu == NULL) || (imu->init_ok == 0U)) return HAL_ERROR;

    if ((imu->gyro_ready != 0U) && (BMI088_GyroReadRegs(BMI088_REG_GYRO_X_LSB, raw, 6U) == HAL_OK)) {
        imu->gyro_raw[0] = (int16_t)(((uint16_t)raw[1] << 8) | raw[0]);
        imu->gyro_raw[1] = (int16_t)(((uint16_t)raw[3] << 8) | raw[2]);
        imu->gyro_raw[2] = (int16_t)(((uint16_t)raw[5] << 8) | raw[4]);
        imu->gyro_dps[0] = (float)imu->gyro_raw[0] / BMI088_GYRO_SENS_2000DPS;
        imu->gyro_dps[1] = (float)imu->gyro_raw[1] / BMI088_GYRO_SENS_2000DPS;
        imu->gyro_dps[2] = (float)imu->gyro_raw[2] / BMI088_GYRO_SENS_2000DPS;
    }

    if ((imu->accel_ready != 0U) && (BMI088_AccelReadRegs(BMI088_REG_ACCEL_X_LSB, raw, 6U) == HAL_OK)) {
        imu->accel_raw[0] = (int16_t)(((uint16_t)raw[1] << 8) | raw[0]);
        imu->accel_raw[1] = (int16_t)(((uint16_t)raw[3] << 8) | raw[2]);
        imu->accel_raw[2] = (int16_t)(((uint16_t)raw[5] << 8) | raw[4]);
        imu->accel_g[0] = (float)imu->accel_raw[0] / BMI088_ACCEL_SENS_6G;
        imu->accel_g[1] = (float)imu->accel_raw[1] / BMI088_ACCEL_SENS_6G;
        imu->accel_g[2] = (float)imu->accel_raw[2] / BMI088_ACCEL_SENS_6G;
    }

    return HAL_OK;
}

static float wrap_phase_pct(float phase_pct)
{
    while (phase_pct >= 100.0f) phase_pct -= 100.0f;
    while (phase_pct < 0.0f) phase_pct += 100.0f;
    return phase_pct;
}

static float interp_array_periodic_hermite(const float *x_key, const float *y_key, uint32_t point_num, float phase_pct)
{
    uint32_t n;
    uint32_t i1;
    uint32_t i2;
    uint32_t i0;
    uint32_t i3;
    float x1, x2, x, h, x0, x3;
    float y0, y1, y2, y3;
    float d10, d21;
    float m1, m2;
    float t, t2, t3;
    float h00, h10, h01, h11;

    if ((x_key == NULL) || (y_key == NULL) || (point_num == 0U)) return 0.0f;
    if (point_num < 3U) return y_key[0];

    n = point_num - 1U;
    if (n < 2U) return y_key[0];

    phase_pct = wrap_phase_pct(phase_pct);

    i1 = 0U;
    while (((i1 + 1U) < n) && (phase_pct >= x_key[i1 + 1U])) i1++;

    i2 = (i1 + 1U) % n;
    i0 = (i1 == 0U) ? (n - 1U) : (i1 - 1U);
    i3 = (i2 + 1U) % n;

    x1 = x_key[i1];
    x2 = (i2 > i1) ? x_key[i2] : (x_key[i2] + 100.0f);
    x  = (phase_pct < x1) ? (phase_pct + 100.0f) : phase_pct;
    h  = x2 - x1;
    if (h < 1e-6f) return y_key[i1];

    x0 = x_key[i0];
    if (x0 > x1) x0 -= 100.0f;
    x3 = x_key[i3];
    if (x3 < x2) x3 += 100.0f;

    y0 = y_key[i0]; y1 = y_key[i1]; y2 = y_key[i2]; y3 = y_key[i3];
    d10 = x2 - x0;
    d21 = x3 - x1;
    if ((d10 < 1e-6f) || (d21 < 1e-6f)) return y1 + (x - x1) * (y2 - y1) / h;

    m1 = ((y2 - y0) / d10) * (1.0f - CURVE_TENSION);
    m2 = ((y3 - y1) / d21) * (1.0f - CURVE_TENSION);

    t = (x - x1) / h;
    t2 = t * t;
    t3 = t2 * t;
    h00 = 2.0f * t3 - 3.0f * t2 + 1.0f;
    h10 = t3 - 2.0f * t2 + t;
    h01 = -2.0f * t3 + 3.0f * t2;
    h11 = t3 - t2;

    return h00 * y1 + h10 * h * m1 + h01 * y2 + h11 * h * m2;
}

static float interp_joint_periodic_hermite(const JointCurvePoint_t *curve, uint32_t count, float phase_pct)
{
    uint32_t n;
    uint32_t i1;
    uint32_t i2;
    uint32_t i0;
    uint32_t i3;
    float x1, x2, x, h, x0, x3;
    float y0, y1, y2, y3;
    float d10, d21;
    float m1, m2;
    float t, t2, t3;
    float h00, h10, h01, h11;

    if ((curve == NULL) || (count == 0U)) return 0.0f;
    if (count < 3U) return curve[0].joint_deg;

    n = count - 1U;
    if (n < 2U) return curve[0].joint_deg;

    phase_pct = wrap_phase_pct(phase_pct);

    i1 = 0U;
    while (((i1 + 1U) < n) && (phase_pct >= curve[i1 + 1U].phase_pct)) i1++;

    i2 = (i1 + 1U) % n;
    i0 = (i1 == 0U) ? (n - 1U) : (i1 - 1U);
    i3 = (i2 + 1U) % n;

    x1 = curve[i1].phase_pct;
    x2 = (i2 > i1) ? curve[i2].phase_pct : (curve[i2].phase_pct + 100.0f);
    x  = (phase_pct < x1) ? (phase_pct + 100.0f) : phase_pct;
    h  = x2 - x1;
    if (h < 1e-6f) return curve[i1].joint_deg;

    x0 = curve[i0].phase_pct;
    if (x0 > x1) x0 -= 100.0f;
    x3 = curve[i3].phase_pct;
    if (x3 < x2) x3 += 100.0f;

    y0 = curve[i0].joint_deg;
    y1 = curve[i1].joint_deg;
    y2 = curve[i2].joint_deg;
    y3 = curve[i3].joint_deg;
    d10 = x2 - x0;
    d21 = x3 - x1;
    if ((d10 < 1e-6f) || (d21 < 1e-6f)) return y1 + (x - x1) * (y2 - y1) / h;

    m1 = ((y2 - y0) / d10) * (1.0f - CURVE_TENSION);
    m2 = ((y3 - y1) / d21) * (1.0f - CURVE_TENSION);

    t = (x - x1) / h;
    t2 = t * t;
    t3 = t2 * t;
    h00 = 2.0f * t3 - 3.0f * t2 + 1.0f;
    h10 = t3 - 2.0f * t2 + t;
    h01 = -2.0f * t3 + 3.0f * t2;
    h11 = t3 - t2;

    return h00 * y1 + h10 * h * m1 + h01 * y2 + h11 * h * m2;
}

static void get_curve_target(const JointCurvePoint_t *curve, uint32_t count, float phase, float *deg)
{
    float y_c;
    float y_m;
    float y_p;

    if (deg == NULL) return;
    y_c = interp_joint_periodic_hermite(curve, count, phase);
    y_m = interp_joint_periodic_hermite(curve, count, phase - PHASE_SMOOTH_PCT);
    y_p = interp_joint_periodic_hermite(curve, count, phase + PHASE_SMOOTH_PCT);
    *deg = 0.25f * (y_m + 2.0f * y_c + y_p);
}

static float Joint_ClampCurveDeg(const JointController_t *j, float deg)
{
    if (j == NULL) return 0.0f;
    if (j->curve == kKneeCurve) return clampf_local(deg, KNEE_CMD_MIN_DEG, KNEE_CMD_MAX_DEG);
    if (j->curve == kHipCurve)  return clampf_local(deg, HIP_CMD_MIN_DEG, HIP_CMD_MAX_DEG);
    return deg;
}

static void Joint_Init(JointController_t *j, const char *name, uint16_t id, float dir, float ratio, const JointCurvePoint_t *c, uint32_t cnt)
{
    if (j == NULL) return;
    memset(j, 0, sizeof(*j));
    j->name = name;
    j->motor_id = id;
    j->dir_sign = dir;
    j->reduction_ratio = ratio;
    j->curve = c;
    j->point_count = cnt;
    j->phase_adv_state = PHASE_ADVANCE_PCT;
    j->slew_base_state = POS_SLEW_BASE_RAD_S_INIT;
    j->wff_alpha_state = WFF_LPF_ALPHA_INIT;
    j->soft_kp = KP_TARGET;
    j->soft_kw = KW_TARGET;
    j->traj_scale = 1.0f;
    j->last_servo_status = HAL_OK;
    j->cmd.id = id;
    j->cmd.mode = 0U;
    j->is_calibrated = 0U;
}

static float Joint_GetAngleScale(const JointController_t *j)
{
    if (j == NULL) return 1.0f;
    if (j->curve == kKneeCurve) return KNEE_JOINT_ANGLE_SCALE;
    if (j->curve == kHipCurve)  return HIP_JOINT_ANGLE_SCALE;
    return 1.0f;
}

static float Joint_GetMaxCmdSpeedDegS(const JointController_t *j)
{
    if (j == NULL) return 60.0f;
    if (j->curve == kKneeCurve) return KNEE_MAX_CMD_SPEED_DEG_S;
    if (j->curve == kHipCurve)  return HIP_MAX_CMD_SPEED_DEG_S;
    return 60.0f;
}

static void Joint_PrepareFollowRelative(JointController_t *j, float target_deg)
{
    float max_step_deg;

    if (j == NULL) return;
    target_deg = Joint_ClampCurveDeg(j, target_deg);
    max_step_deg = Joint_GetMaxCmdSpeedDegS(j) * ((float)CONTROL_PERIOD_MS * 0.001f);
    if (j->traj_started == 0U) {
        j->cmd_pos_state = 0.0f;
        j->traj_started = 1U;
    }
    if (target_deg > (j->cmd_pos_state + max_step_deg)) {
        j->cmd_pos_state += max_step_deg;
    } else if (target_deg < (j->cmd_pos_state - max_step_deg)) {
        j->cmd_pos_state -= max_step_deg;
    } else {
        j->cmd_pos_state = target_deg;
    }

    if (j->soft_kp < KP_TARGET) j->soft_kp += 0.002f;
    if (j->soft_kp > KP_TARGET) j->soft_kp = KP_TARGET;
    j->cmd.id = j->motor_id;
    j->cmd.mode = 1U;
    j->cmd.K_P = j->soft_kp;
    j->cmd.K_W = KW_TARGET;
    j->target_curve_deg = target_deg;
    j->target_ref_deg = target_deg;
    j->cmd.Pos = j->center_pos + j->dir_sign * (j->cmd_pos_state * DEG_TO_RAD) * j->reduction_ratio;
}

static void Joint_PrepareRunRelative(JointController_t *j, float phase_pct)
{
    float curve_deg;
    float zero_deg;

    if ((j == NULL) || (j->curve == NULL) || (j->point_count == 0U)) return;
    get_curve_target(j->curve, j->point_count, phase_pct + j->phase_adv_state, &curve_deg);
    get_curve_target(j->curve, j->point_count, GAIT_START_PHASE_PCT + j->phase_adv_state, &zero_deg);
    Joint_PrepareFollowRelative(j,
        (curve_deg - zero_deg) * Joint_GetAngleScale(j) * g_gait_amp_state);
    j->target_curve_deg = curve_deg - zero_deg;
}

static void Joint_PrepareHoldLast(JointController_t *j)
{
    if (j == NULL) return;

    j->cmd.id = j->motor_id;
    if (j->is_calibrated == 0U) {
        j->cmd.mode = 0U;      /* 还没收到回包时先不强行锁位置 */
        return;
    }

    if (g_node127_first_motion_seen == 0U) {
        /*
 * 上电且肌电没有明显变化前，不给宇树电机发送位置闭环命令。
 * 检测到主动运动意图后，才进入位置控制。
 */
        j->cmd.mode = 0U;
        j->cmd.K_P = 0.0f;
        j->cmd.K_W = 0.0f;
        return;
    }

    j->cmd.mode = 1U;
    j->cmd.K_P = KP_TARGET;
    j->cmd.K_W = KW_TARGET;

    if (j->traj_started == 0U) {
        j->target_ref_deg = 0.0f;
        j->target_curve_deg = 0.0f;
        j->cmd_pos_state = 0.0f;
        j->cmd.Pos = j->center_pos;
        j->traj_started = 1U;
    }
    /* 已经运行过时不重新计算目标，保持上一帧 cmd.Pos。 */
}

static void Joint_HandleResponse(JointController_t *j, HAL_StatusTypeDef status)
{
    if (j == NULL) return;
    j->last_servo_status = status;

    if ((status == HAL_OK) && (j->data.correct != 0U)) {
        j->fb_pos_state = j->data.Pos;
        j->fb_valid_state = 1U;
        if (j->is_calibrated == 0U) {
            j->center_pos = j->data.Pos;
            j->cmd_pos_state = 0.0f;
            j->target_ref_deg = 0.0f;
            j->target_curve_deg = 0.0f;
            j->is_calibrated = 1U;
        }
    }
}

static void Diag_SendAlive(uint32_t now, uint32_t stage_code)
{
    static char tx[384];
    int len;

    g_can2_esr_snapshot = (hcan2.Instance == CAN2) ? CAN2->ESR : 0U;
    g_can2_msr_snapshot = (hcan2.Instance == CAN2) ? CAN2->MSR : 0U;
    g_can2_state_snapshot = (uint32_t)hcan2.State;

    len = snprintf(tx, sizeof(tx),
                   "9001,%lu,%lu,%lu,%lu,%lu,%lu,%lu,%u,%lu,%lu,%lu,%lu,%lu,%lu,%lu,%lu,%u,%u,%lu,%lu,%lu,%lu,%lu,%u,%u\r\n",
                   (unsigned long)now,
                   (unsigned long)stage_code,
                   (unsigned long)g_node127_sync_count,
                   (unsigned long)g_can2_rx_count,
                   (unsigned long)g_can2_127_rx_count,
                   (unsigned long)g_can2_227_rx_count,
                   (unsigned long)g_can2_last_id,
                   (unsigned int)g_node127_sync_last_status,
                   (unsigned long)g_can2_error_code,
                   (unsigned long)g_can2_state_snapshot,
                   (unsigned long)g_can2_tx_ok_count,
                   (unsigned long)g_can2_tx_busy_count,
                   (unsigned long)g_can2_tx_fail_count,
                   (unsigned long)g_can2_tx_abort_count,
                   (unsigned long)g_can2_tx_mailbox_free,
                   (unsigned long)g_can2_busoff_count,
                   (unsigned int)g_ankle_init_step,
                   (unsigned int)g_ankle_init_result,
                   (unsigned long)g_boot_stage_code,
                   (unsigned long)SystemCoreClock,
                   (unsigned long)HAL_RCC_GetPCLK1Freq(),
                   (unsigned long)HAL_RCC_GetPCLK2Freq(),
                   (unsigned long)g_hardfault_count,
                   (unsigned int)g_node127_ctrl_state,
                   (unsigned int)g_node127_calibrated);
    if (len > 0) {
        if (len > (int)sizeof(tx)) len = (int)sizeof(tx);
        Debug_WriteBufAll((uint8_t *)tx, (uint16_t)len, 50U);
    }
}

static void Debug_WriteBufAll(const uint8_t *buf, uint16_t len, uint32_t timeout_ms)
{
    if ((buf == NULL) || (len == 0U)) return;

    if (huart2.Instance == USART2) {
        (void)HAL_UART_Transmit(&huart2, (uint8_t *)buf, len, timeout_ms);
    }
}

static void Debug_Uart3Printf(const char *fmt, ...)
{
    static char buf[256];
    int len;
    va_list ap;

    if (fmt == NULL) return;
    va_start(ap, fmt);
    len = vsnprintf(buf, sizeof(buf), fmt, ap);
    va_end(ap);
    if (len <= 0) return;
    if (len > (int)sizeof(buf)) len = (int)sizeof(buf);
    Debug_WriteBufAll((uint8_t *)buf, (uint16_t)len, 100U);
}

static void Debug_Uart3WriteRaw(const char *str)
{
    if (str == NULL) return;
    Debug_WriteBufAll((uint8_t *)str, (uint16_t)strlen(str), 100U);
}

static void Node127_SendSyncRequest(void)
{
    uint8_t sync_data[8] = {0};
    uint8_t st2;
    uint32_t now = HAL_GetTick();

    /* CAN2（PB5/PB6）固定连接 Node127，CAN1 固定控制踝关节电机。
 * 0x010 同步帧只允许从 CAN2 发送，避免干扰踝关节 CAN 总线。
 */
    st2 = can2_send_msg(NODE127_SYNC_CAN_ID, sync_data, 8U);
    g_node127_sync_last_status = st2;

    g_node127_sync_count++;
    g_node127_last_sync_tick = now;
}

static uint8_t Node127_DataFresh(uint32_t now)
{
    if (g_node127.gyro_valid == 0U) {
        return 0U;
    }
    return ((now - g_node127.gyro_tick) <= NODE127_DATA_TIMEOUT_MS) ? 1U : 0U;
}

static int16_t Node127_GetAbsPitchRateRaw(void)
{
    float x = (float)g_node127.gx - g_node127_gx_bias;
    x = fabsf(x);
    if (x > 32767.0f) x = 32767.0f;
    return (int16_t)x;
}

static float Node127_GetRelativePitchDeg(void)
{
    return THIGH_PITCH_SIGN * (((float)g_node127.pitch_q6 - g_node127_pitch_bias) / NODE127_PITCH_Q);
}

static void Node127_ClearIntent(void)
{
    g_node127_emg_intent = 0U;
    g_node127_emg_on_count = 0U;
    g_node127_emg_off_count = 0U;
    g_node127_emg_wait_release = 0U;
    g_node127_emg_intent_tick = 0U;
}

static void Node127_ResetCalibration(uint32_t now)
{
    uint8_t channel;
    g_node127_calibrated = 0U;
    g_node127_calib_start_tick = now;
    g_node127_calib_count = 0U;
    g_node127_calib_last_sample_tick = 0U;
    g_node127_gx_bias_sum = 0;
    g_node127_pitch_bias_sum = 0;
    g_node127_gx_bias = 0.0f;
    g_node127_pitch_bias = 0.0f;
    g_node127_emg_baseline_valid = 0U;
    g_node127_emg_baseline = 0.0f;
    g_node127_emg_norm = 0.0f;
    for (channel = 0U; channel < 3U; channel++) {
        g_node127_emg_baseline_ch[channel] = 0.0f;
        g_node127_emg_env_ch[channel] = 0.0f;
        g_node127_emg_norm_ch[channel] = 0.0f;
        g_node127_emg_calib_m2_ch[channel] = 0.0f;
        g_node127_emg_on_delta_ch[channel] = NODE127_EMG_ON_MIN_RAW;
        g_node127_emg_off_delta_ch[channel] = NODE127_EMG_OFF_MIN_RAW;
    }
    Node127_ClearIntent();
    g_node127_motion_active = 0U;
    g_node127_motion_level = 0.0f;
    g_node127_intent_level = 0.0f;
    g_node124_ignore_until_tick = now + NODE124_START_IGNORE_MS;
    g_node127_first_motion_seen = 0U;
    g_gait_amp_state = 0.0f;
    g_node127_phase_pct = GAIT_START_PHASE_PCT;
    g_node127_last_phase_pitch_deg = 0.0f;
    g_node127_last_phase_sequence = 0U;
    g_node127_last_imu_motion_tick = 0U;
    g_node127_ctrl_state = NODE127_STATE_CALIB;
}

/* Node127 静止标定只在收到 ID=127 时累计，掉线时保持原状态。 */
static uint8_t Node127_RunCalibration(uint32_t now)
{
    const float emg_raw[3] = {(float)g_node127.emg_front_uv, (float)g_node127.emg_lateral_uv, (float)g_node127.emg_rear_uv};
    float delta;
    float sigma;
    uint8_t channel;
    if (g_node127_calib_start_tick == 0U) Node127_ResetCalibration(now);
    if (!Node127_DataFresh(now)) { g_node127_ctrl_state = NODE127_STATE_LOST; return 0U; }
    if (g_node127.gyro_tick == g_node127_calib_last_sample_tick) return 0U;
    g_node127_calib_last_sample_tick = g_node127.gyro_tick;
    g_node127_ctrl_state = NODE127_STATE_CALIB;
    g_node127_gx_bias_sum += (int32_t)g_node127.gx;
    g_node127_pitch_bias_sum += (int32_t)g_node127.pitch_q6;
    g_node127_calib_count++;
    for (channel = 0U; channel < 3U; channel++) {
        delta = emg_raw[channel] - g_node127_emg_baseline_ch[channel];
        g_node127_emg_baseline_ch[channel] += delta / (float)g_node127_calib_count;
        g_node127_emg_calib_m2_ch[channel] += delta * (emg_raw[channel] - g_node127_emg_baseline_ch[channel]);
    }
    g_node127_emg_baseline_valid = 1U;
    g_node127_emg_baseline = g_node127_emg_baseline_ch[0];
    if (g_node127_emg_baseline_ch[1] > g_node127_emg_baseline) g_node127_emg_baseline = g_node127_emg_baseline_ch[1];
    if (g_node127_emg_baseline_ch[2] > g_node127_emg_baseline) g_node127_emg_baseline = g_node127_emg_baseline_ch[2];
    if (((now - g_node127_calib_start_tick) >= NODE127_CALIB_MS) && (g_node127_calib_count >= 20U)) {
        g_node127_gx_bias = (float)g_node127_gx_bias_sum / (float)g_node127_calib_count;
        g_node127_pitch_bias = (float)g_node127_pitch_bias_sum / (float)g_node127_calib_count;
        for (channel = 0U; channel < 3U; channel++) {
            sigma = sqrtf(g_node127_emg_calib_m2_ch[channel] / (float)(g_node127_calib_count - 1U));
            g_node127_emg_on_delta_ch[channel] = NODE127_EMG_ON_SIGMA * sigma;
            if (g_node127_emg_on_delta_ch[channel] < NODE127_EMG_ON_MIN_RAW) g_node127_emg_on_delta_ch[channel] = NODE127_EMG_ON_MIN_RAW;
            g_node127_emg_off_delta_ch[channel] = NODE127_EMG_OFF_SIGMA * sigma;
            if (g_node127_emg_off_delta_ch[channel] < NODE127_EMG_OFF_MIN_RAW) g_node127_emg_off_delta_ch[channel] = NODE127_EMG_OFF_MIN_RAW;
        }
        g_node127_last_phase_pitch_deg = 0.0f;
        g_node127_last_phase_sequence = g_node127.sample_sequence;
        g_node127_calibrated = 1U;
        g_node127_ctrl_state = NODE127_STATE_IDLE;
        g_node127_motion_active = 0U;
        g_node127_motion_level = 0.0f;
        g_node127_intent_level = 0.0f;
        Node127_ClearIntent();
        g_node124_ignore_until_tick = now + NODE124_START_IGNORE_MS;
        g_node127_first_motion_seen = 0U;
        return 1U;
    }
    return 0U;
}

static void Node127_EmgUpdate(uint32_t now)
{
    const float raw[3] = {(float)g_node127.emg_front_uv, (float)g_node127.emg_lateral_uv, (float)g_node127.emg_rear_uv};
    float diff;
    float active_raw;
    float alpha;
    uint8_t channel;
    uint8_t max_channel = 0U;

    if ((g_node127.gyro_valid == 0U) || ((now - g_node127.gyro_tick) > NODE127_EMG_TIMEOUT_MS)) {
        g_node127_emg_norm = 0.0f;
        for (channel = 0U; channel < 3U; channel++) { g_node127_emg_env_ch[channel] *= 0.82f; g_node127_emg_norm_ch[channel] = 0.0f; }
        return;
    }

    if (g_node127_emg_baseline_valid == 0U) {
        for (channel = 0U; channel < 3U; channel++) {
            g_node127_emg_baseline_ch[channel] = raw[channel];
            g_node127_emg_env_ch[channel] = 0.0f;
            g_node127_emg_norm_ch[channel] = 0.0f;
            g_node127_emg_on_delta_ch[channel] = NODE127_EMG_ON_MIN_RAW;
            g_node127_emg_off_delta_ch[channel] = NODE127_EMG_OFF_MIN_RAW;
        }
        g_node127_emg_baseline = raw[0];
        g_node127_emg_norm = 0.0f;
        g_node127_emg_baseline_valid = 1U;
        return;
    }

    for (channel = 0U; channel < 3U; channel++) {
        diff = fabsf(raw[channel] - g_node127_emg_baseline_ch[channel]);
        if ((g_node127_motion_active == 0U) && (g_node127_emg_intent == 0U) && (g_node127_emg_wait_release == 0U) && (diff < g_node127_emg_off_delta_ch[channel])) {
            g_node127_emg_baseline_ch[channel] += NODE127_EMG_BASELINE_ALPHA * (raw[channel] - g_node127_emg_baseline_ch[channel]);
            diff = fabsf(raw[channel] - g_node127_emg_baseline_ch[channel]);
        }
        alpha = (diff > g_node127_emg_env_ch[channel]) ? NODE127_EMG_ATTACK_ALPHA : NODE127_EMG_RELEASE_ALPHA;
        g_node127_emg_env_ch[channel] += alpha * (diff - g_node127_emg_env_ch[channel]);
        active_raw = g_node127_emg_env_ch[channel] - g_node127_emg_off_delta_ch[channel];
        if (active_raw < 0.0f) active_raw = 0.0f;
        g_node127_emg_norm_ch[channel] = clampf_local(active_raw / NODE127_EMG_FULL_SCALE_RAW, 0.0f, 1.0f);
        if (g_node127_emg_norm_ch[channel] > g_node127_emg_norm_ch[max_channel]) max_channel = channel;
    }

    g_node127_emg_norm = g_node127_emg_norm_ch[max_channel];
    g_node127_emg_baseline = g_node127_emg_baseline_ch[max_channel];
}
static float Node127_EmgNorm(uint32_t now)
{
    if ((g_node127.gyro_valid == 0U) || ((now - g_node127.gyro_tick) > NODE127_EMG_TIMEOUT_MS)) {
        return 0.0f;
    }
    return g_node127_emg_norm;
}

/* 返回 1 表示检测到腿部运动或主动肌电意图，否则返回 0。 */
static uint8_t Node127_UpdateMotionControl(uint32_t now)
{
    float gyro_abs;
    float gyro_active;
    float emg_active;
    float level;
    float intent_target;
    uint8_t any_on = 0U;
    uint8_t all_off = 1U;
    uint8_t channel;
    uint8_t imu_moving;
    uint8_t motion_allowed = 0U;
    if (!Node127_DataFresh(now)) {
        g_node127_ctrl_state = NODE127_STATE_LOST;
        g_node127_motion_active = 0U;
        g_node127_motion_level = 0.0f;
        g_node127_intent_level = 0.0f;
        Node127_ClearIntent();
        return 0U;
    }
    if (g_node127_calibrated == 0U) { (void)Node127_RunCalibration(now); return 0U; }
    Node127_EmgUpdate(now);
    gyro_abs = (float)Node127_GetAbsPitchRateRaw();
    gyro_active = clampf_local((gyro_abs - NODE127_GYRO_DEADBAND_RAW) / NODE127_GYRO_FULL_RAW, 0.0f, 1.0f);
    if (((g_node127.motion_flags & NODE127_IMU_MOVING_FLAG) != 0U) || (gyro_abs >= NODE127_GYRO_DEADBAND_RAW)) g_node127_last_imu_motion_tick = now;
    imu_moving = ((g_node127_last_imu_motion_tick != 0U) && ((now - g_node127_last_imu_motion_tick) <= NODE127_MOTION_HOLD_MS)) ? 1U : 0U;
    emg_active = Node127_EmgNorm(now);
    if ((int32_t)(now - g_node124_ignore_until_tick) < 0) {
        Node127_ClearIntent();
        g_node127_motion_active = 0U;
        g_node127_motion_level = 0.0f;
        g_node127_intent_level = 0.0f;
        g_node127_ctrl_state = NODE127_STATE_IDLE;
        return 0U;
    }
    for (channel = 0U; channel < 3U; channel++) {
        if (g_node127_emg_env_ch[channel] >= g_node127_emg_on_delta_ch[channel]) any_on = 1U;
        if (g_node127_emg_env_ch[channel] > g_node127_emg_off_delta_ch[channel]) all_off = 0U;
    }
    if (g_node127_emg_wait_release != 0U) {
        g_node127_emg_off_count = all_off ? (uint8_t)(g_node127_emg_off_count + 1U) : 0U;
        if (g_node127_emg_off_count >= NODE127_EMG_OFF_COUNT) Node127_ClearIntent();
    } else if (g_node127_emg_intent == 0U) {
        g_node127_emg_on_count = any_on ? (uint8_t)(g_node127_emg_on_count + 1U) : 0U;
        if (g_node127_emg_on_count >= NODE127_EMG_ON_COUNT) {
            g_node127_emg_intent = 1U;
            g_node127_emg_intent_tick = now;
            g_node127_emg_on_count = 0U;
        }
    } else {
        g_node127_emg_off_count = all_off ? (uint8_t)(g_node127_emg_off_count + 1U) : 0U;
        if (g_node127_emg_off_count >= NODE127_EMG_OFF_COUNT) { g_node127_emg_intent = 0U; g_node127_emg_off_count = 0U; }
    }
    intent_target = (g_node127_emg_intent != 0U) ? emg_active : 0.0f;
    if ((g_node127_emg_intent != 0U) && (intent_target < 0.17f)) intent_target = 0.17f;
    if (g_node127_intent_level < intent_target) g_node127_intent_level += NODE127_INTENT_ATTACK_ALPHA * (intent_target - g_node127_intent_level);
    else g_node127_intent_level += NODE127_INTENT_RELEASE_ALPHA * (intent_target - g_node127_intent_level);
    if (g_node127_ctrl_state == NODE127_STATE_MOVE) {
        if ((g_node127_emg_intent != 0U) || (imu_moving != 0U)) motion_allowed = 1U;
    } else if (g_node127_emg_intent != 0U) {
        if (imu_moving != 0U) { motion_allowed = 1U; g_node127_first_motion_seen = 1U; }
        else if ((now - g_node127_emg_intent_tick) <= NODE127_INTENT_WAIT_IMU_MS) {
            g_node127_ctrl_state = NODE127_STATE_INTENT_PENDING;
            g_node127_motion_active = 0U;
            g_node127_motion_level = 0.0f;
            return 0U;
        } else {
            g_node127_emg_intent = 0U;
            g_node127_emg_wait_release = 1U;
            g_node127_emg_off_count = 0U;
        }
    }
    if (motion_allowed == 0U) {
        g_node127_motion_active = 0U;
        g_node127_motion_level = 0.0f;
        g_node127_ctrl_state = g_node127_first_motion_seen ? NODE127_STATE_HOLD : NODE127_STATE_IDLE;
        return 0U;
    }
    level = 0.90f * g_node127_intent_level + 0.10f * gyro_active;
    if (level < 0.35f) level = 0.35f;
    g_node127_motion_level = clampf_local(level, 0.0f, 1.0f);
    g_node127_motion_active = 1U;
    g_node127_ctrl_state = NODE127_STATE_MOVE;
    return 1U;
}
static uint8_t Ankle_WaitForFeedback(uint8_t node_id, uint32_t timeout_ms)
{
    uint32_t t0;

    if (node_id >= 6U) {
        return 0U;
    }

    t0 = HAL_GetTick();
    while ((HAL_GetTick() - t0) < timeout_ms) {
        static uint32_t wait_last_sync = 0U;
        uint32_t now = HAL_GetTick();
        USER_CAN_PollRx();
        if ((now - wait_last_sync) >= NODE127_SYNC_PERIOD_MS) {
            wait_last_sync = now;
            Node127_SendSyncRequest();
        }
        if (motor_fb_valid[node_id] != 0U) {
            return 1U;
        }
    }

    return 0U;
}

static uint8_t Ankle_HasFault(uint8_t node_id)
{
    if (node_id >= 6U) {
        return 1U;
    }

    /* Heartbeat Flags: bit0 motor error, bit1 encoder error, bit2 controller error,
     * bit3 system error. Axis_Error non-zero also means the drive refused or left
     * closed loop.
     */
    if (motor_axis_error[node_id] != 0U) {
        return 1U;
    }
    if ((motor_axis_flags[node_id] & 0x0FU) != 0U) {
        return 1U;
    }
    return 0U;
}

static void Ankle_HoldCurrent(uint8_t node_id)
{
    uint32_t now;
    static uint32_t last_enable_tick[6] = {0U};

    if (node_id >= 6U) {
        return;
    }

    now = HAL_GetTick();

    /* Re-send the enable sequence periodically. Encoder feedback only proves that
     * CAN RX works; it does NOT prove the drive entered closed loop. Re-sending
     * the mode/state commands prevents the motor from staying idle after a missed
     * or refused first enable command.
     */
    if ((now - last_enable_tick[node_id]) >= ANKLE_ENABLE_RETRY_MS) {
        last_enable_tick[node_id] = now;

        /* If the drive is currently idle, refresh the hold target from the latest
         * measured position before requesting closed loop. This prevents the motor
         * from kicking toward an old target when it re-enables.
         */
        if ((motor_axis_state[node_id] != 8U) && (motor_fb_valid[node_id] != 0U)) {
            motor[node_id].init_pos = motor[node_id].pos;
        }

        g_ankle_init_step = 8U;
        g_ankle_last_tx_status = can_set_controller_mode(node_id, ANKLE_CAN_CONTROL_MODE, ANKLE_CAN_INPUT_MODE);
        HAL_Delay(1);
        g_ankle_last_tx_status = can_set_input_pos(node_id, motor[node_id].init_pos, 0, 0);
        HAL_Delay(1);
        g_ankle_last_tx_status = can_set_axis_state(node_id, 8U);
    }

    g_ankle_cmd_deg = 0.0f;
    g_ankle_cmd_turn = motor[node_id].init_pos;
    g_ankle_cmd_turn_state = motor[node_id].init_pos;
    g_ankle_cmd_turn_state_valid = 1U;
    g_ankle_gait_enable = 0U;
    (void)can_set_input_pos(node_id, motor[node_id].init_pos, 0, 0);
}


static void Ankle_IdleBeforeFirstEmg(uint8_t node_id)
{
    static uint32_t last_idle_tick[6] = {0U};
    uint32_t now;

    if (node_id >= 6U) {
        return;
    }

    now = HAL_GetTick();

    /*
 * 上电且未检测到明显肌电前，踝关节不主动进入位置闭环。
 * 只记录当前位置，作为后续控制的初始参考。
 */
    if (motor_fb_valid[node_id] != 0U) {
        motor[node_id].init_pos = motor[node_id].pos;
        g_ankle_cmd_turn_state = motor[node_id].pos;
        g_ankle_cmd_turn_state_valid = 1U;
        g_ankle_cmd_turn = motor[node_id].pos;
    }
    g_ankle_cmd_deg = 0.0f;
    g_ankle_gait_enable = 0U;

    /* 如果初始化时已经进入闭环，先回到 idle，等待下一次肌电触发。 */
    if ((now - last_idle_tick[node_id]) >= 500U) {
        last_idle_tick[node_id] = now;
        (void)can_set_axis_state(node_id, 1U);
    }
}


static void Ankle_HoldLastTarget(uint8_t node_id)
{
    float hold_turn;
    if (node_id >= 6U) {
        return;
    }

    if (g_ankle_cmd_turn_state_valid != 0U) {
        hold_turn = g_ankle_cmd_turn_state;
    } else {
        hold_turn = motor[node_id].init_pos;
        g_ankle_cmd_turn_state = hold_turn;
        g_ankle_cmd_turn_state_valid = 1U;
    }

    g_ankle_gait_enable = 0U;
    g_ankle_cmd_turn = hold_turn;
    (void)can_set_input_pos(node_id, hold_turn, 0, 0);
}

static uint8_t Ankle_InitCANPositionMode(uint8_t node_id)
{
    uint8_t tx;
    uint8_t attempt;

    if (node_id >= 6U) {
        g_ankle_init_result = 10U;
        return 0U;
    }

    g_ankle_init_result = 0U;
    g_ankle_init_step = 1U;
    g_ankle_last_tx_status = 0U;

    motor_fb_valid[node_id] = 0U;
    motor_hb_valid[node_id] = 0U;
    motor_axis_error[node_id] = 0U;
    motor_axis_flags[node_id] = 0U;
    motor_axis_state[node_id] = 0U;

    /* Clear errors and request idle first. */
    g_ankle_init_step = 2U;
    tx = can_clear_errors(node_id);
    g_ankle_last_tx_status = tx;
    HAL_Delay(20);

    g_ankle_init_step = 3U;
    tx = can_set_axis_state(node_id, 1U);
    g_ankle_last_tx_status = tx;
    HAL_Delay(50);

    /* Use practical hold-control parameters first. 2A/very-low gain can feel like
     * no control on a reducer, so this version uses a visible holding stiffness.
     */
    g_ankle_init_step = 4U;
    g_ankle_last_tx_status = can_set_limits(node_id, ANKLE_SAFE_VEL_LIMIT_TURN_S, ANKLE_SAFE_CURRENT_LIMIT_A);
    HAL_Delay(10);
    g_ankle_last_tx_status = can_set_pos_gain(node_id, ANKLE_POS_GAIN);
    HAL_Delay(10);
    g_ankle_last_tx_status = can_set_vel_gains(node_id, ANKLE_VEL_GAIN, ANKLE_VEL_INTEGRATOR_GAIN);
    HAL_Delay(10);

    /* Encoder feedback only means the node ID/baud rate are correct. */
    g_ankle_init_step = 5U;
    if (Ankle_WaitForFeedback(node_id, ANKLE_FEEDBACK_TIMEOUT_MS) == 0U) {
        g_ankle_init_result = 20U;   /* no encoder feedback */
        return 0U;
    }

#if ANKLE_ZERO_BY_LINEAR_COUNT
    set_linear_count(node_id, 0);
    HAL_Delay(50);
    motor[node_id].init_pos = 0.0f;
#else
    /* Do not change the drive's encoder count during debug; simply hold the
     * current reported position. This avoids a wrong zeroing command making the
     * target different from the actual pose.
     */
    motor[node_id].init_pos = motor[node_id].pos;
#endif
    g_ankle_cmd_turn = motor[node_id].init_pos;
    g_ankle_cmd_turn_state = motor[node_id].init_pos;
    g_ankle_cmd_turn_state_valid = 1U;

    g_ankle_init_step = 6U;
    tx = can_set_controller_mode(node_id, ANKLE_CAN_CONTROL_MODE, ANKLE_CAN_INPUT_MODE);  /* position + filtered position */
    g_ankle_last_tx_status = tx;
    HAL_Delay(20);

    g_ankle_init_step = 7U;
    tx = can_set_input_pos(node_id, motor[node_id].init_pos, 0, 0);
    g_ankle_last_tx_status = tx;
    HAL_Delay(20);

    /* Try several times and wait for heartbeat confirmation. If heartbeat is not
     * enabled on the drive, still return ready after successful CAN TX + encoder
     * feedback, and main loop will keep re-sending axis_state=8 periodically.
     */
    for (attempt = 0U; attempt < ANKLE_ENABLE_ATTEMPTS; attempt++) {
        g_ankle_init_step = 8U;
        tx = can_set_axis_state(node_id, 8U);
        g_ankle_last_tx_status = tx;
        HAL_Delay(25);
        (void)can_set_input_pos(node_id, motor[node_id].init_pos, 0, 0);
        HAL_Delay(75);

        if (motor_hb_valid[node_id] != 0U) {
            if ((motor_axis_state[node_id] == 8U) && (Ankle_HasFault(node_id) == 0U)) {
                g_ankle_init_result = 1U;    /* confirmed closed loop */
                return 1U;
            }

            if (Ankle_HasFault(node_id) != 0U) {
                g_ankle_init_result = 30U;   /* heartbeat reports a drive fault */
                return 0U;
            }
        }
    }

    if (motor_hb_valid[node_id] == 0U) {
        g_ankle_init_result = 31U;           /* no heartbeat, but encoder exists */
        return 1U;
    }

    /* Heartbeat exists but the drive never reported state 8. Do not force idle;
     * keep ready so the main loop can keep trying to enter closed loop and print
     * axis_state/axis_error for diagnosis.
     */
    g_ankle_init_result = 32U;
    return 1U;
}

/* Application runtime state. */
static JointController_t knee;
static JointController_t hip;
static BMI088_t imu;
static MT6701_t enc;
static uint32_t last_tick;
static uint32_t last_print;
static uint32_t run_start_tick;
static uint32_t last_node127_sync_tick;
static uint32_t last_diag_tick;
static uint8_t ankle_can_ready;

typedef struct {
    MOTOR_send cmd;
    MOTOR_recv data;
    float center_pos;
    float cmd_deg;
    uint32_t last_tick;
    uint8_t calibrated;
} JointMotorTest_t;

static JointMotorTest_t g_joint_motor_test[2];

void App_TestJointMotor(uint16_t motor_id, float angle_deg, float speed_deg_s)
{
    JointMotorTest_t *test;
    float angle_min;
    float angle_max;
    float reduction_ratio;
    float max_step_deg;
    uint32_t now;
    HAL_StatusTypeDef status;

    if (motor_id > HIP_MOTOR_ID) return;
    test = &g_joint_motor_test[motor_id];
    angle_min = (motor_id == KNEE_MOTOR_ID) ? KNEE_CMD_MIN_DEG : HIP_CMD_MIN_DEG;
    angle_max = (motor_id == KNEE_MOTOR_ID) ? KNEE_CMD_MAX_DEG : HIP_CMD_MAX_DEG;
    reduction_ratio = (motor_id == KNEE_MOTOR_ID) ? KNEE_REDUCTION_RATIO : HIP_REDUCTION_RATIO;
    angle_deg = clampf_local(angle_deg, angle_min, angle_max);
    speed_deg_s = clampf_local(speed_deg_s, 0.0f,
                               (motor_id == KNEE_MOTOR_ID) ? KNEE_MAX_CMD_SPEED_DEG_S : HIP_MAX_CMD_SPEED_DEG_S);

    test->cmd.id = motor_id;
    if (test->calibrated == 0U) {
        test->cmd.mode = 0U;
        status = SERVO_Send_recv(&test->cmd, &test->data);
        if ((status == HAL_OK) && (test->data.correct != 0U)) {
            test->center_pos = test->data.Pos;
            test->cmd_deg = 0.0f;
            test->last_tick = HAL_GetTick();
            test->calibrated = 1U;
        }
        return;
    }

    now = HAL_GetTick();
    max_step_deg = speed_deg_s * (float)(now - test->last_tick) * 0.001f;
    test->last_tick = now;
    if (angle_deg > (test->cmd_deg + max_step_deg)) test->cmd_deg += max_step_deg;
    else if (angle_deg < (test->cmd_deg - max_step_deg)) test->cmd_deg -= max_step_deg;
    else test->cmd_deg = angle_deg;

    test->cmd.mode = 1U;
    test->cmd.K_P = KP_TARGET;
    test->cmd.K_W = KW_TARGET;
    test->cmd.W = 0.0f;
    test->cmd.T = 0.0f;
    test->cmd.Pos = test->center_pos + test->cmd_deg * DEG_TO_RAD * reduction_ratio;
    (void)SERVO_Send_recv(&test->cmd, &test->data);
}
/* ==================================================================
 * 应用主循环逻辑
 * ================================================================== */
void App_Init(void) {
    /*
 * 大型运行状态放在静态区，避免占用栈空间导致 HardFault。
 */

    memset(&imu, 0, sizeof(imu));
    memset(&enc, 0, sizeof(enc));
    g_boot_stage_code = 5U;
    Debug_Uart3WriteRaw("BOOT0: debug alive on USART3 PB10 and USART2 PA2, 115200, 8N1\r\n");
    Debug_Uart3Printf("BOOT1: SYSCLK=%lu PCLK1=%lu PCLK2=%lu\r\n", (unsigned long)SystemCoreClock, (unsigned long)HAL_RCC_GetPCLK1Freq(), (unsigned long)HAL_RCC_GetPCLK2Freq());
    HAL_Delay(50);

    /*
 * 先初始化 CAN1 和 CAN2，再通过 CAN2 持续发送 0x010 同步帧。
 * 即使 Node127 未连接，USART3 仍会持续输出诊断信息。
 */
    USER_CAN2_Start();   /* 127模块：CAN2 PB5/PB6 */
    g_boot_stage_code = 10U;
    USER_CAN1_Start();   /* 踝关节西格玛电机：CAN1 PB8/PB9 */
    g_boot_stage_code = 11U;
    Debug_Uart3Printf("BOOT: CAN1/CAN2 start called, no blocking warmup. USART3 alive.\r\n");
    g_node127_warmup_done = 1U;

    Diag_SendAlive(HAL_GetTick(), 10U);
    (void)BMI088_Init(&imu);
    g_boot_stage_code = 12U;
    Diag_SendAlive(HAL_GetTick(), 11U);
    (void)MT6701_Init(&enc);
    g_boot_stage_code = 13U;
    Diag_SendAlive(HAL_GetTick(), 12U);

    HAL_Delay(50);

    /*
 * 踝关节只有一个 CAN 电机，不向不存在的节点批量发送命令。
 */
    ankle_can_ready = Ankle_InitCANPositionMode(ANKLE_MOTOR_ID);
    g_boot_stage_code = 14U;
    Debug_Uart3Printf("BOOT: ankle init result=%u, ready=%u\r\n", (unsigned int)g_ankle_init_result, (unsigned int)ankle_can_ready);

    // 统一初始化运行状态。
    Joint_Init(&knee, "knee", KNEE_MOTOR_ID, KNEE_DIR_SIGN, KNEE_REDUCTION_RATIO, kKneeCurve, KNEE_POINT_COUNT);
    Joint_Init(&hip, "hip", HIP_MOTOR_ID, HIP_DIR_SIGN, HIP_REDUCTION_RATIO, kHipCurve, HIP_POINT_COUNT);

    /* Node127 从静止标定开始；标定完成或数据掉线时，三个电机只保持。 */
    Node127_ResetCalibration(HAL_GetTick());

    run_start_tick = HAL_GetTick();
    last_tick = run_start_tick;
    last_print = 0U;
    last_node127_sync_tick = run_start_tick;
    last_diag_tick = 0U;

}

void App_RunOnce(void) {
    uint32_t now = HAL_GetTick();
    uint32_t elapsed = now - run_start_tick;

    /* USART3 生命信号：即使传感器或电机未连接，也持续输出诊断信息。 */
    if ((now - last_diag_tick) >= 200U) {
        last_diag_tick = now;
        Diag_SendAlive(now, 20U);
    }

    /* CAN2 同时使用中断和主循环轮询，避免接收链路异常导致数据丢失。 */
    USER_CAN_PollRx();

    if ((now - last_node127_sync_tick) >= NODE127_SYNC_PERIOD_MS) {
        last_node127_sync_tick = now;
        Node127_SendSyncRequest();
    }
    if (now - last_tick >= CONTROL_PERIOD_MS) {
        last_tick = now;

        if (elapsed < INIT_HOLD_MS) {
            /* 上电前 1.5 s 只保持当前位置，不运行步态曲线。 */
            knee.cmd.mode = 0U;
            Joint_HandleResponse(&knee, SERVO_Send_recv(&knee.cmd, &knee.data));

            hip.cmd.mode = 0U;
            Joint_HandleResponse(&hip, SERVO_Send_recv(&hip.cmd, &hip.data));

            if (ankle_can_ready != 0U) {
                /* 上电静止阶段只记录当前位置，不让踝关节主动闭环动作。 */
                Ankle_IdleBeforeFirstEmg(ANKLE_MOTOR_ID);
            }
        } else {
            uint8_t node127_motion = Node127_UpdateMotionControl(now);

            if (node127_motion != 0U) {
                float dt_s = (float)CONTROL_PERIOD_MS * 0.001f;
                float amp_target;
                float amp_step;
                float motion_drive;
                float thigh_pitch_deg;
                float pitch_delta_deg;

                /* Pitch drives the hip directly; accumulated thigh travel drives knee/ankle phase. */
                motion_drive = clampf_local((0.75f * g_node127_intent_level) + (0.25f * g_node127_motion_level), 0.0f, 1.0f);
                amp_target = GAIT_AMP_MIN_SCALE +
                    (GAIT_AMP_MAX_SCALE - GAIT_AMP_MIN_SCALE) * motion_drive;
                amp_target = clampf_local(amp_target, GAIT_AMP_MIN_SCALE, GAIT_AMP_MAX_SCALE);
                amp_step = GAIT_AMP_RAMP_UP_PER_S * dt_s;
                if (g_gait_amp_state < amp_target) {
                    g_gait_amp_state += amp_step;
                    if (g_gait_amp_state > amp_target) g_gait_amp_state = amp_target;
                } else {
                    /* 运动意图仍存在但强度降低时，步态幅值逐渐减小，避免突降。 */
                    g_gait_amp_state -= (0.35f * amp_step);
                    if (g_gait_amp_state < amp_target) g_gait_amp_state = amp_target;
                }

                thigh_pitch_deg = Node127_GetRelativePitchDeg();
                if (g_node127.sample_sequence != g_node127_last_phase_sequence) {
                    pitch_delta_deg = fabsf(thigh_pitch_deg - g_node127_last_phase_pitch_deg);
                    if (pitch_delta_deg > THIGH_PHASE_MAX_DELTA_DEG) {
                        pitch_delta_deg = THIGH_PHASE_MAX_DELTA_DEG;
                    }
                    if (pitch_delta_deg >= THIGH_PHASE_NOISE_DEG) {
                        g_node127_phase_pct = wrap_phase_pct(
                            g_node127_phase_pct + pitch_delta_deg * THIGH_PHASE_PCT_PER_DEG);
                    }
                    g_node127_last_phase_pitch_deg = thigh_pitch_deg;
                    g_node127_last_phase_sequence = g_node127.sample_sequence;
                }

                Joint_PrepareRunRelative(&knee, g_node127_phase_pct);
                Joint_HandleResponse(&knee, SERVO_Send_recv(&knee.cmd, &knee.data));

                Joint_PrepareFollowRelative(&hip, thigh_pitch_deg * THIGH_PITCH_GAIN);
                Joint_HandleResponse(&hip, SERVO_Send_recv(&hip.cmd, &hip.data));
            } else {
                {
                    float dt_s = (float)CONTROL_PERIOD_MS * 0.001f;
                    float amp_decay_step = GAIT_AMP_DECAY_PER_S * dt_s;
                    if (g_gait_amp_state > amp_decay_step) g_gait_amp_state -= amp_decay_step;
                    else g_gait_amp_state = 0.0f;
                }
                /* 127 data lost or human stopped: phase freezes; gait drive decays. */
                Joint_PrepareHoldLast(&knee);
                Joint_HandleResponse(&knee, SERVO_Send_recv(&knee.cmd, &knee.data));

                Joint_PrepareHoldLast(&hip);
                Joint_HandleResponse(&hip, SERVO_Send_recv(&hip.cmd, &hip.data));
            }

            if (ankle_can_ready != 0U) {
                if (Ankle_HasFault(ANKLE_MOTOR_ID) != 0U) {
                    (void)can_set_axis_state(ANKLE_MOTOR_ID, 1U);
                    ankle_can_ready = 0U;
                }
                else if ((g_node127_first_motion_seen == 0U) && (node127_motion == 0U)) {
                    /* 肌电未明显变化前，踝关节保持 idle，不主动寻找位置。 */
                    Ankle_IdleBeforeFirstEmg(ANKLE_MOTOR_ID);
                }
                else if ((ANKLE_HOLD_ONLY_TEST != 0U) || (elapsed < (INIT_HOLD_MS + ANKLE_PRE_GAIT_HOLD_MS))) {
                    Ankle_HoldCurrent(ANKLE_MOTOR_ID);
                }
                else if ((motor_hb_valid[ANKLE_MOTOR_ID] != 0U) && (motor_axis_state[ANKLE_MOTOR_ID] != 8U)) {
                    Ankle_HoldCurrent(ANKLE_MOTOR_ID);
                }
                else if (node127_motion != 0U) {
                    float ankle_curve_deg = ankle_wave_safe_deg_from_percent(g_node127_phase_pct);
                    float ankle_curve_zero_deg = ankle_wave_safe_deg_from_percent(GAIT_START_PHASE_PCT);
                    float ankle_deg = NODE127_GAIT_SCALE * ANKLE_JOINT_ANGLE_SCALE * g_gait_amp_state * (ankle_curve_deg - ankle_curve_zero_deg);
                    float ankle_turn_target = motor[ANKLE_MOTOR_ID].init_pos + ANKLE_CMD_DIR * (ankle_deg * ANKLE_TURN_PER_DEG);
                    float ankle_turn;
                    float max_step_turn = ANKLE_MAX_CMD_SPEED_DEG_S * ANKLE_TURN_PER_DEG * ((float)CONTROL_PERIOD_MS / 1000.0f);

                    if (g_ankle_cmd_turn_state_valid == 0U) {
                        g_ankle_cmd_turn_state = motor[ANKLE_MOTOR_ID].init_pos;
                        g_ankle_cmd_turn_state_valid = 1U;
                    }

                    if (ankle_turn_target > (g_ankle_cmd_turn_state + max_step_turn)) {
                        ankle_turn = g_ankle_cmd_turn_state + max_step_turn;
                    } else if (ankle_turn_target < (g_ankle_cmd_turn_state - max_step_turn)) {
                        ankle_turn = g_ankle_cmd_turn_state - max_step_turn;
                    } else {
                        ankle_turn = ankle_turn_target;
                    }
                    g_ankle_cmd_turn_state = ankle_turn;

                    g_ankle_cmd_deg = ankle_deg;
                    g_ankle_cmd_turn = ankle_turn;
                    g_ankle_gait_enable = 1U;
                    (void)can_set_input_pos(ANKLE_MOTOR_ID, ankle_turn, 0, 0);
                }
                else {
                    /* 无运动时保持上一目标，不回初始位置，也不继续运行曲线。 */
                    Ankle_HoldLastTarget(ANKLE_MOTOR_ID);
                }
            }
        }

        if (now - last_print >= PRINT_PERIOD_MS) {
            last_print = now;

            (void)BMI088_Update(&imu);
            (void)MT6701_Update(&enc);
            Vofa_SendCanFrame();
        }
    }
}

static void Vofa_SendCanFrame(void) {
    static char tx[96];
    uint16_t emg_front;
    uint16_t emg_lateral;
    uint16_t emg_rear;
    int16_t pitch_q6;
    int16_t gx;
    uint16_t motion_flags;
    uint32_t primask;
    uint8_t updated;
    int len;

    primask = __get_PRIMASK();
    __disable_irq();
    updated = g_node127.updated;
    emg_front = g_node127.emg_front_uv;
    emg_lateral = g_node127.emg_lateral_uv;
    emg_rear = g_node127.emg_rear_uv;
    pitch_q6 = g_node127.pitch_q6;
    gx = g_node127.gx;
    motion_flags = g_node127.motion_flags;
    g_node127.updated = 0U;
    if (primask == 0U) {
        __enable_irq();
    }

    if ((updated == 0U) || (huart3.Instance != USART3)) return;

    /* VOFA FireWater: front/lateral/rear EMG, pitch Q6, gyro X Q6, flags. */
    len = snprintf(tx, sizeof(tx), "%u,%u,%u,%d,%d,%d\r\n",
                   (unsigned int)emg_front,
                   (unsigned int)emg_lateral,
                   (unsigned int)emg_rear,
                   (int)pitch_q6, (int)gx, (int)motion_flags);
    if (len > 0) {
        if (len >= (int)sizeof(tx)) len = (int)sizeof(tx) - 1;
        (void)HAL_UART_Transmit(&huart3, (uint8_t *)tx, (uint16_t)len, 30U);
    }
}

/* --- 底层硬件配置（基于 160 MHz） --- */
void App_ErrorHandlerStep(void)
{
    if (huart3.Instance == USART3) {
        const char *error_text = "9999,ERROR_HANDLER\\r\\n";
        Debug_WriteBufAll((uint8_t *)error_text, (uint16_t)strlen(error_text), 50U);
    }
    HAL_Delay(200);
}
