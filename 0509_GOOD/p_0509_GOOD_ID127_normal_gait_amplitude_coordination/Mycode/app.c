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

/* --- 寮曞叆浣犵殑 CAN 鐢垫満搴曞眰搴?--- */
#include "can_driver.h"       
#include "control.h"
#include "getdata.h"
#include "ankle_can_config.h"

/* --- 纭欢 ID 涓庡弬鏁伴厤缃?--- */
#define KNEE_MOTOR_ID              0U        // 鑶濆叧鑺?485 ID
#define HIP_MOTOR_ID               1U        // 楂嬪叧鑺?485 ID
#define ANKLE_MOTOR_ID             ANKLE_CAN_NODE_ID  // 韪濆叧鑺?CAN ID锛屽彧淇濈暀 CAN 鑺傜偣 1

/* 銆愰噸瑕佽皟璇曞紑鍏炽€?: 姝ｅ父姝ユ€? 1: 鍥哄畾瑙掑害娴嬭瘯 */
#define MOTOR_TEST_MODE            0

#define CONTROL_PERIOD_MS          10U
#define PRINT_PERIOD_MS            50U
#define VOFA_ENABLE                1U

/* ================= 127妯″潡铻嶅悎鎺у埗鍙傛暟 =================
 * 127妯″潡閫氳繃 CAN2(PB5/PB6) 閫氫俊锛涗富鎺ф瘡2ms鍙戦€?x010鍚屾甯э紝
 * 127妯″潡鏀跺埌鍚庡洖浼?ID=127(鑲岀數+闄€铻轰华) 鍜?ID=227(鍔犻€熷害)銆?
 */
#define NODE127_SYNC_CAN_ID        0x010U
#define NODE127_SYNC_PERIOD_MS     2U
#define NODE127_DATA_TIMEOUT_MS    250U
#define NODE127_WARMUP_MS           3000U
#define NODE127_EMG_TIMEOUT_MS     180U
#define NODE127_EMG_LPF_ALPHA      0.18f   /* legacy default, kept for compatibility with older notes */
#define NODE127_EMG_ATTACK_ALPHA   0.80f   /* fast envelope attack: human intent should start the gait quickly */
#define NODE127_EMG_RELEASE_ALPHA  0.10f   /* slower release: filters single-sample dropouts without long motor overrun */
#define NODE127_EMG_BASELINE_ALPHA 0.0008f
#define NODE127_EMG_BASELINE_TRACK_RAW 25.0f  /* 闈欐鏃跺厑璁稿熀绾挎參璺熻釜鐨勬渶澶ф紓绉婚噺 */
#define NODE127_EMG_DEADBAND_RAW   6.0f  /* 鑲岀數姝诲尯锛氭寜0614鏁版嵁鏀惧ぇ锛岄伩鍏嶉潤姝㈠熀绾挎紓绉昏瑙﹀彂 */
#define NODE127_EMG_FULL_SCALE_RAW 25.0f /* 鑲岀數褰掍竴鍖栨弧閲忕▼锛屾寜127瀹為檯0~2048杈撳嚭閲嶆柊鏍囧畾 */
#define NODE127_GYRO_DEADBAND_RAW  250.0f
#define NODE127_GYRO_FULL_RAW      3500.0f
#define NODE127_MOTION_HOLD_MS     80U     /* short anti-jitter hold only */
#define NODE127_PHASE_MIN_SPEED    90.0f   /* 姝ｅ父璧拌矾鐩镐綅閫熷害涓嬮檺锛氱害1.3~3.1s/姝ユ€佸懆鏈燂紝鍙敱鑲岀數寮哄害璋冭妭 */
#define NODE127_PHASE_MAX_SPEED    205.0f   /* 姝ｅ父璧拌矾鐩镐綅閫熷害涓婇檺锛氬己鑲岀數鏃舵帴杩戣嚜鐒舵棰戯紝浣嗕粛閬垮厤杩囧揩 */
#define NODE127_GAIT_SCALE         1.0f
#define NODE127_CALIB_MS            300U     /* 涓婄數鍚庨噰闆?27闈欐闆跺亸锛屾湡闂翠笁涓數鏈哄彧淇濇寔 */
#define NODE127_MOVE_ON_GYRO_NORM   0.12f     /* 闄€铻轰华涓嶅啀鍗曠嫭瑙﹀彂锛屽彧鍙備笌杩愬姩寮哄害 */
#define NODE127_MOVE_ON_EMG_NORM    0.170f    /* 鑲岀數涓昏Е鍙戦槇鍊硷細鏄庢樉鍙戝姏鎵嶈繘鍏ユ鎬?*/
#define NODE127_STOP_GYRO_NORM      0.030f    /* 鍋滄鏃朵富瑕佺湅鑲岀數锛岄檧铻轰华鍙仛杈呭姪鏄剧ず */
#define NODE127_STOP_EMG_NORM       0.060f
#define NODE127_STOP_CONFIRM_MS     220U
#define NODE127_FAST_STOP_CONFIRM_MS 120U
#define NODE127_EMG_ON_CONFIRM_MS   0U     /* quick but debounced start when EMG stays above threshold */
#define NODE127_EMG_RESTART_MIN_MS  140U    /* short restart lockout after a real stop */
#define NODE127_INTENT_ATTACK_ALPHA 0.80f
#define NODE127_INTENT_RELEASE_ALPHA 0.28f
#define NODE124_PEAK_HOLD_MS        650U
#define NODE124_PEAK_RISE_NORM      0.075f
#define NODE124_PEAK_MIN_NORM       0.140f
#define NODE124_START_IGNORE_MS     1200U
#define NODE124_EMG_START_RAW      14.0f
#define NODE124_EMG_KEEP_RAW       10.0f

/* =============== 涓夊叧鑺傚崗璋冧笌骞呭害闄愬埗鍙傛暟 ===============
 * 鍙傝€冧汉浣撴鎬佹枃鐚殑鍋氭硶锛氶珛/鑶?韪濅笉鍚勮嚜鐙珛璺戯紝鑰屾槸鍏辩敤涓€涓?gait phase锛?
 * 127鑲岀數鍙喅瀹氭槸鍚﹀厑璁歌蛋璺紝鑲岀數/闄€铻轰华寮哄害鍐冲畾鐩镐綅閫熷害鍜屽箙搴︺€?
 * 涓婄數/棣栨鍚姩鏃舵墍鏈夊叧鑺傜洰鏍囦粠0鐩稿瑙掑紑濮嬶紝閬垮厤楂嬪叧鑺備竴鍚姩灏辫烦鍒版洸绾跨殑23搴︺€?
 */
#define GAIT_AMP_MIN_SCALE          0.35f    /* 鍒氬惎鍔ㄦ椂閲囩敤绾?5%姝ｅ父姝ユ€佸箙搴︼紝閬垮厤绐佺劧璺冲彉 */
#define GAIT_AMP_MAX_SCALE          1.00f    /* 鏈€澶ф寜姝ｅ父浜轰綋姝ユ€佸箙搴﹁緭鍑猴紝涓嶅啀浜轰负缂╁皬 */
#define GAIT_AMP_RAMP_UP_PER_S      2.20f    /* 骞呭害娓愬彉锛氱害1~1.5绉掔敱灏忓箙杩囨浮鍒版甯稿箙搴?*/
#define GAIT_PHASE_SPEED_LPF_ALPHA  0.32f    /* phase speed low-pass */
#define GAIT_AMP_DECAY_PER_S        1.60f    /* decay commanded gait amplitude quickly after human intent disappears */
#define KNEE_JOINT_ANGLE_SCALE      0.92f    /* 鑶濆叧鑺傛甯告鎬佹渶澶х害60掳灞堟洸锛屾ā鏉垮嘲鍊?5掳锛屽彇0.92 */
#define HIP_JOINT_ANGLE_SCALE       0.85f    /* 楂嬪叧鑺傜害30掳灞堟洸鍒?0掳浼稿睍锛屾€诲箙搴︾害40掳 */
#define ANKLE_JOINT_ANGLE_SCALE     0.95f    /* 韪濆叧鑺傛寜鎺ヨ繎姝ｅ父姝ユ€佽窎灞?鑳屽眻骞呭害杈撳嚭 */
#define KNEE_MAX_CMD_SPEED_DEG_S    320.0f   /* 骞呭害鎭㈠鍒版甯稿悗锛岄檺閫熶篃鐩稿簲鎻愰珮锛岄槻姝㈢洰鏍囪窡涓嶄笂鏇茬嚎 */
#define HIP_MAX_CMD_SPEED_DEG_S     240.0f

#define INIT_HOLD_MS               1500U
#define GAIT_PERIOD_MS             1000U

/* 鐗╃悊杞崲甯搁噺 */
#define DEG_TO_RAD                 0.01745329252f
#define RAD_TO_DEG                 57.2957795131f
#define KNEE_REDUCTION_RATIO       6.33f
#define HIP_REDUCTION_RATIO        6.33f
#define ANKLE_WAVE_SCALE           1.0f
#define ANKLE_TURN_PER_DEG         (ANKLE_OUTPUT_REDUCTION_RATIO / 360.0f)

/* 鐩存帴鎸夊浘涓婄殑鍘熷姝ｈ礋鏇茬嚎杩愬姩锛岄浂鐐瑰彇涓婄數褰撳墠浣嶇疆 */
#define KNEE_CMD_MIN_DEG           (-5.0f)
#define KNEE_CMD_MAX_DEG           (66.0f)

#define HIP_CMD_MIN_DEG            (-24.0f)
#define HIP_CMD_MAX_DEG            (24.0f)

/* 韪濆叧鑺傛洸绾挎寜鍥句腑韪濆叧鑺傛洸绾挎墽琛屻€?
 * 杩欓噷鐨勮搴︽槸鐩稿鈥滀笂鐢垫椂鑴氱殑浣嶇疆鈥濈殑杈撳嚭杞磋搴︼紝
 * 浠ｇ爜浼氳嚜鍔ㄥ噺鍘昏捣濮嬬浉浣嶇殑鏇茬嚎鍊硷紝閬垮厤涓婄數鐬棿鍏堟湞涓€涓柟鍚戦《鍒伴檺浣嶃€?
 */
#define ANKLE_CMD_MIN_DEG          (-22.0f)  /* 姝ｅ父姝ユ€佽笣璺栧眻鍙帴杩?0掳锛岀暀鏈烘浣欓噺 */
#define ANKLE_CMD_MAX_DEG          (12.0f)   /* 姝ｅ父姝ユ€佽笣鑳屽眻绾?~10掳锛屼繚鐣欏皯閲忎綑閲?*/
#define ANKLE_GAIT_OFFSET_DEG      (0.0f)
#define ANKLE_MAX_CMD_SPEED_DEG_S  160.0f

/* 浠庡浘涓鎬?0% 寮€濮嬭窇韪濆叧鑺傛洸绾?*/
#define GAIT_START_PHASE_PCT       0.0f

#define KNEE_DIR_SIGN              1.0f
#define HIP_DIR_SIGN               1.0f
/* 韪濆叧鑺傛柟鍚戙€傝嫢瀹為檯鍔ㄤ綔涓庢洸绾挎柟鍚戠浉鍙嶏紝鍙敼杩欓噷涓?-1.0f銆?*/
#define ANKLE_CMD_DIR              (1.0f)
#define ANKLE_FEEDBACK_TIMEOUT_MS  1000U
#define ANKLE_FEEDBACK_STALE_MS    300U
#define ANKLE_CLOSED_LOOP_DELAY_MS 120U
#define ANKLE_START_RAMP_MS        1000U

#define KP_TARGET                  0.52f    /* 姝ｅ父骞呭害涓嬮€傚綋鎻愰珮浣嶇疆鍒氬害锛屼繚璇佽兘璺熶笂姝ユ€佹洸绾?*/
#define KW_TARGET                  0.18f
#define KP_HOLD_BEFORE_EMG         0.08f    /* 鏈娴嬪埌鑲岀數鍓嶅彧杞讳繚鎸侊紝閬垮厤涓婄數閿佸畾閫犳垚鎶栧姩 */
#define KW_HOLD_BEFORE_EMG         0.08f

/* --- 绠楁硶鎺у埗鍙傛暟 --- */
#define PHASE_ADVANCE_PCT          0.0f      /* 涓夊叧鑺傚叡鐢ㄥ悓涓€鐩镐綅锛岄伩鍏嶉珛鑶濇彁鍓嶈€岃笣涓嶅悓姝?*/
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

/* --- 鑶濆叧鑺備笌楂嬪叧鑺傛鎬佹暟鎹?---
 * 璁捐鐩爣锛氭甯告鎬佽繎浼煎箙搴︺€?
 * 鑶濓細鎽嗗姩鏈熷眻鏇插嘲鍊肩害60掳锛涢珛锛氬垵濮嬪眻鏇层€佹敮鎾戞湡浼稿睍銆佹憜鍔ㄦ湡閲嶆柊灞堟洸锛?
 * 浠ｇ爜浼氱粺涓€鍑忓幓 GAIT_START_PHASE_PCT 澶勭殑鏇茬嚎鍊硷紝鍥犳涓変釜鍏宠妭涓嶄細鍦ㄨ倢鐢佃Е鍙戠灛闂磋烦鍒扮粷瀵硅銆?
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

/* --- 鍏ㄥ眬鍙ユ焺瀹氫箟 --- */
UART_HandleTypeDef huart1; // RS485 鐢垫満涓插彛锛圥A9/PA10锛?
UART_HandleTypeDef huart2; // 鍏煎璋冭瘯/VOFA 涓插彛锛圥A2/PA3锛宎ngle_v15鍘熷伐绋嬩娇鐢級
UART_HandleTypeDef huart3; // 璋冭瘯/VOFA 涓插彛锛圥B10/PB11锛岀敤鎴峰浘涓帴鍙ｏ級
SPI_HandleTypeDef hspi1;   // BMI088 SPI1 (PA5/PA6/PA7)
extern CAN_HandleTypeDef hcan1; // 浠?can.c 寮曠敤
extern CAN_HandleTypeDef hcan2; // 127妯″潡 CAN2

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

/* 涓婄數闃舵鐮侊細鐢ㄤ簬鍒ゆ柇绋嬪簭鍗″湪鍝竴姝ャ€?*/
volatile uint32_t g_boot_stage_code = 0U;
volatile uint32_t g_hardfault_count = 0U;

/* 127妯″潡铻嶅悎鎺у埗鐘舵€侊細鍙127涓嶅姩锛宲hase涓嶆帹杩涳紝涓変釜鍏宠妭淇濇寔涓婁竴鐩爣銆?*/
static float g_node127_emg_baseline = 0.0f;
static float g_node127_emg_env = 0.0f;
static float g_node127_emg_norm = 0.0f;
static float g_node127_emg_delta_raw = 0.0f;
static uint8_t g_node127_emg_baseline_valid = 0U;
static uint8_t g_node127_motion_active = 0U;
static uint32_t g_node127_last_motion_tick = 0U;
static float g_node127_motion_level = 0.0f;
static float g_node127_intent_level = 0.0f;
static float g_node127_gyro_level = 0.0f;
static float g_node124_prev_emg_norm = 0.0f;
static uint32_t g_node124_last_peak_tick = 0U;
static uint32_t g_node124_ignore_until_tick = 0U;
static float g_node127_phase_pct = GAIT_START_PHASE_PCT;
static float g_gait_amp_state = 0.0f;          /* 缁熶竴姝ユ€佸箙搴︾郴鏁帮紝鎵€鏈夊叧鑺傚叡鐢?*/
static float g_gait_phase_speed_state = 0.0f;  /* 骞虫粦鍚庣殑鐩镐綅閫熷害锛屾墍鏈夊叧鑺傚叡鐢?*/
static uint8_t g_node127_first_motion_seen = 0U;

typedef enum {
    NODE127_STATE_BOOT = 0,
    NODE127_STATE_CALIB = 1,
    NODE127_STATE_IDLE = 2,
    NODE127_STATE_MOVE = 3,
    NODE127_STATE_HOLD = 4,
    NODE127_STATE_LOST = 5
} Node127ControlState_t;

static volatile uint8_t g_node127_ctrl_state = NODE127_STATE_BOOT;
static uint8_t g_node127_calibrated = 0U;
static uint32_t g_node127_calib_start_tick = 0U;
static uint32_t g_node127_calib_count = 0U;
static int32_t g_node127_gx_bias_sum = 0;
static int32_t g_node127_gy_bias_sum = 0;
static int32_t g_node127_gz_bias_sum = 0;
static float g_node127_gx_bias = 0.0f;
static float g_node127_gy_bias = 0.0f;
static float g_node127_gz_bias = 0.0f;
static uint32_t g_node127_still_start_tick = 0U;
static uint32_t g_node127_emg_on_start_tick = 0U;
static uint32_t g_node127_last_stop_tick = 0U;


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

/* --- 韪濆叧鑺傛鎬佹彃鍊?--- */
static float ankle_wave_raw_from_percent(float gait_percent) {
    static const float gait_key[] = {0.0f, 6.0f, 10.0f, 16.0f, 22.0f, 28.0f, 35.0f, 45.0f, 52.0f, 56.0f, 62.0f, 68.0f, 75.0f, 82.0f, 90.0f, 96.0f, 100.0f};
    /*
     * 鎺ヨ繎姝ｅ父姝ユ€佺殑韪濆叧鑺傛洸绾匡細
     * 鏃╂湡鎵块噸杞诲害璺栧眻锛屾敮鎾戜腑鏈熼€愭笎鑳屽眻锛岃宫鍦版湡鏄庢樉璺栧眻锛屾憜鍔ㄦ湡鍥炲埌鎺ヨ繎涓珛浣嶃€?
     * 鍚庣画浠嶄細鍑忓幓0%鐩镐綅鍊硷紝鍥犳涓婄數/棣栨瑙﹀彂涓嶄細绐佺劧璺宠銆?
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

/* --- 鍑芥暟澹版槑 --- */
void SystemClock_Config(void);
void MX_GPIO_Init(void);
void MX_USART1_UART_Init(void);
void MX_USART2_UART_Init(void);
void MX_USART3_UART_Init(void);
void MX_SPI1_Init(void);
static void Vofa_SendFrame(const BMI088_t *imu, const MT6701_t *enc);
static void Diag_SendAlive(uint32_t now, uint32_t stage_code);
static void Debug_WriteBufAll(const uint8_t *buf, uint16_t len, uint32_t timeout_ms);
static void Debug_Uart3Printf(const char *fmt, ...);
static void Debug_Uart3WriteRaw(const char *s);
static void Node127_SendSyncRequest(void);

/* ================= 缂栬瘧閿欒淇锛氳ˉ鍏ㄦ鎬佹彃鍊笺€侀珛鑶濆叧鑺傛帶鍒躲€丅MI088銆丮T6701鍑芥暟 =================
 * 涔嬪墠 main.c 鍙繚鐣欎簡杩欎簺鍑芥暟鐨勫０鏄庯紝浣嗘病鏈夊疄鐜帮紝Keil 浼氭姤锛?
 *   function "interp_array_periodic_hermite" was referenced but not defined
 *   function "BMI088_Init/Update" was referenced but not defined
 *   function "MT6701_Init/Update" was referenced but not defined
 * 鍚屾椂 Joint_Init / Joint_HandleResponse / Joint_PrepareRunRelative / Joint_PrepareHoldLast
 * 娌℃湁鎻愬墠澹版槑鎴栧疄鐜帮紝瀵艰嚧 #223-D implicit declaration銆?
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

    /* BMI088鍔犻€熷害璁′笂鐢甸粯璁ゅ湪I2C妯″紡锛岄渶瑕佷竴娆ummy read鍒囧埌SPI銆?*/
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

static void Joint_PrepareRunRelative(JointController_t *j, float phase_pct)
{
    float curve_deg;
    float zero_deg;
    float target_deg;
    float max_step_deg;

    if ((j == NULL) || (j->curve == NULL) || (j->point_count == 0U)) return;

    /*
     * 鍏抽敭淇锛氫汉浣撴鎬佹ā鏉挎槸鈥滅粷瀵规洸绾库€濓紝浣嗙數鏈轰笂鐢甸浂鐐规槸褰撳墠浣嶇疆銆?
     * 鎵€浠ラ珛/鑶濆懡浠ゅ繀椤诲噺鍘昏捣濮嬬浉浣嶅鐨勬洸绾垮€笺€?
     * 鍚﹀垯楂嬪叧鑺傚湪0%鐩镐綅浼氫竴鍚姩灏辫烦鍒扮害23掳锛岄€犳垚骞呭害澶с€佷笁鍏宠妭涓嶅崗璋冦€?
     */
    get_curve_target(j->curve, j->point_count, phase_pct + j->phase_adv_state, &curve_deg);
    get_curve_target(j->curve, j->point_count, GAIT_START_PHASE_PCT + j->phase_adv_state, &zero_deg);

    target_deg = (curve_deg - zero_deg) * Joint_GetAngleScale(j) * g_gait_amp_state;
    target_deg = Joint_ClampCurveDeg(j, target_deg);

    /* 鐩爣瑙掗檺閫燂細闃叉鑲岀數绐佺劧鍙樺ぇ鏃剁數鏈轰竴姝ヨ烦寰堝ぇ銆?*/
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
    j->target_curve_deg = curve_deg - zero_deg;
    j->target_ref_deg = target_deg;
    j->cmd.Pos = j->center_pos + j->dir_sign * (j->cmd_pos_state * DEG_TO_RAD) * j->reduction_ratio;
}

static void Joint_PrepareHoldLast(JointController_t *j)
{
    if (j == NULL) return;

    j->cmd.id = j->motor_id;
    if (j->is_calibrated == 0U) {
        j->cmd.mode = 0U;      /* 杩樻病鏀跺埌鍥炲寘鏃跺厛涓嶅己琛岄攣浣嶇疆 */
        return;
    }

    if (g_node127_first_motion_seen == 0U) {
        /*
         * 涓婄數鍚庛€佽倢鐢垫病鏈夋槑鏄惧彉鍖栦箣鍓嶏紝涓嶇粰瀹囨爲鐢垫満涓嬩綅缃棴鐜懡浠ゃ€?
         * 杩欐牱涓嶄細鍥犱负闆剁偣/鍒氬害/鏇茬嚎鐩爣閫犳垚涓婄數杞诲井鍔ㄤ綔銆?
         * 鐪熸妫€娴嬪埌鑲岀數涓诲姩鎰忓浘鍚庯紝Joint_PrepareRunRelative() 鎵嶈繘鍏ヤ綅缃帶鍒躲€?
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
    /* 宸茬粡璺戣繃鏃朵笉閲嶆柊璁＄畻鐩爣锛屼繚鎸佷笂涓€甯?cmd.Pos銆?*/
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

    if (huart3.Instance == USART3) {
        (void)HAL_UART_Transmit(&huart3, (uint8_t *)buf, len, timeout_ms);
    }
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

    /* 纭欢鍥哄畾锛欳AN2(PB5/PB6) 鎺ユ敹/瑙﹀彂 127 妯″潡锛孋AN1 鍙帶鍒惰笣鍏宠妭鐢垫満銆?
     * 鍥犳 0x010 鍚屾甯у彧鍏佽浠?CAN2 鍙戦€侊紝绂佹鍚?CAN1 鍙戦€侊紝閬垮厤骞叉壈韪濆叧鑺傜數鏈烘€荤嚎銆?*/
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

/* 榛樿鐢╕杞翠綔涓轰富杩愬姩杞达紱濡傛灉鏂瑰悜/鐏垫晱搴︿笉瀵癸紝鍚庣画鍙渶瑕佹敼杩欓噷銆?*/
static int16_t Node127_GetPrimaryGyroRaw(void)
{
    return (int16_t)((float)g_node127.gy - g_node127_gy_bias);
}

static int16_t Node127_GetMaxAbsGyroRaw(void)
{
    float x = (float)g_node127.gx - g_node127_gx_bias;
    float y = (float)g_node127.gy - g_node127_gy_bias;
    float z = (float)g_node127.gz - g_node127_gz_bias;
    float ax = fabsf(x);
    float ay = fabsf(y);
    float az = fabsf(z);
    float m = ax;
    if (ay > m) m = ay;
    if (az > m) m = az;
    if (m > 32767.0f) m = 32767.0f;
    return (int16_t)m;
}

static void Node127_ResetCalibration(uint32_t now)
{
    g_node127_calibrated = 0U;
    g_node127_calib_start_tick = now;
    g_node127_calib_count = 0U;
    g_node127_gx_bias_sum = 0;
    g_node127_gy_bias_sum = 0;
    g_node127_gz_bias_sum = 0;
    g_node127_gx_bias = 0.0f;
    g_node127_gy_bias = 0.0f;
    g_node127_gz_bias = 0.0f;
    g_node127_emg_baseline_valid = 0U;
    g_node127_emg_baseline = 0.0f;
    g_node127_emg_env = 0.0f;
    g_node127_emg_norm = 0.0f;
    g_node127_emg_delta_raw = 0.0f;
    g_node127_motion_active = 0U;
    g_node127_motion_level = 0.0f;
    g_node127_intent_level = 0.0f;
    g_node127_gyro_level = 0.0f;
    g_node124_prev_emg_norm = 0.0f;
    g_node124_last_peak_tick = now - (NODE124_PEAK_HOLD_MS + 1U);
    g_node124_ignore_until_tick = now + NODE124_START_IGNORE_MS;
    g_node127_first_motion_seen = 0U;
    g_gait_amp_state = 0.0f;
    g_gait_phase_speed_state = 0.0f;
    g_node127_phase_pct = GAIT_START_PHASE_PCT;
    g_node127_still_start_tick = 0U;
    g_node127_emg_on_start_tick = 0U;
    g_node127_last_stop_tick = now;
    g_node127_ctrl_state = NODE127_STATE_CALIB;
}

/* 127闈欐鏍囧畾锛氬彧鍦ㄦ敹鍒癐D=127鏃剁疮璁★紝娌℃敹鍒板氨涓€鐩翠繚鎸侊紝涓嶈鐢垫満涔卞姩銆?*/
static uint8_t Node127_RunCalibration(uint32_t now)
{
    if (g_node127_calib_start_tick == 0U) {
        Node127_ResetCalibration(now);
    }

    if (!Node127_DataFresh(now)) {
        g_node127_ctrl_state = NODE127_STATE_LOST;
        return 0U;
    }

    g_node127_ctrl_state = NODE127_STATE_CALIB;
    g_node127_gx_bias_sum += (int32_t)g_node127.gx;
    g_node127_gy_bias_sum += (int32_t)g_node127.gy;
    g_node127_gz_bias_sum += (int32_t)g_node127.gz;
    g_node127_calib_count++;

    if (g_node127_emg_baseline_valid == 0U) {
        g_node127_emg_baseline = (float)g_node127.emg;
        g_node127_emg_baseline_valid = 1U;
    } else {
        g_node127_emg_baseline += 0.02f * ((float)g_node127.emg - g_node127_emg_baseline);
    }

    if (((now - g_node127_calib_start_tick) >= NODE127_CALIB_MS) && (g_node127_calib_count >= 20U)) {
        g_node127_gx_bias = (float)g_node127_gx_bias_sum / (float)g_node127_calib_count;
        g_node127_gy_bias = (float)g_node127_gy_bias_sum / (float)g_node127_calib_count;
        g_node127_gz_bias = (float)g_node127_gz_bias_sum / (float)g_node127_calib_count;
        g_node127_calibrated = 1U;
        g_node127_ctrl_state = NODE127_STATE_IDLE;
        g_node127_motion_active = 0U;
        g_node127_motion_level = 0.0f;
        g_node127_intent_level = 0.0f;
        g_node124_prev_emg_norm = 0.0f;
        g_node124_last_peak_tick = now - (NODE124_PEAK_HOLD_MS + 1U);
        g_node124_ignore_until_tick = now + NODE124_START_IGNORE_MS;
        g_node127_first_motion_seen = 0U;
        g_node127_emg_on_start_tick = 0U;
        g_node127_last_motion_tick = now;
        g_node127_still_start_tick = now;
        return 1U;
    }

    return 0U;
}

static void Node127_EmgUpdate(uint32_t now)
{
    float raw;
    float diff;
    float active_raw;
    float alpha;

    if ((g_node127.gyro_valid == 0U) || ((now - g_node127.gyro_tick) > NODE127_EMG_TIMEOUT_MS)) {
        g_node127_emg_norm = 0.0f;
        g_node127_emg_env *= 0.82f;
        return;
    }

    raw = (float)g_node127.emg;
    if (g_node127_emg_baseline_valid == 0U) {
        g_node127_emg_baseline = raw;
        g_node127_emg_env = 0.0f;
        g_node127_emg_norm = 0.0f;
        g_node127_emg_baseline_valid = 1U;
        return;
    }

    diff = fabsf(raw - g_node127_emg_baseline);

    if ((g_node127_motion_active == 0U) && (diff < NODE127_EMG_BASELINE_TRACK_RAW)) {
        g_node127_emg_baseline += NODE127_EMG_BASELINE_ALPHA * (raw - g_node127_emg_baseline);
        diff = fabsf(raw - g_node127_emg_baseline);
    }

    g_node127_emg_delta_raw = diff;
    alpha = (diff > g_node127_emg_env) ? NODE127_EMG_ATTACK_ALPHA : NODE127_EMG_RELEASE_ALPHA;
    g_node127_emg_env += alpha * (diff - g_node127_emg_env);
    active_raw = g_node127_emg_env - NODE127_EMG_DEADBAND_RAW;
    if (active_raw < 0.0f) active_raw = 0.0f;
    g_node127_emg_norm = clampf_local(active_raw / NODE127_EMG_FULL_SCALE_RAW, 0.0f, 1.0f);
}
static float Node127_EmgNorm(uint32_t now)
{
    if ((g_node127.gyro_valid == 0U) || ((now - g_node127.gyro_tick) > NODE127_EMG_TIMEOUT_MS)) {
        return 0.0f;
    }
    return g_node127_emg_norm;
}

/* 杩斿洖1琛ㄧず127妫€娴嬪埌浜鸿吙杩愬姩/鑲岀數涓诲姩鎰忓浘锛涜繑鍥?琛ㄧず浜轰笉鍔紝搴斾繚鎸併€?*/
static uint8_t Node127_UpdateMotionControl(uint32_t now)
{
    float gyro_abs;
    float gyro_active;
    float emg_active;
    float emg_raw;
    float emg_rise;
    float level;
    float intent_target;
    uint8_t peak_detected;
    uint8_t peak_window_active;

    if (!Node127_DataFresh(now)) {
        g_node127_ctrl_state = NODE127_STATE_LOST;
        g_node127_motion_active = 0U;
        g_node127_motion_level = 0.0f;
        g_node127_intent_level = 0.0f;
        g_node127_gyro_level = 0.0f;
        g_node127_emg_on_start_tick = 0U;
        return 0U;
    }

    if (g_node127_calibrated == 0U) {
        (void)Node127_RunCalibration(now);
        return 0U;
    }

    Node127_EmgUpdate(now);

    gyro_abs = (float)Node127_GetMaxAbsGyroRaw();
    gyro_active = (gyro_abs - NODE127_GYRO_DEADBAND_RAW) / NODE127_GYRO_FULL_RAW;
    gyro_active = clampf_local(gyro_active, 0.0f, 1.0f);
    emg_active = Node127_EmgNorm(now);
    emg_raw = (float)g_node127.emg;
    g_node127_gyro_level = gyro_active;
    if ((int32_t)(now - g_node124_ignore_until_tick) < 0) {
        g_node124_prev_emg_norm = emg_active;
        g_node124_last_peak_tick = now - (NODE124_PEAK_HOLD_MS + 1U);
        g_node127_motion_active = 0U;
        g_node127_motion_level = 0.0f;
        g_node127_intent_level = 0.0f;
        g_node127_ctrl_state = NODE127_STATE_IDLE;
        return 0U;
    }

    emg_rise = emg_active - g_node124_prev_emg_norm;
    peak_detected = 0U;
    if ((emg_raw >= NODE124_EMG_START_RAW) && (emg_active >= NODE127_MOVE_ON_EMG_NORM)) {
        peak_detected = 1U;
    }
    else if ((emg_raw >= NODE124_EMG_START_RAW) && (emg_active >= NODE124_PEAK_MIN_NORM) && (emg_rise >= NODE124_PEAK_RISE_NORM)) {
        peak_detected = 1U;
    }

    if (peak_detected != 0U) {
        g_node124_last_peak_tick = now;
        g_node127_first_motion_seen = 1U;
        g_node127_still_start_tick = 0U;
    }

    peak_window_active = ((now - g_node124_last_peak_tick) <= NODE124_PEAK_HOLD_MS) ? 1U : 0U;

    intent_target = peak_window_active ? emg_active : 0.0f;
    if (peak_detected != 0U && intent_target < NODE127_MOVE_ON_EMG_NORM) {
        intent_target = NODE127_MOVE_ON_EMG_NORM;
    }
    if (g_node127_intent_level < intent_target) {
        g_node127_intent_level += NODE127_INTENT_ATTACK_ALPHA * (intent_target - g_node127_intent_level);
    } else {
        g_node127_intent_level += NODE127_INTENT_RELEASE_ALPHA * (intent_target - g_node127_intent_level);
    }

    g_node124_prev_emg_norm = emg_active;

    if (peak_window_active == 0U) {
        g_node127_motion_active = 0U;
        g_node127_motion_level = 0.0f;
        g_node127_emg_on_start_tick = 0U;
        g_node127_last_stop_tick = now;
        g_node127_ctrl_state = g_node127_first_motion_seen ? NODE127_STATE_HOLD : NODE127_STATE_IDLE;
        return 0U;
    }

    level = 0.90f * g_node127_intent_level + 0.10f * gyro_active;
    if (level < 0.35f) level = 0.35f;
    g_node127_motion_level = clampf_local(level, 0.0f, 1.0f);
    g_node127_motion_active = 1U;
    g_node127_last_motion_tick = now;
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
     * 涓婄數鍚庢病鏈夋娴嬪埌鏄庢樉鑲岀數鍓嶏紝韪濆叧鑺備笉瑕佷富鍔ㄨ繘鍏ヤ綅缃棴鐜紝
     * 閬垮厤涓婄數杞诲井鎵句綅缃?鎷夊洖鍒濆鐐广€傝繖閲屼粎璁板綍褰撳墠浣嶇疆浣滀负鍚庣画闆剁偣銆?
     */
    if (motor_fb_valid[node_id] != 0U) {
        motor[node_id].init_pos = motor[node_id].pos;
        g_ankle_cmd_turn_state = motor[node_id].pos;
        g_ankle_cmd_turn_state_valid = 1U;
        g_ankle_cmd_turn = motor[node_id].pos;
    }
    g_ankle_cmd_deg = 0.0f;
    g_ankle_gait_enable = 0U;

    /* 濡傛灉鍒濆鍖栨椂宸茬粡杩涘叆闂幆锛屽懆鏈熸€ч€€鍥瀒dle锛岀洿鍒扮涓€娆¤倢鐢佃Е鍙戙€?*/
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

/* ==================================================================
 * MAIN 鎵ц閫昏緫
 * ================================================================== */
void App_Main(void) {
    /* 閲嶈淇锛氬師宸ョ▼ startup 鍙湁 0x400 鏍堬紝灞€閮ㄥぇ缁撴瀯浣撳鏄撳鑷?HardFault锛?
     * 鎵€浠ヨ繖浜涜繍琛屽璞℃斁鍒伴潤鎬佸尯锛屼笉鍐嶅崰鐢ㄦ爤銆?*/
    static JointController_t knee, hip;
    static BMI088_t imu;
    static MT6701_t enc;
    uint32_t last_tick, last_print, run_start_tick;
    uint32_t last_node127_sync_tick;
    uint32_t last_diag_tick;
    uint8_t ankle_can_ready;

    g_boot_stage_code = 1U;
    HAL_Init();
    g_boot_stage_code = 2U;
    SystemClock_Config();
    g_boot_stage_code = 3U; 
    memset(&imu, 0, sizeof(imu));
    memset(&enc, 0, sizeof(enc));
    MX_GPIO_Init();
    g_boot_stage_code = 4U;
    MX_USART3_UART_Init();
    MX_USART2_UART_Init();
    g_boot_stage_code = 5U;
    Debug_Uart3WriteRaw("BOOT0: debug alive on USART3 PB10 and USART2 PA2, 115200, 8N1\r\n");
    Debug_Uart3Printf("BOOT1: SYSCLK=%lu PCLK1=%lu PCLK2=%lu\r\n", (unsigned long)SystemCoreClock, (unsigned long)HAL_RCC_GetPCLK1Freq(), (unsigned long)HAL_RCC_GetPCLK2Freq());
    MX_USART1_UART_Init();
    g_boot_stage_code = 6U;
    MX_SPI1_Init();
    g_boot_stage_code = 7U;
    HAL_Delay(50);

    /* 鍏抽敭淇锛氫笉瑕佹妸绋嬪簭鍗″湪127棰勭儹閲屻€?
     * 鍏堝垵濮嬪寲 CAN1+CAN2锛屽啀鍒嗗埆鍚姩锛涗富寰幆鍙€氳繃CAN2鎸佺画鍙戦€?x010鍚屾甯с€?
     * 杩欐牱鍗充娇127娌℃湁鎺ュソ锛孶SART3涔熶細鎸佺画杈撳嚭璇婃柇鏁版嵁銆?*/
    MX_CAN1_Init();
    g_boot_stage_code = 8U;
    MX_CAN2_Init();
    g_boot_stage_code = 9U;
    USER_CAN2_Start();   /* 127妯″潡锛欳AN2 PB5/PB6 */
    g_boot_stage_code = 10U;
    USER_CAN1_Start();   /* 韪濆叧鑺傝タ鏍肩帥鐢垫満锛欳AN1 PB8/PB9 */
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

    /* 韪濆叧鑺傚彧鏈変竴涓?CAN 鐢垫満锛氫笉瑕佸啀瀵?0~3 鍙疯妭鐐规壒閲忓垵濮嬪寲锛?
     * 涔熶笉瑕佸涓嶅瓨鍦ㄧ殑 CAN ID 鍙戦€侀棴鐜懡浠ゃ€?
     */
    ankle_can_ready = Ankle_InitCANPositionMode(ANKLE_MOTOR_ID);
    g_boot_stage_code = 14U;
    Debug_Uart3Printf("BOOT: ankle init result=%u, ready=%u\r\n", (unsigned int)g_ankle_init_result, (unsigned int)ankle_can_ready);

    // 缁熶竴鍒濆鍖栫粨鏋勪綋
    Joint_Init(&knee, "knee", KNEE_MOTOR_ID, KNEE_DIR_SIGN, KNEE_REDUCTION_RATIO, kKneeCurve, KNEE_POINT_COUNT);
    Joint_Init(&hip, "hip", HIP_MOTOR_ID, HIP_DIR_SIGN, HIP_REDUCTION_RATIO, kHipCurve, HIP_POINT_COUNT);

    /* 127鎺у埗閫昏緫浠庨潤姝㈡爣瀹氬紑濮嬶細鏍囧畾瀹屾垚鍓嶃€?27鎺夌嚎鏃讹紝涓変釜鐢垫満閮藉彧淇濇寔銆?*/
    Node127_ResetCalibration(HAL_GetTick());

    run_start_tick = HAL_GetTick();
    last_tick = run_start_tick;
    last_print = 0U;
    last_node127_sync_tick = run_start_tick;
    last_diag_tick = 0U;

    while (1) {
        uint32_t now = HAL_GetTick();
        uint32_t elapsed = now - run_start_tick;

        /* USART3鐢熷懡淇″彿锛氬嵆浣?27/CAN/鐢垫満閮芥病鏈夋帴濂斤紝涔熷繀椤绘寔缁緭鍑恒€?*/
        if ((now - last_diag_tick) >= 200U) {
            last_diag_tick = now;
            Diag_SendAlive(now, 20U);
        }

        /* CAN2鎺ユ敹閲囩敤涓柇+杞鍙屼繚闄╋紝闃叉NVIC/杩囨护鍣ㄥ紓甯稿鑷?27鏁版嵁杩涗笉鏉ャ€?*/
        USER_CAN_PollRx();

        if ((now - last_node127_sync_tick) >= NODE127_SYNC_PERIOD_MS) {
            last_node127_sync_tick = now;
            Node127_SendSyncRequest();
        }
        if (now - last_tick >= CONTROL_PERIOD_MS) {
            last_tick = now;

            if (elapsed < INIT_HOLD_MS) {
                /* 涓婄數鍓?.5绉掑彧鍋氶浂鐐硅褰?褰撳墠浣嶇疆淇濇寔锛岀粷涓嶈窇鏇茬嚎銆?*/
                knee.cmd.mode = 0U;
                Joint_HandleResponse(&knee, SERVO_Send_recv(&knee.cmd, &knee.data));

                hip.cmd.mode = 0U;
                Joint_HandleResponse(&hip, SERVO_Send_recv(&hip.cmd, &hip.data));

                if (ankle_can_ready != 0U) {
                    /* 涓婄數闈欐闃舵锛氬彧璁板綍褰撳墠浣嶇疆锛屼笉璁╄笣鍏宠妭涓诲姩闂幆鍔ㄤ綔銆?*/
                    Ankle_IdleBeforeFirstEmg(ANKLE_MOTOR_ID);
                }
            } else {
                uint8_t node127_motion = Node127_UpdateMotionControl(now);

                if (node127_motion != 0U) {
                    float dt_s = (float)CONTROL_PERIOD_MS * 0.001f;
                    float amp_target;
                    float amp_step;
                    float phase_speed_target;
                    float motion_drive;

                    /*
                     * 涓夊叧鑺傚崗璋冩牳蹇冿細
                     * 1) 楂嬨€佽啙銆佽笣鍏辩敤鍚屼竴涓?g_node127_phase_pct锛屼繚璇佺浉浣嶄竴鑷达紱
                     * 2) 鍏辩敤 g_gait_amp_state 鍋氬箙搴︽笎鍙橈紝鏈€缁堝厑璁歌揪鍒版甯镐汉璧拌矾骞呭害锛?
                     * 3) 鏇茬嚎鏈韩鍖呭惈鍚勫叧鑺傚湪姝ユ€佷腑鐨勫厛鍚庡叧绯伙紝涓嶅啀璁╂瘡涓數鏈哄悇鑷嫭绔嬭窇銆?
                     */
                    motion_drive = clampf_local((0.75f * g_node127_intent_level) + (0.25f * g_node127_motion_level), 0.0f, 1.0f);
                    amp_target = GAIT_AMP_MIN_SCALE +
                        (GAIT_AMP_MAX_SCALE - GAIT_AMP_MIN_SCALE) * motion_drive;
                    amp_target = clampf_local(amp_target, GAIT_AMP_MIN_SCALE, GAIT_AMP_MAX_SCALE);
                    amp_step = GAIT_AMP_RAMP_UP_PER_S * dt_s;
                    if (g_gait_amp_state < amp_target) {
                        g_gait_amp_state += amp_step;
                        if (g_gait_amp_state > amp_target) g_gait_amp_state = amp_target;
                    } else {
                        /* 浜轰粛鍦ㄥ彂鍔涗絾寮哄害鍙樺皬锛屽箙搴︽參鎱㈠彉灏忥紝閬垮厤绐侀檷銆?*/
                        g_gait_amp_state -= (0.35f * amp_step);
                        if (g_gait_amp_state < amp_target) g_gait_amp_state = amp_target;
                    }

                    phase_speed_target = NODE127_PHASE_MIN_SPEED +
                        (NODE127_PHASE_MAX_SPEED - NODE127_PHASE_MIN_SPEED) * motion_drive;
                    if (g_gait_phase_speed_state <= 1.0f) {
                        g_gait_phase_speed_state = phase_speed_target;
                    } else {
                        g_gait_phase_speed_state += GAIT_PHASE_SPEED_LPF_ALPHA *
                            (phase_speed_target - g_gait_phase_speed_state);
                    }
                    g_node127_phase_pct = wrap_phase_pct(g_node127_phase_pct + g_gait_phase_speed_state * dt_s);

                    Joint_PrepareRunRelative(&knee, g_node127_phase_pct);
                    Joint_HandleResponse(&knee, SERVO_Send_recv(&knee.cmd, &knee.data));

                    Joint_PrepareRunRelative(&hip, g_node127_phase_pct);
                    Joint_HandleResponse(&hip, SERVO_Send_recv(&hip.cmd, &hip.data));
                } else {
                    {
                        float dt_s = (float)CONTROL_PERIOD_MS * 0.001f;
                        float amp_decay_step = GAIT_AMP_DECAY_PER_S * dt_s;
                        if (g_gait_amp_state > amp_decay_step) g_gait_amp_state -= amp_decay_step;
                        else g_gait_amp_state = 0.0f;
                        g_gait_phase_speed_state *= 0.75f;
                        if (g_gait_phase_speed_state < 1.0f) g_gait_phase_speed_state = 0.0f;
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
                        /* 鑲岀數娌℃湁鏄庢樉鍙樺寲鍓嶏紝韪濆叧鑺備繚鎸乮dle锛屼笉涓诲姩鎵句綅缃€?*/
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
                        /* 浜轰笉鍔ㄦ椂韪濆叧鑺備繚鎸佷笂涓€鐩爣锛屼笉鍥炲垵濮嬩綅銆佷笉缁х画璺戙€?*/
                        Ankle_HoldLastTarget(ANKLE_MOTOR_ID);
                    }
                }
            }

            if (now - last_print >= PRINT_PERIOD_MS) {
                last_print = now;

                (void)BMI088_Update(&imu);
                (void)MT6701_Update(&enc);
                Vofa_SendFrame(&imu, &enc);
            }
        }
    }
}

static void Vofa_SendFrame(const BMI088_t *imu, const MT6701_t *enc) {
    static char tx[768];
    uint32_t now = HAL_GetTick();
    long imu_gx_milli = 0, imu_gy_milli = 0, imu_gz_milli = 0;
    long enc_deg_milli = 0;
    long ankle_pos_milli = (long)(motor[ANKLE_MOTOR_ID].pos * 1000.0f);
    long ankle_init_milli = (long)(motor[ANKLE_MOTOR_ID].init_pos * 1000.0f);
    long ankle_cmd_deg_milli = (long)(g_ankle_cmd_deg * 1000.0f);
    long ankle_cmd_turn_milli = (long)(g_ankle_cmd_turn * 1000.0f);
    long emg_norm_milli = (long)(g_node127_emg_norm * 1000.0f);
    long gyro_level_milli = (long)(g_node127_gyro_level * 1000.0f);
    long phase_milli = (long)(g_node127_phase_pct * 1000.0f);
    long gait_amp_milli = (long)(g_gait_amp_state * 1000.0f);
    long gait_speed_milli = (long)(g_gait_phase_speed_state * 1000.0f);
    long emg_delta_raw = (long)g_node127_emg_delta_raw;
    long emg_baseline_raw = (long)g_node127_emg_baseline;
    int len;

    if (imu != NULL) {
        imu_gx_milli = (long)(imu->gyro_dps[0] * 1000.0f);
        imu_gy_milli = (long)(imu->gyro_dps[1] * 1000.0f);
        imu_gz_milli = (long)(imu->gyro_dps[2] * 1000.0f);
    }
    if (enc != NULL) {
        enc_deg_milli = (long)(enc->angle_deg * 1000.0f);
    }

    /* 9002 涓虹函鏁存暟 ASCII 璇婃柇甯э紝涓嶄娇鐢?printf 娴偣鏍煎紡銆?
     * 涔嬪墠 %.3f/%.4f 鍦?Keil 鏈紑鍚?float printf 鎴栨爤杈冨皬鏃跺彲鑳藉鑷村紓甯革紝
     * 浠庤€屽嚭鐜扳€淰OFA 瀹屽叏娌℃湁鏁版嵁鈥濄€?
     */
    len = snprintf(tx, sizeof(tx),
                   "9002,%lu,%ld,%ld,%ld,%u,%ld,%u,%u,%u,%u,%ld,%ld,%ld,%ld,%u,%u,%lu,%u,%u,%u,%lu,%lu,%d,%d,%d,%d,%d,%d,%u,%ld,%ld,%ld,%u,%lu,%lu,%lu,%lu,%lu,%u,%lu,%lu,%lu,%lu,%lu,%lu,%lu,%lu,%lu,%lu,%lu,%u,%u,%ld,%ld,%lu,%lu,%lu,%d,%ld,%ld\r\n",
                   (unsigned long)now,
                   imu_gx_milli,
                   imu_gy_milli,
                   imu_gz_milli,
                   (imu != NULL) ? (unsigned int)imu->init_ok : 0U,
                   enc_deg_milli,
                   (enc != NULL) ? (unsigned int)enc->angle_raw : 0U,
                   (enc != NULL) ? (unsigned int)enc->read_ok : 0U,
                   (unsigned int)g_ankle_init_step,
                   (unsigned int)g_ankle_init_result,
                   ankle_pos_milli,
                   ankle_init_milli,
                   ankle_cmd_deg_milli,
                   ankle_cmd_turn_milli,
                   (unsigned int)g_ankle_gait_enable,
                   (unsigned int)motor_axis_state[ANKLE_MOTOR_ID],
                   (unsigned long)motor_axis_flags[ANKLE_MOTOR_ID],
                   (unsigned int)motor_axis_error[ANKLE_MOTOR_ID],
                   (unsigned int)motor_fb_valid[ANKLE_MOTOR_ID],
                   (unsigned int)motor_hb_valid[ANKLE_MOTOR_ID],
                   (unsigned long)g_can1_error_code,
                   (unsigned long)g_can1_busoff_count,
                   (int)g_node127.emg,
                   (int)g_node127.gx,
                   (int)g_node127.gy,
                   (int)g_node127.gz,
                   (int)g_node127.ax,
                   (int)g_node127.ay,
                   (unsigned int)g_node127.gyro_valid,
                   emg_norm_milli,
                   gyro_level_milli,
                   phase_milli,
                   (unsigned int)g_node127_motion_active,
                   (unsigned long)g_node127_sync_count,
                   (unsigned long)g_can2_rx_count,
                   (unsigned long)g_can2_127_rx_count,
                   (unsigned long)g_can2_227_rx_count,
                   (unsigned long)g_can2_last_id,
                   (unsigned int)g_node127_sync_last_status,
                   (unsigned long)g_can2_busoff_count,
                   (unsigned long)g_can2_tx_ok_count,
                   (unsigned long)g_can2_tx_fail_count,
                   (unsigned long)g_can2_tx_busy_count,
                   (unsigned long)g_can2_tx_abort_count,
                   (unsigned long)g_can2_tx_mailbox_free,
                   (unsigned long)g_can2_esr_snapshot,
                   (unsigned long)g_can2_msr_snapshot,
                   (unsigned long)g_can2_state_snapshot,
                   (unsigned long)g_node127_warmup_done,
                   (unsigned long)g_boot_stage_code,
                   (unsigned int)g_node127_ctrl_state,
                   (unsigned int)g_node127_calibrated,
                   gait_amp_milli,
                   gait_speed_milli,
                   (unsigned long)g_node127_rx_bus,
                   (unsigned long)g_can1_127_rx_count,
                   (unsigned long)g_can1_227_rx_count,
                   (int)Node127_GetPrimaryGyroRaw(),
                   emg_delta_raw,
                   emg_baseline_raw);
    if (len > 0) {
        if (len > (int)sizeof(tx)) len = (int)sizeof(tx);
        Debug_WriteBufAll((uint8_t *)tx, (uint16_t)len, 30U);
    }
}

/* --- 搴曞眰纭欢閰嶇疆 (鍩轰簬 160MHz) --- */
void Error_Handler(void) {
    /* 涓嶅啀鏃犲０姝绘満锛氬鏋淯SART3宸茬粡鍒濆鍖栵紝鎸佺画鍙戦敊璇爣璁般€?*/
    while (1) {
        if (huart3.Instance == USART3) {
            const char *e = "9999,ERROR_HANDLER\r\n";
            Debug_WriteBufAll((uint8_t *)e, (uint16_t)strlen(e), 50U);
        }
        HAL_Delay(200);
    }
}
