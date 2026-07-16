#include "getdata.h"
#include "ankle_can_config.h"

#define MOTOR_CAN_MAX_NODE_ID 6U
#define NODE124_EMG_CAN_ID_PRIMARY 125U
#define NODE124_EMG_CAN_ID_ALT     124U

volatile uint8_t motor_fb_valid[MOTOR_CAN_MAX_NODE_ID] = {0};
volatile uint32_t motor_fb_tick[MOTOR_CAN_MAX_NODE_ID] = {0};
volatile uint32_t motor_axis_error[MOTOR_CAN_MAX_NODE_ID] = {0};
volatile uint8_t motor_axis_state[MOTOR_CAN_MAX_NODE_ID] = {0};
volatile uint8_t motor_axis_flags[MOTOR_CAN_MAX_NODE_ID] = {0};
volatile uint8_t motor_hb_valid[MOTOR_CAN_MAX_NODE_ID] = {0};
volatile uint32_t motor_hb_tick[MOTOR_CAN_MAX_NODE_ID] = {0};
volatile uint8_t g_ankle_init_step = 0U;
volatile uint8_t g_ankle_init_result = 0U;
volatile uint8_t g_ankle_last_tx_status = 0U;

Node127Data_t g_node127 = {0};
volatile uint32_t g_can2_rx_count = 0U;
volatile uint32_t g_can2_127_rx_count = 0U;
volatile uint32_t g_can2_227_rx_count = 0U;
volatile uint32_t g_can2_last_id = 0U;
volatile uint32_t g_can2_error_code = 0U;
volatile uint32_t g_can2_busoff_count = 0U;
volatile uint32_t g_node127_sync_count = 0U;
volatile uint8_t  g_node127_sync_last_status = 0U;
volatile uint32_t g_can2_tx_ok_count = 0U;
volatile uint32_t g_can2_tx_fail_count = 0U;
volatile uint32_t g_can2_tx_busy_count = 0U;
volatile uint32_t g_can2_tx_abort_count = 0U;
volatile uint32_t g_can2_tx_mailbox_free = 0U;
volatile uint32_t g_can2_esr_snapshot = 0U;
volatile uint32_t g_can2_msr_snapshot = 0U;
volatile uint32_t g_can2_state_snapshot = 0U;
volatile uint32_t g_node127_warmup_done = 0U;
volatile uint32_t g_node127_last_sync_tick = 0U;
volatile uint32_t g_node127_rx_bus = 0U;   /* 0=none, 2=CAN2. 127模块固定走CAN2，CAN1只控制踝关节 */
volatile uint32_t g_can1_127_rx_count = 0U;  /* 保留给VOFA兼容，固定为0 */
volatile uint32_t g_can1_227_rx_count = 0U;  /* 保留给VOFA兼容，固定为0 */

static uint32_t read_u32_le(const uint8_t *p)
{
    return ((uint32_t)p[0]) |
           ((uint32_t)p[1] << 8) |
           ((uint32_t)p[2] << 16) |
           ((uint32_t)p[3] << 24);
}

static int16_t read_i16_be(const uint8_t *p)
{
    return (int16_t)(((uint16_t)p[0] << 8) | (uint16_t)p[1]);
}

static uint16_t read_u16_be(const uint8_t *p)
{
    return (uint16_t)(((uint16_t)p[0] << 8) | (uint16_t)p[1]);
}

static uint16_t max_u16_3(uint16_t a, uint16_t b, uint16_t c)
{
    uint16_t value = (a > b) ? a : b;
    return (value > c) ? value : c;
}

static void HandleHeartbeat(uint8_t id, const uint8_t *RxData, uint8_t DLC)
{
    if ((id >= MOTOR_CAN_MAX_NODE_ID) || (RxData == NULL) || (DLC < 8U)) {
        return;
    }

    motor_axis_error[id] = read_u32_le(&RxData[0]);
    motor_axis_state[id] = RxData[4];
    motor_axis_flags[id] = RxData[5];
    motor_hb_valid[id] = 1U;
    motor_hb_tick[id] = HAL_GetTick();
}

static void HandleCAN1MotorFrame(uint32_t std_id, const uint8_t *buf, uint8_t dlc)
{
    if ((buf == NULL) || (dlc < 8U)) return;

    if (std_id == calculate_can_id(ANKLE_CAN_NODE_ID, GET_ENCODER_ESTIMATES)) {
        HandleMessage(ANKLE_CAN_NODE_ID, buf, dlc);
    }
    else if (std_id == calculate_can_id(ANKLE_CAN_NODE_ID, HEARTBEAT)) {
        HandleHeartbeat(ANKLE_CAN_NODE_ID, buf, dlc);
    }
}

static void HandleNode127Frame(uint8_t bus, uint32_t std_id, const uint8_t *buf, uint8_t dlc)
{
    static uint16_t pending_sequence;
    static uint16_t pending_emg_front_uv;
    static uint16_t pending_emg_lateral_uv;
    static uint16_t pending_emg_rear_uv;
    static uint8_t pending_emg_valid;
    uint32_t now;
    uint16_t emg_mav;
    uint16_t sequence;

    if ((buf == NULL) || (dlc < 2U)) return;

    now = HAL_GetTick();
    g_node127_rx_bus = bus;

    /* New ID124/F042 EMG module protocol:
     * host sends sync frame 0x010, module replies with StdId 125 in the supplied
     * firmware (accept 124 too, matching the board name). Byte0~1 is big-endian
     * uint16 MAV, the feature value that peaks in VOFA when the user contracts.
     */
    if ((std_id == NODE124_EMG_CAN_ID_PRIMARY) || (std_id == NODE124_EMG_CAN_ID_ALT)) {
        emg_mav = (uint16_t)(((uint16_t)buf[0] << 8) | (uint16_t)buf[1]);
        g_node127.emg = emg_mav;
        g_node127.emg_front_uv = emg_mav;
        g_node127.emg_lateral_uv = emg_mav;
        g_node127.emg_rear_uv = emg_mav;
        g_node127.gx = 0;
        g_node127.gy = 0;
        g_node127.gz = 0;
        g_node127.ax = 0;
        g_node127.ay = 0;
        g_node127.az = 0;
        g_node127.gyro_valid = 1U;
        g_node127.accel_valid = 0U;
        g_node127.updated = 1U;
        g_node127.gyro_tick = now;
        g_node127.last_tick = now;
        g_can2_127_rx_count++;
        return;
    }

    /* Acquisition board protocol. ID127 and ID227 form one atomic sample pair.
     * ID127: sequence, front/lateral/rear thigh EMG MAV in uV.
     * ID227: same sequence, BMI088 gx/gy/gz in Q6 degrees/s.
     */
    if ((buf == NULL) || (dlc < 8U)) return;
    if ((std_id != 127U) && (std_id != 227U)) {
        return;
    }

    if (std_id == 127U) {
        pending_sequence = read_u16_be(&buf[0]);
        pending_emg_front_uv = read_u16_be(&buf[2]);
        pending_emg_lateral_uv = read_u16_be(&buf[4]);
        pending_emg_rear_uv = read_u16_be(&buf[6]);
        pending_emg_valid = 1U;
        g_can2_127_rx_count++;
    }
    else if (std_id == 227U) {
        sequence = read_u16_be(&buf[0]);
        g_node127.last_tick = now;
        g_can2_227_rx_count++;
        if ((pending_emg_valid == 0U) || (sequence != pending_sequence)) {
            pending_emg_valid = 0U;
            return;
        }

        g_node127.sample_sequence = sequence;
        g_node127.emg_front_uv = pending_emg_front_uv;
        g_node127.emg_lateral_uv = pending_emg_lateral_uv;
        g_node127.emg_rear_uv = pending_emg_rear_uv;
        g_node127.emg = max_u16_3(pending_emg_front_uv,
                                 pending_emg_lateral_uv,
                                 pending_emg_rear_uv);
        g_node127.gx = read_i16_be(&buf[2]);
        g_node127.gy = read_i16_be(&buf[4]);
        g_node127.gz = read_i16_be(&buf[6]);
        g_node127.ax = 0;
        g_node127.ay = 0;
        g_node127.az = 0;
        g_node127.gyro_valid = 1U;
        g_node127.accel_valid = 0U;
        g_node127.updated = 1U;
        g_node127.gyro_tick = now;
        g_node127.last_tick = now;
        pending_emg_valid = 0U;
    }
}
static void USER_CAN_ProcessRx(CAN_HandleTypeDef *hcan)
{
    CAN_RxHeaderTypeDef rx_header;
    uint8_t buf[8] = {0};

    if (hcan == NULL) return;

    while (HAL_CAN_GetRxFifoFillLevel(hcan, CAN_RX_FIFO0) > 0U) {
        if (HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &rx_header, buf) != HAL_OK) {
            break;
        }

        if ((rx_header.IDE != CAN_ID_STD) || (rx_header.RTR != CAN_RTR_DATA)) {
            continue;
        }

        if (hcan->Instance == CAN1) {
            /* CAN1 固定用于踝关节电机反馈，不接收/解析 127 数据，避免 127 同步帧或ID判断干扰踝关节CAN总线。 */
            HandleCAN1MotorFrame(rx_header.StdId, buf, rx_header.DLC);
        }
        else if (hcan->Instance == CAN2) {
            /* CAN2固定用于127模块，接收入口完全按 angle_v15_0530：
             * 任何CAN2标准数据帧都先计数并记录最后ID，再按ID=127/227解析。 */
            g_can2_rx_count++;
            g_can2_last_id = rx_header.StdId;
            HandleNode127Frame(2U, rx_header.StdId, buf, rx_header.DLC);
        }
    }
}

void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
    USER_CAN_ProcessRx(hcan);
}

void USER_CAN_PollRx(void)
{
    extern CAN_HandleTypeDef hcan1;
    extern CAN_HandleTypeDef hcan2;
    USER_CAN_ProcessRx(&hcan1);
    USER_CAN_ProcessRx(&hcan2);
}

void HandleMessage(uint8_t id, const uint8_t *RxData, uint8_t DLC)
{
    uint32_t positionRaw;
    uint32_t velocityRaw;

    if ((id >= MOTOR_CAN_MAX_NODE_ID) || (RxData == NULL) || (DLC < 8U)) {
        return;
    }

    positionRaw = read_u32_le(&RxData[0]);
    motor[id].pos = intToFloat(positionRaw);

    velocityRaw = read_u32_le(&RxData[4]);
    motor[id].vel = intToFloat(velocityRaw);

    motor_fb_valid[id] = 1U;
    motor_fb_tick[id] = HAL_GetTick();
}
