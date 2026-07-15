#include "main.h"
#include "motor_control.h"
#include "crc_ccitt.h"
#include <stdint.h>
#include <string.h> // 增加字符串库支持

/* --- 硬件方向控制：PB1 --- */
#define RS485_DIR_PORT  GPIOB 
#define RS485_DIR_PIN   GPIO_PIN_1 

#define MOTOR_UART_TIMEOUT_MS   3U  // 避免单个485电机无响应时长期阻塞主循环/VOFA
#define SATURATE(value, minv, maxv) \
    do { \
        if ((value) < (minv)) { (value) = (minv); } \
        else if ((value) > (maxv)) { (value) = (maxv); } \
    } while (0)

// 外部硬件句柄声明
extern UART_HandleTypeDef huart1;
extern CAN_HandleTypeDef hcan1; // 新增：CAN 句柄

/* 内部函数声明 */
static void MotorUartFlushRx(void);
int modify_data(MOTOR_send *motor_s);
int extract_data(MOTOR_recv *motor_r);

static void MotorUartFlushRx(void)
{
    while (__HAL_UART_GET_FLAG(&huart1, UART_FLAG_RXNE) != RESET)
    {
        __HAL_UART_FLUSH_DRREGISTER(&huart1);
    }
    __HAL_UART_CLEAR_OREFLAG(&huart1);
}

/* 宇树协议封装 */
int modify_data(MOTOR_send *motor_s)
{
    if (motor_s == 0) return -1;
    motor_s->hex_len = (int)sizeof(ControlData_t);
    motor_s->motor_send_data.head[0] = 0xFEU;
    motor_s->motor_send_data.head[1] = 0xEEU;

    SATURATE(motor_s->K_P,  0.0f,      25.599f);
    SATURATE(motor_s->K_W,  0.0f,      25.599f);
    SATURATE(motor_s->T,   -127.99f,  127.99f);
    SATURATE(motor_s->W,   -804.0f,    804.0f);
    SATURATE(motor_s->Pos, -411774.0f, 411774.0f);

    motor_s->motor_send_data.mode.id     = (uint8_t)(motor_s->id & 0x0FU);
    motor_s->motor_send_data.mode.status = (uint8_t)(motor_s->mode & 0x07U);
    motor_s->motor_send_data.comd.k_pos   = (int16_t)(motor_s->K_P / 25.6f * 32768.0f);
    motor_s->motor_send_data.comd.k_spd   = (int16_t)(motor_s->K_W / 25.6f * 32768.0f);
    motor_s->motor_send_data.comd.pos_des = (int32_t)(motor_s->Pos / 6.2832f * 32768.0f);
    motor_s->motor_send_data.comd.spd_des = (int16_t)(motor_s->W / 6.2832f * 256.0f);
    motor_s->motor_send_data.comd.tor_des = (int16_t)(motor_s->T * 256.0f);
    motor_s->motor_send_data.CRC16 = crc_ccitt(0U, (const uint8_t *)&motor_s->motor_send_data, 15U);
    return 0;
}

/* 宇树协议解析 */
int extract_data(MOTOR_recv *motor_r)
{
    if (motor_r == 0) return 0;
    if (motor_r->motor_recv_data.CRC16 != crc_ccitt(0U, (const uint8_t *)&motor_r->motor_recv_data, 14U))
    {
        motor_r->correct = 0;
        return 0;
    }
    motor_r->motor_id   = motor_r->motor_recv_data.mode.id;
    motor_r->mode       = motor_r->motor_recv_data.mode.status;
    motor_r->Temp       = motor_r->motor_recv_data.fbk.temp;
    motor_r->MError     = (unsigned char)motor_r->motor_recv_data.fbk.MError;
    motor_r->W          = ((float)motor_r->motor_recv_data.fbk.speed / 256.0f) * 6.2832f;
    motor_r->T          = ((float)motor_r->motor_recv_data.fbk.torque) / 256.0f;
    motor_r->Pos        = 6.2832f * ((float)motor_r->motor_recv_data.fbk.pos) / 32768.0f;
    motor_r->footForce  = (float)motor_r->motor_recv_data.fbk.force;
    motor_r->correct    = 1;
    return 1;
}

/* 宇树 485 物理层收发实现 */
HAL_StatusTypeDef SERVO_Send_recv(MOTOR_send *pData, MOTOR_recv *rData)
{
    HAL_StatusTypeDef status;
    uint16_t rxlen = 0U;

    if ((pData == 0) || (rData == 0)) return HAL_ERROR;

    modify_data(pData);
    
    __HAL_UART_CLEAR_FLAG(&huart1, UART_FLAG_ORE | UART_FLAG_NE | UART_FLAG_FE | UART_FLAG_PE);
    MotorUartFlushRx();

    HAL_GPIO_WritePin(RS485_DIR_PORT, RS485_DIR_PIN, GPIO_PIN_SET);
    status = HAL_UART_Transmit(&huart1, (uint8_t *)&pData->motor_send_data, (uint16_t)sizeof(ControlData_t), MOTOR_UART_TIMEOUT_MS);
    {
        uint32_t t0 = HAL_GetTick();
        while(__HAL_UART_GET_FLAG(&huart1, UART_FLAG_TC) == RESET) {
            if ((HAL_GetTick() - t0) > MOTOR_UART_TIMEOUT_MS) {
                HAL_GPIO_WritePin(RS485_DIR_PORT, RS485_DIR_PIN, GPIO_PIN_RESET);
                return HAL_TIMEOUT;
            }
        }
    }

    HAL_GPIO_WritePin(RS485_DIR_PORT, RS485_DIR_PIN, GPIO_PIN_RESET);
    __HAL_UART_CLEAR_FLAG(&huart1, UART_FLAG_ORE | UART_FLAG_NE | UART_FLAG_FE | UART_FLAG_PE);
    __HAL_UART_FLUSH_DRREGISTER(&huart1);

    status = HAL_UARTEx_ReceiveToIdle(&huart1, (uint8_t *)&rData->motor_recv_data, (uint16_t)sizeof(MotorData_t), &rxlen, MOTOR_UART_TIMEOUT_MS);
    
    if (status != HAL_OK) return status;
    if (rxlen != (uint16_t)sizeof(MotorData_t)) return HAL_ERROR;
    if (extract_data(rData) == 0) return HAL_ERROR;

    rData->hex_len = (int)sizeof(MotorData_t);
    return HAL_OK;
}

/* --- 以下为新增融合逻辑 --- */

/**
 * @brief 统一电机通信调度入口
 * 应用层（main.c）直接调用此函数，根据电机类型自动分流指令
 */
HAL_StatusTypeDef Motor_Comm_Execute(MOTOR_send *pData, MOTOR_recv *rData)
{
    if (pData == NULL || rData == NULL) return HAL_ERROR;

    if (pData->motor_type == MOTOR_TYPE_UNITREE_485) {
        // 走原有的 485 驱动
        return SERVO_Send_recv(pData, rData);
    } 
    else if (pData->motor_type == MOTOR_TYPE_SIGMA_CAN) {
        // 走新增的 CAN 驱动
        return CAN_Sigma_Send_recv(pData, rData);
    }
    return HAL_ERROR;
}

/**
 * @brief 西格玛 CAN 电机物理层发送
 */
HAL_StatusTypeDef CAN_Sigma_Send_recv(MOTOR_send *pData, MOTOR_recv *rData)
{
    CAN_TxHeaderTypeDef TxHeader;
    uint32_t TxMailbox;
    uint8_t payload[8] = {0};

    // 1. 配置 CAN 帧头
    TxHeader.StdId = pData->id; 
    TxHeader.IDE   = CAN_ID_STD;
    TxHeader.RTR   = CAN_RTR_DATA;
    TxHeader.DLC   = 8;
    TxHeader.TransmitGlobalTime = DISABLE;

    // 2. 协议打包（这里以常用的位置+增益模式映射为例）
    // 注意：西格玛的具体缩放因子请对照你的协议手册
    int16_t p_target = (int16_t)(pData->Pos * 100.0f);
    int16_t v_target = (int16_t)(pData->W * 10.0f);
    
    payload[0] = (uint8_t)(p_target >> 8);
    payload[1] = (uint8_t)(p_target & 0xFF);
    payload[2] = (uint8_t)(v_target >> 8);
    payload[3] = (uint8_t)(v_target & 0xFF);
    // ... 其他字节填充 ...

    // 3. 发送
    if (HAL_CAN_AddTxMessage(&hcan1, &TxHeader, payload, &TxMailbox) != HAL_OK) {
        return HAL_ERROR;
    }
    
    // 4. 模拟反馈
    rData->correct = 1; 
    rData->motor_id = (uint8_t)pData->id;
    return HAL_OK;
}
