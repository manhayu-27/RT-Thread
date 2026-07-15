#include "can_driver.h"
#include "ankle_can_config.h"

/* USER CODE BEGIN 0 */
CAN_TxHeaderTypeDef g_canx_txheader; 
CAN_RxHeaderTypeDef g_canx_rxheader; 
volatile uint32_t g_can1_error_code = 0U;
volatile uint32_t g_can1_busoff_count = 0U;
extern volatile uint32_t g_can2_error_code;
extern volatile uint32_t g_can2_busoff_count;
extern volatile uint32_t g_can2_tx_ok_count;
extern volatile uint32_t g_can2_tx_fail_count;
extern volatile uint32_t g_can2_tx_busy_count;
extern volatile uint32_t g_can2_tx_abort_count;
extern volatile uint32_t g_can2_tx_mailbox_free;
extern volatile uint32_t g_can2_esr_snapshot;
extern volatile uint32_t g_can2_msr_snapshot;
extern volatile uint32_t g_can2_state_snapshot;

/* USER CODE END 0 */

void CAN_Config_1Mbps_Timing(CAN_HandleTypeDef *hcan)
{
  /* CAN2 使用固定时序接收 Node127 数据。
 * 当前 APB1=42 MHz，Prescaler=3、BS1=8、BS2=5，对应 1 Mbps。
 * 不在运行时猜测波特率，避免时钟判断差异导致同步帧接收失败。
 */
  hcan->Init.Prescaler = 4;
  hcan->Init.TimeSeg1 = CAN_BS1_7TQ;
  hcan->Init.TimeSeg2 = CAN_BS2_2TQ;
}

uint8_t can1_send_msg(uint32_t id, uint8_t *msg, uint8_t len)
{
    CAN_TxHeaderTypeDef tx_header;
    uint32_t tx_mailbox;
    uint32_t t0;

    if ((msg == NULL) || (len > 8U) || (id > 0x7FFU)) {
        return 1U;
    }

    t0 = HAL_GetTick();
    while (HAL_CAN_GetTxMailboxesFreeLevel(&hcan1) == 0U) {
        if ((HAL_GetTick() - t0) > 5U) {
            return 2U;
        }
    }

    tx_header.StdId = id;
    tx_header.IDE = CAN_ID_STD;
    tx_header.RTR = CAN_RTR_DATA;
    tx_header.DLC = len;
    tx_header.TransmitGlobalTime = DISABLE;

    if (HAL_CAN_AddTxMessage(&hcan1, &tx_header, msg, &tx_mailbox) != HAL_OK) {
        return 3U;
    }

    return 0U;
}

uint8_t can2_send_msg(uint32_t id, uint8_t *msg, uint8_t len)
{
    /* CAN2 发送直接调用 HAL_CAN_AddTxMessage。
 * 发送过程中不执行 Stop、Start 或 Abort，避免丢失 0x010 同步帧。
 */
    CAN_TxHeaderTypeDef tx_header;
    uint32_t tx_mailbox;
    uint32_t t0;

    if ((msg == NULL) || (len > 8U) || (id > 0x7FFU)) {
        g_can2_tx_fail_count++;
        return 1U;
    }

    t0 = HAL_GetTick();
    while (HAL_CAN_GetTxMailboxesFreeLevel(&hcan2) == 0U) {
        g_can2_tx_busy_count++;
        if ((HAL_GetTick() - t0) > 2U) {
            g_can2_tx_mailbox_free = 0U;
            return 2U;
        }
    }

    g_can2_tx_mailbox_free = HAL_CAN_GetTxMailboxesFreeLevel(&hcan2);
    g_can2_esr_snapshot = CAN2->ESR;
    g_can2_msr_snapshot = CAN2->MSR;
    g_can2_state_snapshot = (uint32_t)hcan2.State;

    tx_header.StdId = id;
    tx_header.IDE = CAN_ID_STD;
    tx_header.RTR = CAN_RTR_DATA;
    tx_header.DLC = len;
    tx_header.TransmitGlobalTime = DISABLE;

    if (HAL_CAN_AddTxMessage(&hcan2, &tx_header, msg, &tx_mailbox) != HAL_OK) {
        g_can2_tx_fail_count++;
        g_can2_error_code = HAL_CAN_GetError(&hcan2);
        return 3U;
    }

    g_can2_tx_ok_count++;
    return 0U;
}


void USER_CAN1_Start(void) {
    CAN_FilterTypeDef can_filter;

    can_filter.FilterBank           = 0;
    can_filter.FilterMode           = CAN_FILTERMODE_IDMASK;
    /* CAN1 只用于踝关节电机，保持全接收并解析踝关节反馈。
 * 不在 CAN1 上发送或解析 Node127 数据，避免干扰踝关节总线。
 */
    can_filter.FilterIdHigh         = 0x0000;
    can_filter.FilterIdLow          = 0x0000;
    can_filter.FilterMaskIdHigh     = 0x0000;
    can_filter.FilterMaskIdLow      = 0x0000;
    can_filter.FilterFIFOAssignment = CAN_FILTER_FIFO0;
    can_filter.FilterScale          = CAN_FILTERSCALE_32BIT;
    can_filter.FilterActivation     = ENABLE;
    can_filter.SlaveStartFilterBank = 14;

    if (HAL_CAN_ConfigFilter(&hcan1, &can_filter) != HAL_OK) {
        g_can1_error_code = 0xC101U;
        return;
    }
    if (HAL_CAN_Start(&hcan1) != HAL_OK) {
        g_can1_error_code = HAL_CAN_GetError(&hcan1) | 0xC1020000U;
        return;
    }
    if (HAL_CAN_ActivateNotification(&hcan1, CAN_IT_RX_FIFO0_MSG_PENDING | CAN_IT_RX_FIFO0_FULL | CAN_IT_RX_FIFO0_OVERRUN | CAN_IT_BUSOFF | CAN_IT_ERROR) != HAL_OK) {
        g_can1_error_code = HAL_CAN_GetError(&hcan1) | 0xC1030000U;
        return;
    }
}

void USER_CAN2_Start(void) {
    CAN_FilterTypeDef can_filter;

    can_filter.FilterBank           = 14;
    can_filter.FilterMode           = CAN_FILTERMODE_IDMASK;
    can_filter.FilterIdHigh         = 0x0000;
    can_filter.FilterIdLow          = 0x0000;
    can_filter.FilterMaskIdHigh      = 0x0000;
    can_filter.FilterMaskIdLow       = 0x0000;
    can_filter.FilterFIFOAssignment = CAN_FILTER_FIFO0;
    can_filter.FilterScale          = CAN_FILTERSCALE_32BIT;
    can_filter.FilterActivation      = ENABLE;
    can_filter.SlaveStartFilterBank = 14;

    if (HAL_CAN_ConfigFilter(&hcan2, &can_filter) != HAL_OK) {
        g_can2_error_code = 0xC201U;
        return;
    }
    if (HAL_CAN_Start(&hcan2) != HAL_OK) {
        g_can2_error_code = HAL_CAN_GetError(&hcan2) | 0xC2020000U;
        return;
    }
    /* CAN2 只开启 RX FIFO0 消息挂起中断。
 * 不开启 ERROR/BUSOFF 中断，避免错误回调重启 CAN2 影响 Node127 同步。
 */
    if (HAL_CAN_ActivateNotification(&hcan2, CAN_IT_RX_FIFO0_MSG_PENDING) != HAL_OK) {
        g_can2_error_code = HAL_CAN_GetError(&hcan2) | 0xC2030000U;
        return;
    }
}

void HAL_CAN_ErrorCallback(CAN_HandleTypeDef *hcan)
{
    uint32_t err;

    if (hcan == NULL) {
        return;
    }

    err = HAL_CAN_GetError(hcan);

    if (hcan->Instance == CAN1) {
        g_can1_error_code = err;
        if ((err & HAL_CAN_ERROR_BOF) != 0U) {
            g_can1_busoff_count++;
            (void)HAL_CAN_Stop(&hcan1);
            (void)HAL_CAN_Start(&hcan1);
            (void)HAL_CAN_ActivateNotification(&hcan1, CAN_IT_RX_FIFO0_MSG_PENDING | CAN_IT_RX_FIFO0_FULL | CAN_IT_RX_FIFO0_OVERRUN | CAN_IT_BUSOFF | CAN_IT_ERROR);
        }
    }
    else if (hcan->Instance == CAN2) {
        /* CAN2 错误回调只记录错误，不反复 Stop/Start。
 * 主循环继续每 2 ms 发送 0x010 同步帧。
 */
        g_can2_error_code = err;
        if ((err & HAL_CAN_ERROR_BOF) != 0U) {
            g_can2_busoff_count++;
        }
    }
}

/* USER CODE END 1 */
