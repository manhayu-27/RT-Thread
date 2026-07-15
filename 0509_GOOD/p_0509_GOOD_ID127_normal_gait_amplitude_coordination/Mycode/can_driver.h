#ifndef CAN_DRIVER_H
#define CAN_DRIVER_H

#include "can.h"

extern CAN_TxHeaderTypeDef g_canx_txheader;
extern CAN_RxHeaderTypeDef g_canx_rxheader;

void CAN_Config_1Mbps_Timing(CAN_HandleTypeDef *hcan);
uint8_t can1_send_msg(uint32_t id, uint8_t *msg, uint8_t len);
uint8_t can2_send_msg(uint32_t id, uint8_t *msg, uint8_t len);
void USER_CAN1_Start(void);
void USER_CAN2_Start(void);

extern volatile uint32_t g_can1_error_code;
extern volatile uint32_t g_can1_busoff_count;
extern volatile uint32_t g_can2_error_code;
extern volatile uint32_t g_can2_busoff_count;

#endif
