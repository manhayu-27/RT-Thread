/* stm32f4xx_it.c 完整代码 - 解决重启问题的版本 */
#include "main.h"
#include "stm32f4xx_it.h"

/* 声明外部句柄，必须与 can.c 一致 */
extern CAN_HandleTypeDef hcan1;
extern CAN_HandleTypeDef hcan2;
extern UART_HandleTypeDef huart2;
extern UART_HandleTypeDef huart3;
extern volatile uint32_t g_boot_stage_code;
extern volatile uint32_t g_hardfault_count;

/******************************************************************************/
/* Cortex-M4 内核异常处理器                                         */
/******************************************************************************/

void NMI_Handler(void) { }

void HardFault_Handler(void) {
  g_hardfault_count++;
  /* 尽量从USART3吐出硬错误阶段码；即使串口没初始化也不会再继续执行主逻辑。 */
  while (1) {
    const char msg[] = "9998,HARDFAULT\r\n";
    if (huart3.Instance == USART3) {
      (void)HAL_UART_Transmit(&huart3, (uint8_t *)msg, (uint16_t)(sizeof(msg)-1U), 50U);
    }
    if (huart2.Instance == USART2) {
      (void)HAL_UART_Transmit(&huart2, (uint8_t *)msg, (uint16_t)(sizeof(msg)-1U), 50U);
    }
    for (volatile uint32_t i = 0; i < 800000U; ++i) { __NOP(); }
  }
}

void MemManage_Handler(void) {
  while (1) { }
}

void BusFault_Handler(void) {
  while (1) { }
}

void UsageFault_Handler(void) {
  while (1) { }
}

void SVC_Handler(void) { }

void DebugMon_Handler(void) { }

void PendSV_Handler(void) { }

/**
  * @brief 处理系统滴答定时器（HAL_Delay依赖此中断）
  */
void SysTick_Handler(void)
{
  HAL_IncTick();
}

/******************************************************************************/
/* STM32F4xx 外设中断处理器                                         */
/******************************************************************************/

/**
  * @brief 处理 CAN1 发送中断
  */
void CAN1_TX_IRQHandler(void)
{
  HAL_CAN_IRQHandler(&hcan1);
}

/**
  * @brief 处理 CAN1 接收 FIFO 0 中断（西格玛电机回包核心入口）
  */
void CAN1_RX0_IRQHandler(void)
{
  HAL_CAN_IRQHandler(&hcan1);
}

void CAN1_RX1_IRQHandler(void)
{
  HAL_CAN_IRQHandler(&hcan1);
}

/**
  * @brief 处理 CAN1 错误中断
  */
void CAN1_SCE_IRQHandler(void)
{
  HAL_CAN_IRQHandler(&hcan1);
}


/**
  * @brief 处理 CAN2 发送中断（127模块同步帧发送）
  */
void CAN2_TX_IRQHandler(void)
{
  HAL_CAN_IRQHandler(&hcan2);
}

/**
  * @brief 处理 CAN2 接收 FIFO0 中断（127模块 ID=127/227 数据入口）
  */
void CAN2_RX0_IRQHandler(void)
{
  HAL_CAN_IRQHandler(&hcan2);
}

void CAN2_RX1_IRQHandler(void)
{
  HAL_CAN_IRQHandler(&hcan2);
}

/**
  * @brief 处理 CAN2 错误中断
  */
void CAN2_SCE_IRQHandler(void)
{
  HAL_CAN_IRQHandler(&hcan2);
}
