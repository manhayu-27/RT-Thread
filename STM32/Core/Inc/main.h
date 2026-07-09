/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */

/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define BMI_ACC_CS_Pin GPIO_PIN_3
#define BMI_ACC_CS_GPIO_Port GPIOA
#define BMI_GYRO_CS_Pin GPIO_PIN_4
#define BMI_GYRO_CS_GPIO_Port GPIOA
#define ADS_DRDY_Pin GPIO_PIN_10
#define ADS_DRDY_GPIO_Port GPIOB
#define ADS_DRDY_EXTI_IRQn EXTI15_10_IRQn
#define ADS_RESET_PWDN_Pin GPIO_PIN_13
#define ADS_RESET_PWDN_GPIO_Port GPIOB
#define ADS_CS_Pin GPIO_PIN_15
#define ADS_CS_GPIO_Port GPIOA
#define ADS_START_Pin GPIO_PIN_1
#define ADS_START_GPIO_Port GPIOE

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
