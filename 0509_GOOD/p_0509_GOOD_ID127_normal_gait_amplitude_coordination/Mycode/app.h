#ifndef APP_H
#define APP_H

#include "main.h"

extern UART_HandleTypeDef huart1;
extern UART_HandleTypeDef huart2;
extern UART_HandleTypeDef huart3;
extern SPI_HandleTypeDef hspi1;

void App_Init(void);
void App_RunOnce(void);
void App_ErrorHandlerStep(void);
void App_TestJointMotor(uint16_t motor_id, float angle_deg, float speed_deg_s);

#endif
