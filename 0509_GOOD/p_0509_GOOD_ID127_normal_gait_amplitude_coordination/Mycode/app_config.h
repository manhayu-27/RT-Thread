#ifndef APP_CONFIG_H
#define APP_CONFIG_H

#include "main.h"

#define RS485_DIR_PORT             GPIOB
#define RS485_DIR_PIN              GPIO_PIN_1
#define RS485_UART_BAUDRATE        4000000U
#define DEBUG_UART_BAUDRATE        115200U

#define BMI088_GYRO_CS_PORT        GPIOA
#define BMI088_GYRO_CS_PIN         GPIO_PIN_0
#define BMI088_ACCEL_CS_PORT       GPIOA
#define BMI088_ACCEL_CS_PIN        GPIO_PIN_1
#define BMI088_GYRO_INT_PORT       GPIOA
#define BMI088_GYRO_INT_PIN        GPIO_PIN_2
#define BMI088_ACCEL_INT_PORT      GPIOA
#define BMI088_ACCEL_INT_PIN       GPIO_PIN_3

#define MT6701_SCK_PORT            GPIOE
#define MT6701_SCK_PIN             GPIO_PIN_2
#define MT6701_CS_PORT             GPIOE
#define MT6701_CS_PIN              GPIO_PIN_4
#define MT6701_MISO_PORT           GPIOE
#define MT6701_MISO_PIN            GPIO_PIN_5

#endif
