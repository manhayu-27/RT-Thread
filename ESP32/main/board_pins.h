#pragma once

/* ESP32-S3-WROOM-1U pins derived from doc/采集板.tel. */

/* Native USB Serial/JTAG */
#define BOARD_USB_DM_GPIO       19
#define BOARD_USB_DP_GPIO       20

/* Router Wi-Fi used by ESP32 STA mode. Fill these before flashing. */
#define BOARD_WIFI_STA_SSID       "Redmi K40"
#define BOARD_WIFI_STA_PASSWORD   "228425529syh"

/* Set to 0 after the browser/WebSocket test to restore STM32 acquisition. */
#define BOARD_ENABLE_WEB_TEST_SIGNAL 0

/* U13.19/U13.20: ESP32_TX/ESP32_RX nets to the STM32F407. */
#define BOARD_STM32_UART_TX_GPIO 11
#define BOARD_STM32_UART_RX_GPIO 12

/* U13.10/U13.11/U13.12: RX_GPS/TX_GPS/PPS from the board netlist. */
#define BOARD_GPS_UART_TX_GPIO   17
#define BOARD_GPS_UART_RX_GPIO   18
#define BOARD_GPS_PPS_GPIO        8

/* 1.54-inch 240x240 SPI LCD */
#define BOARD_ENABLE_LCD          1
#define BOARD_LCD_SCLK_GPIO      13
#define BOARD_LCD_MOSI_GPIO      15
#define BOARD_LCD_DC_GPIO        16
#define BOARD_LCD_RST_GPIO       -1
#define BOARD_LCD_CS_GPIO        21
#define BOARD_LCD_BL_GPIO        14

/*
 * The new schematic has no rotary-encoder connections. Keep the historical
 * pin assignments documented, but do not configure or sample them.
 */
#define BOARD_ENABLE_ENCODER       0
#define BOARD_ENCODER_A_GPIO     18
#define BOARD_ENCODER_B_GPIO      3
#define BOARD_ENCODER_KEY_GPIO    2

#define BOARD_LCD_WIDTH          240
#define BOARD_LCD_HEIGHT         240

/* Emma's ST7789 driver uses a native 240x240 window with no row offset. */
#define BOARD_LCD_X_GAP            0
#define BOARD_LCD_Y_GAP            0
