# 耳电采集板 ESP32-S3FN8 工程

本工程根据 `../Demo/doc/耳电采集.tel` 中的网络连接创建，适用于板上焊接
`ESP32-S3FN8`、未焊接外置 `W25Q128JVPIQ` 的硬件。

## 已配置硬件

| 功能 | ESP32-S3 GPIO | 原理图网络 |
| --- | ---: | --- |
| USB D- | 19 | USB1 D- |
| USB D+ | 20 | USB1 D+ |
| STM32 UART TX | 33 | ESP32_TX |
| STM32 UART RX | 34 | ESP32_RX |
| LCD SCLK | 13 | GPIO13 / LCD pin 9 |
| LCD MOSI | 15 | GPIO15 / LCD pin 10 |
| LCD DC | 16 | GPIO16 / LCD pin 7 |
| LCD RST | 17 | GPIO17 / LCD pin 11 |
| LCD CS | 21 | GPIO21 / LCD pin 8 |
| 编码器 A | 18 | EC_A |
| 编码器 B | 3 | EC_B |
| 编码器按键 | 2 | EC_BT |

Flash 配置为芯片封装内的 8 MB Flash，DIO、80 MHz。工程不会访问未焊接的
`U12 W25Q128JVPIQ`。原理图中 U12 所在的 SPI0/1 网络属于启动 Flash 总线，
应用程序也不应将其作为普通 GPIO 使用。

## 编译和下载

在 ESP-IDF PowerShell 环境中执行：

```powershell
cd D:\Myproject\32project\RT-Thread\ESP32
idf.py set-target esp32s3
idf.py build
idf.py -p COMx flash monitor
```

也可以使用板载原生 USB。首次下载时按住 `GPIO0` 对应的 BOOT 操作，再复位或
重新上电进入下载模式。

## 上电现象

1. USB Serial/JTAG 输出启动日志。
2. LCD 显示六色测试条。
3. STM32 UART 收到 `ESP32-S3FN8 board online`，STM32 回传内容会显示在日志中。
4. 转动或按下编码器时，日志打印位置和按键事件。

## STM32 双向通信协议

STM32 使用 `PA2/USART2_TX -> GPIO34` 和 `PD6/USART2_RX <- GPIO33`，两端均为
`115200, 8N1`。STM32 每秒发送 `STM32,PING,<sequence>`，ESP32 回复
`ESP32,PONG,<sequence>`，STM32 再发送 `STM32,ACK,<sequence>`。USB 日志出现
`UART round trip confirmed` 即表示收发两个方向都正常。

若 LCD 图像整体上下错位，请在 `main/board_pins.h` 中把
`BOARD_LCD_Y_GAP` 从 `80` 改为 `0`；不同批次 1.54 英寸 ST7789 屏的显存起点可能不同。

## 原理图注意事项

`LCD_BL` 网络只连接到背光 MOS 管 `Q4` 的栅极和下拉电阻 `R6`，没有连接到
ESP32-S3。按当前原理图，`Q4` 默认关断，软件无法控制或点亮背光。调试 LCD 时需
先确认实板是否已经飞线、改版或采用其他方式使 `Q4` 导通，否则即使 SPI 初始化
和刷屏正常，屏幕也会表现为无背光黑屏。
