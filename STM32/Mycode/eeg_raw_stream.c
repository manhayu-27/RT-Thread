#include <rtthread.h>
#include "main.h"
#include "can.h"
#include "spi.h"
#include "usart.h"
#include "bmi088.h"
#include "eeg_raw_stream.h"
#include "filter.h"
#include <stdio.h>
#include <string.h>
#include <math.h>

#define ADS1194_CMD_WAKEUP      0x02U
#define ADS1194_CMD_STANDBY     0x04U
#define ADS1194_CMD_RESET       0x06U
#define ADS1194_CMD_START       0x08U
#define ADS1194_CMD_STOP        0x0AU
#define ADS1194_CMD_RDATAC      0x10U
#define ADS1194_CMD_SDATAC      0x11U
#define ADS1194_CMD_RDATA       0x12U
#define ADS1194_CMD_RREG        0x20U
#define ADS1194_CMD_WREG        0x40U

#define ADS1194_REG_ID          0x00U
#define ADS1194_REG_CONFIG1     0x01U
#define ADS1194_REG_CONFIG2     0x02U
#define ADS1194_REG_CONFIG3     0x03U
#define ADS1194_REG_CH1SET      0x05U
#define ADS1194_REG_CH2SET      0x06U
#define ADS1194_REG_CH3SET      0x07U
#define ADS1194_REG_CH4SET      0x08U

#define ADS1194_FRAME_BYTES     11U
#define ADS1194_SPI_TIMEOUT_MS  20U
#define ADS1194_THREAD_STACK    1024U
#define ADS1194_UART_DECIMATE   1U
#define ADS1194_VREF_MV         2400.0f
#define ADS1194_GAIN            12.0f

#define SENSOR_CAN_SYNC_ID      0x010U
#define SENSOR_CAN_EMG_ID       127U
#define SENSOR_CAN_GYRO_ID      227U
#define SENSOR_CAN_WINDOW_SIZE  5U
#define SENSOR_CAN_GYRO_Q       64.0f

static volatile uint8_t sensor_can_sync_pending;
static uint16_t sensor_can_sequence;
static float sensor_can_emg_sum_uv[3];
static uint8_t sensor_can_window_count;

static void uart1_send_text(const char *text);

static uint16_t saturate_u16(float value)
{
    if (value <= 0.0f)
    {
        return 0U;
    }
    if (value >= 65535.0f)
    {
        return 65535U;
    }
    return (uint16_t)(value + 0.5f);
}

static int16_t saturate_i16(float value)
{
    if (value <= -32768.0f)
    {
        return (int16_t)-32768;
    }
    if (value >= 32767.0f)
    {
        return 32767;
    }
    return (int16_t)((value >= 0.0f) ? (value + 0.5f) : (value - 0.5f));
}

static void put_u16_be(uint8_t *dst, uint16_t value)
{
    dst[0] = (uint8_t)(value >> 8U);
    dst[1] = (uint8_t)value;
}

static void sensor_can_send(uint32_t std_id, const uint8_t data[8])
{
    CAN_TxHeaderTypeDef header = {0};
    uint32_t mailbox;

    header.StdId = std_id;
    header.IDE = CAN_ID_STD;
    header.RTR = CAN_RTR_DATA;
    header.DLC = 8U;
    header.TransmitGlobalTime = DISABLE;
    (void)HAL_CAN_AddTxMessage(&hcan1, &header, (uint8_t *)data, &mailbox);
}

static void sensor_can_publish(float ch2_uv,
                               float ch3_uv,
                               float ch4_uv,
                               const bmi088_motion_data_t *motion)
{
    uint8_t emg_data[8] = {0U};
    uint8_t gyro_data[8] = {0U};

    if ((sensor_can_sync_pending == 0U) || (motion == RT_NULL))
    {
        return;
    }
    sensor_can_sync_pending = 0U;
    sensor_can_sequence++;

    put_u16_be(&emg_data[0], sensor_can_sequence);
    put_u16_be(&emg_data[2], saturate_u16(ch2_uv));
    put_u16_be(&emg_data[4], saturate_u16(ch3_uv));
    put_u16_be(&emg_data[6], saturate_u16(ch4_uv));

    put_u16_be(&gyro_data[0], sensor_can_sequence);
    put_u16_be(&gyro_data[2], (uint16_t)saturate_i16(motion->gyro.x_dps * SENSOR_CAN_GYRO_Q));
    put_u16_be(&gyro_data[4], (uint16_t)saturate_i16(motion->gyro.y_dps * SENSOR_CAN_GYRO_Q));
    put_u16_be(&gyro_data[6], (uint16_t)saturate_i16(motion->gyro.z_dps * SENSOR_CAN_GYRO_Q));

    sensor_can_send(SENSOR_CAN_EMG_ID, emg_data);
    sensor_can_send(SENSOR_CAN_GYRO_ID, gyro_data);
}

static int sensor_can_init(void)
{
    CAN_FilterTypeDef filter = {0};

    filter.FilterBank = 0U;
    filter.FilterMode = CAN_FILTERMODE_IDMASK;
    filter.FilterScale = CAN_FILTERSCALE_32BIT;
    filter.FilterIdHigh = (uint32_t)(SENSOR_CAN_SYNC_ID << 5U);
    filter.FilterMaskIdHigh = (uint32_t)(0x7FFU << 5U);
    filter.FilterFIFOAssignment = CAN_FILTER_FIFO0;
    filter.FilterActivation = ENABLE;
    filter.SlaveStartFilterBank = 14U;

    if ((HAL_CAN_ConfigFilter(&hcan1, &filter) != HAL_OK) ||
        (HAL_CAN_Start(&hcan1) != HAL_OK) ||
        (HAL_CAN_ActivateNotification(&hcan1, CAN_IT_RX_FIFO0_MSG_PENDING) != HAL_OK))
    {
        uart1_send_text("SENSOR_CAN_ERROR,INIT\r\n");
        return -1;
    }

    uart1_send_text("SENSOR_CAN_READY,1000000,ID127_EMG,ID227_GYRO\r\n");
    return 0;
}

static void uart1_send_text(const char *text)
{
    if (text != RT_NULL)
    {
        (void)HAL_UART_Transmit(&huart1,
                                (uint8_t *)text,
                                (uint16_t)strlen(text),
                                100U);
    }
}

static void uart2_send_text(const char *text)
{
    if (text != RT_NULL)
    {
        (void)HAL_UART_Transmit(&huart2,
                                (uint8_t *)text,
                                (uint16_t)strlen(text),
                                20U);
    }
}

static void ads1194_cs(uint8_t selected)
{
    HAL_GPIO_WritePin(ADS_CS_GPIO_Port,
                      ADS_CS_Pin,
                      selected ? GPIO_PIN_RESET : GPIO_PIN_SET);
}

static HAL_StatusTypeDef ads1194_xfer(const uint8_t *tx,
                                      uint8_t *rx,
                                      uint16_t len)
{
    HAL_StatusTypeDef status;

    ads1194_cs(1U);
    status = HAL_SPI_TransmitReceive(&hspi3,
                                     (uint8_t *)tx,
                                     rx,
                                     len,
                                     ADS1194_SPI_TIMEOUT_MS);
    ads1194_cs(0U);
    HAL_Delay(1U);

    return status;
}

static HAL_StatusTypeDef ads1194_cmd(uint8_t cmd)
{
    uint8_t rx = 0U;

    return ads1194_xfer(&cmd, &rx, 1U);
}

static HAL_StatusTypeDef ads1194_read_reg(uint8_t reg, uint8_t *value)
{
    uint8_t tx[3] = {(uint8_t)(ADS1194_CMD_RREG | reg), 0x00U, 0x00U};
    uint8_t rx[3] = {0U, 0U, 0U};
    HAL_StatusTypeDef status;

    if (value == RT_NULL)
    {
        return HAL_ERROR;
    }

    status = ads1194_xfer(tx, rx, sizeof(tx));
    if (status == HAL_OK)
    {
        *value = rx[2];
    }

    return status;
}

static HAL_StatusTypeDef ads1194_write_reg(uint8_t reg, uint8_t value)
{
    uint8_t tx[3] = {(uint8_t)(ADS1194_CMD_WREG | reg), 0x00U, value};
    uint8_t rx[3] = {0U, 0U, 0U};

    return ads1194_xfer(tx, rx, sizeof(tx));
}

static void ads1194_print_pins(const char *tag)
{
    char line[96];

    (void)snprintf(line,
                   sizeof(line),
                   "ADS1194_PINS,%s,CS=%u,RST=%u,START=%u,DRDY=%u\r\n",
                   tag,
                   (unsigned int)HAL_GPIO_ReadPin(ADS_CS_GPIO_Port, ADS_CS_Pin),
                   (unsigned int)HAL_GPIO_ReadPin(ADS_RESET_PWDN_GPIO_Port,
                                                  ADS_RESET_PWDN_Pin),
                   (unsigned int)HAL_GPIO_ReadPin(ADS_START_GPIO_Port,
                                                  ADS_START_Pin),
                   (unsigned int)HAL_GPIO_ReadPin(ADS_DRDY_GPIO_Port,
                                                  ADS_DRDY_Pin));
    uart1_send_text(line);
}

static int ads1194_read_regs(uint8_t *regs, uint8_t count)
{
    uint8_t index;

    for (index = 0U; index < count; index++)
    {
        if (ads1194_read_reg(index, &regs[index]) != HAL_OK)
        {
            return -1;
        }
    }

    return 0;
}

static void ads1194_print_regs(const char *tag)
{
    uint8_t regs[9] = {0U};
    char line[128];

    if (ads1194_read_regs(regs, sizeof(regs)) != 0)
    {
        uart1_send_text("ADS1194_REGS,READ_ERROR\r\n");
        return;
    }

    (void)snprintf(line,
                   sizeof(line),
                   "ADS1194_REGS,%s,ID=0x%02X,C1=0x%02X,C2=0x%02X,C3=0x%02X,CH=%02X,%02X,%02X,%02X\r\n",
                   tag,
                   regs[ADS1194_REG_ID],
                   regs[ADS1194_REG_CONFIG1],
                   regs[ADS1194_REG_CONFIG2],
                   regs[ADS1194_REG_CONFIG3],
                   regs[ADS1194_REG_CH1SET],
                   regs[ADS1194_REG_CH2SET],
                   regs[ADS1194_REG_CH3SET],
                   regs[ADS1194_REG_CH4SET]);
    uart1_send_text(line);
}

static void ads1194_reset_pins(void)
{
    ads1194_cs(0U);
    HAL_GPIO_WritePin(ADS_START_GPIO_Port, ADS_START_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(ADS_RESET_PWDN_GPIO_Port,
                      ADS_RESET_PWDN_Pin,
                      GPIO_PIN_RESET);
    HAL_Delay(10U);
    HAL_GPIO_WritePin(ADS_RESET_PWDN_GPIO_Port,
                      ADS_RESET_PWDN_Pin,
                      GPIO_PIN_SET);
    HAL_Delay(200U);
}

static int ads1194_configure_raw_input(void)
{
    static const uint8_t init_regs[][2] =
    {
        {ADS1194_REG_CONFIG1, 0x04U},
        {ADS1194_REG_CONFIG2, 0x20U},
        {ADS1194_REG_CONFIG3, 0xCCU},
        {ADS1194_REG_CH1SET,  0x60U},
        {ADS1194_REG_CH2SET,  0x60U},
        {ADS1194_REG_CH3SET,  0x60U},
        {ADS1194_REG_CH4SET,  0x60U}
    };
    uint8_t index;

    for (index = 0U; index < (uint8_t)(sizeof(init_regs) / sizeof(init_regs[0])); index++)
    {
        if (ads1194_write_reg(init_regs[index][0], init_regs[index][1]) != HAL_OK)
        {
            return -1;
        }
    }

    return 0;
}

static int ads1194_wait_drdy(uint32_t timeout_ms)
{
    while (timeout_ms > 0U)
    {
        if (HAL_GPIO_ReadPin(ADS_DRDY_GPIO_Port, ADS_DRDY_Pin) == GPIO_PIN_RESET)
        {
            return 0;
        }
        HAL_Delay(1U);
        timeout_ms--;
    }

    return -1;
}

static int16_t ads1194_decode_ch(const uint8_t *frame, uint8_t channel)
{
    uint8_t offset = (uint8_t)(3U + (2U * channel));

    return (int16_t)(((uint16_t)frame[offset] << 8U) | frame[offset + 1U]);
}

static float ads1194_code_to_mv(int16_t code)
{
    return ((float)code * ADS1194_VREF_MV) / (32768.0f * ADS1194_GAIN);
}

static void ads1194_read_one_frame(uint32_t *sample_count)
{
    uint8_t tx[ADS1194_FRAME_BYTES + 1U] = {ADS1194_CMD_RDATA};
    uint8_t rx[ADS1194_FRAME_BYTES + 1U] = {0U};
    const uint8_t *frame = &rx[1];
    bmi088_motion_data_t motion = {0};
    uint8_t motion_valid;
    uint8_t fall = 0U;
    char line[160];
    float ch1;
    float ch2;
    float ch3;
    float ch4;

    if (ads1194_wait_drdy(1000U) != 0)
    {
        return;
    }

    if (ads1194_xfer(tx, rx, sizeof(rx)) != HAL_OK)
    {
        return;
    }

    if ((frame[0] & 0xF0U) != 0xC0U)
    {
        return;
    }

    (*sample_count)++;
    ch1 = filter_process_sample_channel(0U, ads1194_code_to_mv(ads1194_decode_ch(frame, 0U)));
    ch2 = filter_process_sample_channel(1U, ads1194_code_to_mv(ads1194_decode_ch(frame, 1U)));
    ch3 = filter_process_sample_channel(2U, ads1194_code_to_mv(ads1194_decode_ch(frame, 2U)));
    ch4 = filter_process_sample_channel(3U, ads1194_code_to_mv(ads1194_decode_ch(frame, 3U)));
    motion_valid = bmi088_get_latest_state(&motion, &fall);

    sensor_can_emg_sum_uv[0] += fabsf(ch2) * 1000.0f;
    sensor_can_emg_sum_uv[1] += fabsf(ch3) * 1000.0f;
    sensor_can_emg_sum_uv[2] += fabsf(ch4) * 1000.0f;
    sensor_can_window_count++;
    if (sensor_can_window_count >= SENSOR_CAN_WINDOW_SIZE)
    {
        if (motion_valid != 0U)
        {
            sensor_can_publish(sensor_can_emg_sum_uv[0] / (float)SENSOR_CAN_WINDOW_SIZE,
                               sensor_can_emg_sum_uv[1] / (float)SENSOR_CAN_WINDOW_SIZE,
                               sensor_can_emg_sum_uv[2] / (float)SENSOR_CAN_WINDOW_SIZE,
                               &motion);
        }
        sensor_can_emg_sum_uv[0] = 0.0f;
        sensor_can_emg_sum_uv[1] = 0.0f;
        sensor_can_emg_sum_uv[2] = 0.0f;
        sensor_can_window_count = 0U;
    }

    if ((*sample_count % ADS1194_UART_DECIMATE) != 0U)
    {
        return;
    }

    (void)snprintf(line,
                   sizeof(line),
                   "%.5f,%.5f,%.5f,%.5f,%.2f,%.2f,%.2f,%u,%.2f,%.2f,%.2f,%.2f\r\n",
                   (double)ch1,
                   (double)ch2,
                   (double)ch3,
                   (double)ch4,
                   (double)motion.gyro.x_dps,
                   (double)motion.gyro.y_dps,
                   (double)motion.gyro.z_dps,
                   (unsigned int)fall,
                   (double)motion.roll_deg,
                   (double)motion.pitch_deg,
                   (double)motion.yaw_deg,
                   (double)motion.temperature_c);
    uart2_send_text(line);
}

static void ads1194_test_thread(void *parameter)
{
    uint8_t id = 0U;
    uint32_t sample_count = 0U;

    (void)parameter;
    filter_init();

    uart1_send_text("ADS1194_TEST_BEGIN\r\n");
    ads1194_print_pins("BOOT");
    ads1194_reset_pins();
    ads1194_print_pins("AFTER_PIN_RESET");

    if (ads1194_cmd(ADS1194_CMD_RESET) != HAL_OK)
    {
        uart1_send_text("ADS1194_ERROR,RESET_CMD\r\n");
        return;
    }
    HAL_Delay(10U);

    (void)ads1194_cmd(ADS1194_CMD_SDATAC);
    ads1194_print_regs("DEFAULT");

    if ((ads1194_read_reg(ADS1194_REG_ID, &id) != HAL_OK) ||
        ((id & 0xFCU) != 0xB4U))
    {
        char line[48];

        (void)snprintf(line, sizeof(line), "ADS1194_ERROR,BAD_ID,0x%02X\r\n", id);
        uart1_send_text(line);
        return;
    }

    if (ads1194_configure_raw_input() != 0)
    {
        uart1_send_text("ADS1194_ERROR,CONFIG\r\n");
        return;
    }
    ads1194_print_regs("CONFIGURED");

    HAL_GPIO_WritePin(ADS_START_GPIO_Port, ADS_START_Pin, GPIO_PIN_SET);
    (void)ads1194_cmd(ADS1194_CMD_START);

    while (1)
    {
        ads1194_read_one_frame(&sample_count);
    }
}

int eeg_raw_stream_init(void)
{
    rt_thread_t tid;

    /* CAN failure must not stop ADS1194 -> ESP32 streaming. */
    (void)sensor_can_init();

    tid = rt_thread_create("ads1194t",
                           ads1194_test_thread,
                           RT_NULL,
                           ADS1194_THREAD_STACK,
                           20,
                           2);
    if (tid == RT_NULL)
    {
        uart1_send_text("ADS1194_ERROR,THREAD\r\n");
        return -1;
    }

    rt_thread_startup(tid);
    return 0;
}

void HAL_GPIO_EXTI_Callback(uint16_t gpio_pin)
{
    (void)gpio_pin;
}

void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
    CAN_RxHeaderTypeDef header;
    uint8_t data[8];

    if ((hcan == RT_NULL) || (hcan->Instance != CAN1))
    {
        return;
    }

    while (HAL_CAN_GetRxFifoFillLevel(hcan, CAN_RX_FIFO0) > 0U)
    {
        if (HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &header, data) != HAL_OK)
        {
            break;
        }
        if ((header.IDE == CAN_ID_STD) &&
            (header.RTR == CAN_RTR_DATA) &&
            (header.StdId == SENSOR_CAN_SYNC_ID))
        {
            sensor_can_sync_pending = 1U;
        }
    }
}
