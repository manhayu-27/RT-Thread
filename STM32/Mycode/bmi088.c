#include <rtthread.h>
#include "main.h"
#include "spi.h"
#include "usart.h"
#include "bmi088.h"
#include <math.h>
#include <stdio.h>
#include <string.h>

#define BMI088_SPI_TIMEOUT_MS       10U
#define BMI088_ACC_CHIP_ID_REG      0x00U
#define BMI088_ACC_CHIP_ID_VALUE    0x1EU
#define BMI088_ACC_DATA_REG         0x12U
#define BMI088_ACC_TEMP_REG         0x22U
#define BMI088_ACC_CONF_REG         0x40U
#define BMI088_ACC_RANGE_REG        0x41U
#define BMI088_ACC_PWR_CONF_REG     0x7CU
#define BMI088_ACC_PWR_CTRL_REG     0x7DU
#define BMI088_GYRO_CHIP_ID_REG     0x00U
#define BMI088_GYRO_CHIP_ID_VALUE   0x0FU
#define BMI088_GYRO_DATA_REG        0x02U
#define BMI088_GYRO_RANGE_REG       0x0FU
#define BMI088_GYRO_BANDWIDTH_REG   0x10U
#define BMI088_GYRO_LPM1_REG        0x11U
#define BMI088_GYRO_SOFTRESET_REG   0x14U
#define BMI088_GYRO_SOFTRESET_VALUE 0xB6U

#define BMI088_ACC_CONF_100HZ       0xA8U
#define BMI088_ACC_RANGE_6G         0x01U
#define BMI088_ACC_ACTIVE_MODE      0x00U
#define BMI088_ACC_ENABLE           0x04U
#define BMI088_ACC_G_PER_LSB        (6.0f / 32768.0f)
#define BMI088_GYRO_RANGE_500_DPS   0x02U
#define BMI088_GYRO_BW_1000_116_HZ  0x82U
#define BMI088_GYRO_NORMAL_MODE     0x00U
#define BMI088_GYRO_DPS_PER_LSB     (500.0f / 32768.0f)
#define BMI088_DEBUG_STREAM_ENABLE  0U
#define BMI088_PRINT_INTERVAL_MS    10U
#define BMI088_RAD_TO_DEG           57.2957795f
#define BMI088_PITCH_ACCEL_ALPHA    0.02f
#define BMI088_FALL_FLEX_DEG        45.0f
#define BMI088_FALL_Y_G             0.70f
#define BMI088_FALL_FREE_G_SQ       (0.60f * 0.60f)
#define BMI088_FALL_IMPACT_G_SQ     (1.80f * 1.80f)
#define BMI088_FALL_CONFIRM_COUNT   3U
#define BMI088_FULL_CIRCLE_DEG      360.0f
#define BMI088_HALF_CIRCLE_DEG      180.0f

static uint8_t gyro_chip_id;
static uint8_t accel_chip_id;
static volatile float latest_gyro_x_dps;
static volatile float latest_gyro_y_dps;
static volatile float latest_gyro_z_dps;
static volatile float latest_roll_deg;
static volatile float latest_pitch_deg;
static volatile float latest_yaw_deg;
static volatile float latest_temperature_c;
static volatile uint8_t latest_fall;
static volatile uint8_t latest_motion_valid;
static volatile uint8_t motion_stream_to_esp32;

static void accel_select(void)
{
    HAL_GPIO_WritePin(BMI_ACC_CS_GPIO_Port,
                      BMI_ACC_CS_Pin,
                      GPIO_PIN_RESET);
}

static void accel_deselect(void)
{
    HAL_GPIO_WritePin(BMI_ACC_CS_GPIO_Port,
                      BMI_ACC_CS_Pin,
                      GPIO_PIN_SET);
}

static void gyro_select(void)
{
    HAL_GPIO_WritePin(BMI_GYRO_CS_GPIO_Port,
                      BMI_GYRO_CS_Pin,
                      GPIO_PIN_RESET);
}

static void gyro_deselect(void)
{
    HAL_GPIO_WritePin(BMI_GYRO_CS_GPIO_Port,
                      BMI_GYRO_CS_Pin,
                      GPIO_PIN_SET);
}

static HAL_StatusTypeDef gyro_write_register(uint8_t address, uint8_t value)
{
    uint8_t tx_data[2];
    HAL_StatusTypeDef status;

    tx_data[0] = (uint8_t)(address & 0x7FU);
    tx_data[1] = value;

    gyro_select();
    status = HAL_SPI_Transmit(&hspi1,
                              tx_data,
                              sizeof(tx_data),
                              BMI088_SPI_TIMEOUT_MS);
    gyro_deselect();
    return status;
}

static HAL_StatusTypeDef gyro_read_registers(uint8_t address,
                                             uint8_t *data,
                                             uint16_t length)
{
    uint8_t tx_data[7] = {0U};
    uint8_t rx_data[7] = {0U};
    HAL_StatusTypeDef status;

    if ((data == RT_NULL) || (length == 0U) || (length > 6U))
    {
        return HAL_ERROR;
    }

    tx_data[0] = (uint8_t)(address | 0x80U);
    gyro_select();
    status = HAL_SPI_TransmitReceive(&hspi1,
                                     tx_data,
                                     rx_data,
                                     (uint16_t)(length + 1U),
                                     BMI088_SPI_TIMEOUT_MS);
    gyro_deselect();

    if (status == HAL_OK)
    {
        memcpy(data, &rx_data[1], length);
    }

    return status;
}

static HAL_StatusTypeDef accel_write_register(uint8_t address, uint8_t value)
{
    uint8_t tx_data[2];
    HAL_StatusTypeDef status;

    tx_data[0] = (uint8_t)(address & 0x7FU);
    tx_data[1] = value;

    accel_select();
    status = HAL_SPI_Transmit(&hspi1,
                              tx_data,
                              sizeof(tx_data),
                              BMI088_SPI_TIMEOUT_MS);
    accel_deselect();
    HAL_Delay(1U);
    return status;
}

static HAL_StatusTypeDef accel_read_registers(uint8_t address,
                                              uint8_t *data,
                                              uint16_t length)
{
    uint8_t tx_data[8] = {0U};
    uint8_t rx_data[8] = {0U};
    HAL_StatusTypeDef status;

    if ((data == RT_NULL) || (length == 0U) || (length > 6U))
    {
        return HAL_ERROR;
    }

    tx_data[0] = (uint8_t)(address | 0x80U);
    accel_select();
    status = HAL_SPI_TransmitReceive(&hspi1,
                                     tx_data,
                                     rx_data,
                                     (uint16_t)(length + 2U),
                                     BMI088_SPI_TIMEOUT_MS);
    accel_deselect();

    if (status == HAL_OK)
    {
        memcpy(data, &rx_data[2], length);
    }

    return status;
}

static int16_t decode_le_i16(const uint8_t *data)
{
    return (int16_t)(((uint16_t)data[1] << 8U) | data[0]);
}

static void debug_uart_send(const char *text)
{
    if (text != RT_NULL)
    {
        (void)HAL_UART_Transmit(&huart1,
                                (uint8_t *)text,
                                (uint16_t)strlen(text),
                                100U);
    }
}

static void esp32_uart_send(const char *text)
{
    if (text != RT_NULL)
    {
        (void)HAL_UART_Transmit(&huart2,
                                (uint8_t *)text,
                                (uint16_t)strlen(text),
                                10U);
    }
}

int bmi088_init(void)
{
    uint8_t value = 0U;

    accel_deselect();
    gyro_deselect();
    HAL_Delay(50U);

    /* The first access switches the accelerometer from I2C to SPI mode. */
    (void)accel_read_registers(BMI088_ACC_CHIP_ID_REG, &value, 1U);
    HAL_Delay(1U);
    if (accel_read_registers(BMI088_ACC_CHIP_ID_REG, &value, 1U) != HAL_OK)
    {
        return -11;
    }
    if (value != BMI088_ACC_CHIP_ID_VALUE)
    {
        accel_chip_id = value;
        return -12;
    }
    accel_chip_id = value;

    if (accel_write_register(BMI088_ACC_PWR_CTRL_REG,
                             BMI088_ACC_ENABLE) != HAL_OK)
    {
        return -13;
    }
    HAL_Delay(2U);
    if (accel_write_register(BMI088_ACC_PWR_CONF_REG,
                             BMI088_ACC_ACTIVE_MODE) != HAL_OK)
    {
        return -14;
    }
    if (accel_write_register(BMI088_ACC_CONF_REG,
                             BMI088_ACC_CONF_100HZ) != HAL_OK)
    {
        return -15;
    }
    if (accel_write_register(BMI088_ACC_RANGE_REG,
                             BMI088_ACC_RANGE_6G) != HAL_OK)
    {
        return -16;
    }

    /* The first access selects the gyroscope SPI interface. */
    (void)gyro_read_registers(BMI088_GYRO_CHIP_ID_REG, &value, 1U);
    HAL_Delay(1U);
    if (gyro_read_registers(BMI088_GYRO_CHIP_ID_REG, &value, 1U) != HAL_OK)
    {
        return -1;
    }
    if (value != BMI088_GYRO_CHIP_ID_VALUE)
    {
        gyro_chip_id = value;
        return -2;
    }

    if (gyro_write_register(BMI088_GYRO_SOFTRESET_REG,
                            BMI088_GYRO_SOFTRESET_VALUE) != HAL_OK)
    {
        return -3;
    }
    HAL_Delay(80U);

    if ((gyro_read_registers(BMI088_GYRO_CHIP_ID_REG, &value, 1U) != HAL_OK) ||
        (value != BMI088_GYRO_CHIP_ID_VALUE))
    {
        gyro_chip_id = value;
        return -4;
    }
    gyro_chip_id = value;

    if (gyro_write_register(BMI088_GYRO_RANGE_REG,
                            BMI088_GYRO_RANGE_500_DPS) != HAL_OK)
    {
        return -5;
    }
    if (gyro_write_register(BMI088_GYRO_BANDWIDTH_REG,
                            BMI088_GYRO_BW_1000_116_HZ) != HAL_OK)
    {
        return -6;
    }
    if (gyro_write_register(BMI088_GYRO_LPM1_REG,
                            BMI088_GYRO_NORMAL_MODE) != HAL_OK)
    {
        return -7;
    }
    HAL_Delay(30U);

    if ((gyro_read_registers(BMI088_GYRO_RANGE_REG, &value, 1U) != HAL_OK) ||
        (value != BMI088_GYRO_RANGE_500_DPS))
    {
        return -8;
    }
    if ((gyro_read_registers(BMI088_GYRO_BANDWIDTH_REG, &value, 1U) != HAL_OK) ||
        (value != BMI088_GYRO_BW_1000_116_HZ))
    {
        return -9;
    }

    return 0;
}

int bmi088_read_accel(bmi088_accel_data_t *data)
{
    uint8_t raw[6];
    int16_t x;
    int16_t y;
    int16_t z;

    if (data == RT_NULL)
    {
        return -1;
    }

    if (accel_read_registers(BMI088_ACC_DATA_REG,
                             raw,
                             sizeof(raw)) != HAL_OK)
    {
        return -2;
    }

    x = decode_le_i16(&raw[0]);
    y = decode_le_i16(&raw[2]);
    z = decode_le_i16(&raw[4]);
    data->x_g = (float)x * BMI088_ACC_G_PER_LSB;
    data->y_g = (float)y * BMI088_ACC_G_PER_LSB;
    data->z_g = (float)z * BMI088_ACC_G_PER_LSB;
    return 0;
}

int bmi088_read_temperature(float *temperature_c)
{
    uint8_t raw[2];
    int16_t code;

    if (temperature_c == RT_NULL)
    {
        return -1;
    }

    if (accel_read_registers(BMI088_ACC_TEMP_REG,
                             raw,
                             sizeof(raw)) != HAL_OK)
    {
        return -2;
    }

    code = (int16_t)((((uint16_t)raw[0] << 3U) | (raw[1] >> 5U)) & 0x07FFU);
    if ((code & 0x0400) != 0)
    {
        code |= (int16_t)0xF800;
    }
    *temperature_c = 23.0f + ((float)code * 0.125f);
    return 0;
}

int bmi088_read_gyro(bmi088_gyro_data_t *data)
{
    uint8_t raw[6];
    int16_t x;
    int16_t y;
    int16_t z;

    if (data == RT_NULL)
    {
        return -1;
    }

    if (gyro_read_registers(BMI088_GYRO_DATA_REG,
                            raw,
                            sizeof(raw)) != HAL_OK)
    {
        return -2;
    }

    x = decode_le_i16(&raw[0]);
    y = decode_le_i16(&raw[2]);
    z = decode_le_i16(&raw[4]);
    data->x_dps = (float)x * BMI088_GYRO_DPS_PER_LSB;
    data->y_dps = (float)y * BMI088_GYRO_DPS_PER_LSB;
    data->z_dps = (float)z * BMI088_GYRO_DPS_PER_LSB;
    return 0;
}

uint8_t bmi088_get_latest_motion(bmi088_gyro_data_t *gyro, uint8_t *fall)
{
    uint8_t valid;

    __disable_irq();
    valid = latest_motion_valid;
    if (gyro != RT_NULL)
    {
        gyro->x_dps = latest_gyro_x_dps;
        gyro->y_dps = latest_gyro_y_dps;
        gyro->z_dps = latest_gyro_z_dps;
    }
    if (fall != RT_NULL)
    {
        *fall = latest_fall;
    }
    __enable_irq();

    return valid;
}

uint8_t bmi088_get_latest_state(bmi088_motion_data_t *motion, uint8_t *fall)
{
    uint8_t valid;

    __disable_irq();
    valid = latest_motion_valid;
    if (motion != RT_NULL)
    {
        motion->gyro.x_dps = latest_gyro_x_dps;
        motion->gyro.y_dps = latest_gyro_y_dps;
        motion->gyro.z_dps = latest_gyro_z_dps;
        motion->roll_deg = latest_roll_deg;
        motion->pitch_deg = latest_pitch_deg;
        motion->yaw_deg = latest_yaw_deg;
        motion->temperature_c = latest_temperature_c;
    }
    if (fall != RT_NULL)
    {
        *fall = latest_fall;
    }
    __enable_irq();

    return valid;
}

void bmi088_set_motion_stream_to_esp32(uint8_t enabled)
{
    motion_stream_to_esp32 = (enabled != 0U) ? 1U : 0U;
}

static void calc_euler_deg(const bmi088_accel_data_t *accel,
                           const bmi088_gyro_data_t *gyro,
                           float *roll_deg,
                           float *pitch_deg,
                           float *yaw_deg)
{
    static uint8_t angle_ready;
    static uint32_t last_tick;
    uint32_t now_tick = rt_tick_get();
    float dt_s = 0.0f;
    float accel_pitch_deg;

    if ((accel == RT_NULL) || (gyro == RT_NULL) ||
        (roll_deg == RT_NULL) || (pitch_deg == RT_NULL) || (yaw_deg == RT_NULL))
    {
        return;
    }

    *roll_deg = atan2f(accel->x_g, -accel->y_g) * BMI088_RAD_TO_DEG;
    accel_pitch_deg = atan2f(accel->z_g,
                             sqrtf((accel->x_g * accel->x_g) +
                                   (accel->y_g * accel->y_g))) * BMI088_RAD_TO_DEG;

    if (angle_ready != 0U)
    {
        dt_s = (float)(now_tick - last_tick) / (float)RT_TICK_PER_SECOND;
        latest_pitch_deg = ((1.0f - BMI088_PITCH_ACCEL_ALPHA) *
                            (latest_pitch_deg + gyro->x_dps * dt_s)) +
                           (BMI088_PITCH_ACCEL_ALPHA * accel_pitch_deg);
        latest_yaw_deg += gyro->z_dps * dt_s;
        if (latest_yaw_deg > BMI088_HALF_CIRCLE_DEG)
        {
            latest_yaw_deg -= BMI088_FULL_CIRCLE_DEG;
        }
        else if (latest_yaw_deg < -BMI088_HALF_CIRCLE_DEG)
        {
            latest_yaw_deg += BMI088_FULL_CIRCLE_DEG;
        }
    }
    else
    {
        latest_pitch_deg = accel_pitch_deg;
        angle_ready = 1U;
    }
    last_tick = now_tick;
    *pitch_deg = latest_pitch_deg;
    *yaw_deg = latest_yaw_deg;
}

float bmi088_calc_flex_x_deg(const bmi088_accel_data_t *accel)
{
    float angle_deg;

    if (accel == RT_NULL)
    {
        return 0.0f;
    }

    /*
     * User mounting convention:
     * Y = gravity/vertical reference, Z = thigh anterior/front, X = left-right.
     */
    angle_deg = atan2f(accel->z_g, -accel->y_g) * BMI088_RAD_TO_DEG;
    if (angle_deg < 0.0f)
    {
        angle_deg += BMI088_FULL_CIRCLE_DEG;
    }

    return angle_deg;
}

uint8_t bmi088_detect_fall(const bmi088_accel_data_t *accel,
                           float flex_x_deg)
{
    static uint8_t confirm_count;
    float acc_g_sq;
    float flex_from_upright_deg;
    uint8_t abnormal;

    if (accel == RT_NULL)
    {
        return 0U;
    }

    acc_g_sq = (accel->x_g * accel->x_g) +
               (accel->y_g * accel->y_g) +
               (accel->z_g * accel->z_g);
    flex_from_upright_deg = (flex_x_deg > BMI088_HALF_CIRCLE_DEG) ?
                            (BMI088_FULL_CIRCLE_DEG - flex_x_deg) :
                            flex_x_deg;
    abnormal = ((flex_from_upright_deg >= BMI088_FALL_FLEX_DEG) ||
                (fabsf(accel->y_g) <= BMI088_FALL_Y_G) ||
                (acc_g_sq <= BMI088_FALL_FREE_G_SQ) ||
                (acc_g_sq >= BMI088_FALL_IMPACT_G_SQ)) ? 1U : 0U;

    if (abnormal != 0U)
    {
        if (confirm_count < BMI088_FALL_CONFIRM_COUNT)
        {
            confirm_count++;
        }
    }
    else
    {
        confirm_count = 0U;
    }

    return (confirm_count >= BMI088_FALL_CONFIRM_COUNT) ? 1U : 0U;
}

static void bmi088_thread_entry(void *parameter)
{
    bmi088_accel_data_t accel;
    bmi088_gyro_data_t gyro;
    float flex_x_deg;
    float roll_deg;
    float pitch_deg;
    float yaw_deg;
    float temperature_c = 0.0f;
    uint8_t fall;
    char line[180];

    (void)parameter;

    while (1)
    {
        if ((bmi088_read_accel(&accel) == 0) &&
            (bmi088_read_gyro(&gyro) == 0))
        {
            flex_x_deg = bmi088_calc_flex_x_deg(&accel);
            calc_euler_deg(&accel, &gyro, &roll_deg, &pitch_deg, &yaw_deg);
            (void)bmi088_read_temperature(&temperature_c);
            fall = bmi088_detect_fall(&accel, flex_x_deg);
            __disable_irq();
            latest_gyro_x_dps = gyro.x_dps;
            latest_gyro_y_dps = gyro.y_dps;
            latest_gyro_z_dps = gyro.z_dps;
            latest_roll_deg = roll_deg;
            latest_pitch_deg = pitch_deg;
            latest_yaw_deg = yaw_deg;
            latest_temperature_c = temperature_c;
            latest_fall = fall;
            latest_motion_valid = 1U;
            __enable_irq();
            (void)snprintf(line,
                           sizeof(line),
                           "flex_x_deg,%.2f,acc_g,%.3f,%.3f,%.3f,gyro_dps,%.2f,%.2f,%.2f,rpy,%.2f,%.2f,%.2f,temp,%.2f,fall,%s\r\n",
                           flex_x_deg,
                           accel.x_g,
                           accel.y_g,
                           accel.z_g,
                           gyro.x_dps,
                           gyro.y_dps,
                           gyro.z_dps,
                           roll_deg,
                           pitch_deg,
                           yaw_deg,
                           temperature_c,
                           (fall != 0U) ? "FALL" : "NORMAL");
#if BMI088_DEBUG_STREAM_ENABLE
            debug_uart_send(line);
#endif
            if (motion_stream_to_esp32 != 0U)
            {
                (void)snprintf(line,
                               sizeof(line),
                               "0.000,0.000,0.000,0.000,%.2f,%.2f,%.2f,%u,%.2f,%.2f,%.2f,%.2f\r\n",
                               gyro.x_dps,
                               gyro.y_dps,
                               gyro.z_dps,
                               (unsigned int)fall,
                               roll_deg,
                               pitch_deg,
                               yaw_deg,
                               temperature_c);
                esp32_uart_send(line);
            }
        }
        else
        {
#if BMI088_DEBUG_STREAM_ENABLE
            debug_uart_send("BMI088_READ_ERROR\r\n");
#endif
        }

        rt_thread_mdelay(BMI088_PRINT_INTERVAL_MS);
    }
}

int bmi088_start(void)
{
    rt_thread_t thread;
    char line[64];
    int status = bmi088_init();

    if (status != 0)
    {
        (void)snprintf(line,
                       sizeof(line),
                       "BMI088_INIT_ERROR,%d,acc=0x%02X,gyro=0x%02X\r\n",
                       status,
                       accel_chip_id,
                       gyro_chip_id);
        debug_uart_send(line);
        return status;
    }

    debug_uart_send("BMI088_INIT_OK,acc=0x1E,gyro=0x0F,acc=6g,gyro=500dps\r\n");
    thread = rt_thread_create("bmi088",
                              bmi088_thread_entry,
                              RT_NULL,
                              1536,
                              18,
                              10);
    if (thread == RT_NULL)
    {
        debug_uart_send("BMI088_THREAD_ERROR\r\n");
        return -10;
    }

    rt_thread_startup(thread);
    return 0;
}
