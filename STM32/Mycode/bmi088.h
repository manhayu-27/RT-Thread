#ifndef __BMI088_H__
#define __BMI088_H__

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef struct
{
    float x_dps;
    float y_dps;
    float z_dps;
} bmi088_gyro_data_t;

typedef struct
{
    float x_g;
    float y_g;
    float z_g;
} bmi088_accel_data_t;

typedef struct
{
    bmi088_gyro_data_t gyro;
    float roll_deg;
    float pitch_deg;
    float yaw_deg;
    float temperature_c;
} bmi088_motion_data_t;

int bmi088_init(void);
int bmi088_read_gyro(bmi088_gyro_data_t *data);
int bmi088_read_accel(bmi088_accel_data_t *data);
int bmi088_read_temperature(float *temperature_c);
uint8_t bmi088_get_latest_motion(bmi088_gyro_data_t *gyro, uint8_t *fall);
uint8_t bmi088_get_latest_state(bmi088_motion_data_t *motion, uint8_t *fall);
void bmi088_set_motion_stream_to_esp32(uint8_t enabled);
float bmi088_calc_flex_x_deg(const bmi088_accel_data_t *accel);
uint8_t bmi088_detect_fall(const bmi088_accel_data_t *accel,
                           float flex_x_deg);
int bmi088_start(void);

#ifdef __cplusplus
}
#endif

#endif /* __BMI088_H__ */
