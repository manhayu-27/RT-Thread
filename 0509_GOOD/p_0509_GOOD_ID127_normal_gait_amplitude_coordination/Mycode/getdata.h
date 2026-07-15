#ifndef _GETDATA_H
#define _GETDATA_H

#include "main.h"
#include "can.h"
#include "control.h"

extern volatile uint8_t motor_fb_valid[6];
extern volatile uint32_t motor_fb_tick[6];
extern volatile uint32_t motor_axis_error[6];
extern volatile uint8_t motor_axis_state[6];
extern volatile uint8_t motor_axis_flags[6];
extern volatile uint8_t motor_hb_valid[6];
extern volatile uint32_t motor_hb_tick[6];
extern volatile uint8_t g_ankle_init_step;
extern volatile uint8_t g_ankle_init_result;
extern volatile uint8_t g_ankle_last_tx_status;

void HandleMessage(uint8_t id, const uint8_t *RxData, uint8_t DLC);
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan);

/* ---------------- 127模块(CAN2)数据与调试变量 ---------------- */
typedef struct {
    volatile uint16_t emg;
    volatile int16_t gx;
    volatile int16_t gy;
    volatile int16_t gz;
    volatile int16_t ax;
    volatile int16_t ay;
    volatile int16_t az;
    volatile uint8_t gyro_valid;
    volatile uint8_t accel_valid;
    volatile uint32_t gyro_tick;
    volatile uint32_t accel_tick;
    volatile uint32_t last_tick;
    volatile uint8_t updated;
} Node127Data_t;

extern Node127Data_t g_node127;
extern volatile uint32_t g_can2_rx_count;
extern volatile uint32_t g_can2_127_rx_count;
extern volatile uint32_t g_can2_227_rx_count;
extern volatile uint32_t g_can2_last_id;
extern volatile uint32_t g_can2_error_code;
extern volatile uint32_t g_can2_busoff_count;
extern volatile uint32_t g_node127_sync_count;
extern volatile uint8_t  g_node127_sync_last_status;

extern volatile uint32_t g_can2_tx_ok_count;
extern volatile uint32_t g_can2_tx_fail_count;
extern volatile uint32_t g_can2_tx_busy_count;
extern volatile uint32_t g_can2_tx_abort_count;
extern volatile uint32_t g_can2_tx_mailbox_free;
extern volatile uint32_t g_can2_esr_snapshot;
extern volatile uint32_t g_can2_msr_snapshot;
extern volatile uint32_t g_can2_state_snapshot;
extern volatile uint32_t g_node127_warmup_done;
extern volatile uint32_t g_node127_last_sync_tick;
extern volatile uint32_t g_node127_rx_bus;
extern volatile uint32_t g_can1_127_rx_count;
extern volatile uint32_t g_can1_227_rx_count;

void USER_CAN_PollRx(void);

#endif
