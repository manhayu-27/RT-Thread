#ifndef __MOTOR_CONTROL_H
#define __MOTOR_CONTROL_H

#include <stdint.h>
#include "main.h"
#include "ris_protocol.h"

// --- 新增：定义电机通信类型 ---
typedef enum {
    MOTOR_TYPE_UNITREE_485 = 0, // 宇树电机 (RS-485)
    MOTOR_TYPE_SIGMA_CAN   = 1  // 西格玛电机 (CAN)
} MotorType_t;

#pragma pack(push, 1)

/** 宇树电机专用协议结构体 (保持不变) **/
typedef struct {
    uint8_t head[2];
    RIS_Mode_t mode;
    RIS_Comd_t comd;
    uint16_t CRC16;
} ControlData_t;

typedef struct {
    uint8_t head[2];
    RIS_Mode_t mode;
    RIS_Fbk_t fbk;
    uint16_t CRC16;
} MotorData_t;

#pragma pack(pop)

/** 应用层通用的发送结构体 **/
typedef struct
{
    ControlData_t motor_send_data; // 宇树私有数据包
    MotorType_t motor_type;        // 新增：标识此电机走 485 还是 CAN
    int hex_len;
    unsigned short id;             // 电机ID (485站号 或 CAN标准帧ID)
    unsigned short mode;
    float T;
    float W;
    float Pos;
    float K_P;
    float K_W;
} MOTOR_send;

/** 应用层通用的接收结构体 **/
typedef struct
{
    MotorData_t motor_recv_data;   // 宇树私有回传包
    int hex_len;
    int correct;
    unsigned char motor_id;
    unsigned char mode;
    int Temp;
    unsigned char MError;
    float T;
    float W;
    float Pos;
    float footForce;
} MOTOR_recv;

/* --- 函数原型 --- */

// 宇树专用数据转换
int modify_data(MOTOR_send *motor_s);
int extract_data(MOTOR_recv *motor_r);

// --- 核心：通用的通信调度函数 ---
// 它内部会判断是走 RS485 还是走 CAN
HAL_StatusTypeDef Motor_Comm_Execute(MOTOR_send *pData, MOTOR_recv *rData);

// 宇树 485 底层实现 (原有的)
HAL_StatusTypeDef SERVO_Send_recv(MOTOR_send *pData, MOTOR_recv *rData);

// 西格玛 CAN 底层实现 (新增)
HAL_StatusTypeDef CAN_Sigma_Send_recv(MOTOR_send *pData, MOTOR_recv *rData);

#endif
