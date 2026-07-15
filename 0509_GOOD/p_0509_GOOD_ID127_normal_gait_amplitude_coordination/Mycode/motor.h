#ifndef __MOTOR_H__
#define __MOTOR_H__
#include "main.h"
//#include "./SYSTEM/sys/sys.h"
//#include "./SYSTEM/usart/usart.h"

typedef union {
    uint32_t i;
    float f;
} FloatConverter;

struct Motors {
	float pos;
	float vel;
	float tag_pos;
	float init_pos;
  float Torques;
  float tag_Torques;
};
// CAN 电机协议说明。
#define HEARTBEAT 0x001 // CAN 命令：HEARTBEAT。
// Axis_Error, Axis_State, Motor_Flag, Encoder_Flag, Controller_Flag, Traj_Done, Life

// CAN 电机协议说明。
#define ESTOP 0x002 // CAN 命令：ESTOP。

// CAN 电机协议说明。
#define GET_ERROR 0x003 // CAN 命令：GET_ERROR。
// Error_Type

// CAN 电机协议说明。
#define RX_SDO 0x004 // CAN 命令：RX_SDO。

// CAN 电机协议说明。
#define TX_SDO 0x005 // CAN 命令：TX_SDO。

// CAN 电机协议说明。
#define SET_AXIS_NODE_ID 0x006 // CAN 命令：SET_AXIS_NODE_ID。
// Axis_Node_ID

// CAN 电机协议说明。
#define SET_AXIS_STATE 0x007 // CAN 命令：SET_AXIS_STATE。
// Axis_Requested_State

// CAN 电机协议说明。
#define MIT_CONTROL 0x008 // CAN 命令：MIT_CONTROL。

// CAN 电机协议说明。
#define GET_ENCODER_ESTIMATES 0x009 // CAN 命令：GET_ENCODER_ESTIMATES。
// Pos_Estimate, Vel_Estimate

// CAN 电机协议说明。
#define GET_ENCODER_COUNT 0x00A // CAN 命令：GET_ENCODER_COUNT。
// Shadow_Count, Count_In_Cpr

// CAN 电机协议说明。
#define SET_CONTROLLER_MODE 0x00B // CAN 命令：SET_CONTROLLER_MODE。
// Control_Mode, Input_Mode

// CAN 电机协议说明。
#define SET_INPUT_POS 0x00C // CAN 命令：SET_INPUT_POS。
// Input_Pos, Vel_FF, Torque_FF

// CAN 电机协议说明。
#define SET_INPUT_VEL 0x00D // CAN 命令：SET_INPUT_VEL。
// Input_Vel, Torque_FF

// CAN 电机协议说明。
#define SET_INPUT_TORQUE 0x00E // CAN 命令：SET_INPUT_TORQUE。
// Input_Torque

// CAN 电机协议说明。
#define SET_LIMITS 0x00F // CAN 命令：SET_LIMITS。
// Velocity_Limit, Current_Limit

// CAN 电机协议说明。
#define START_ANTICOGGING 0x010 // CAN 命令：START_ANTICOGGING。

// CAN 电机协议说明。
#define SET_TRAJ_VEL_LIMIT 0x011 // CAN 命令：SET_TRAJ_VEL_LIMIT。
// Traj_Vel_Limit

// CAN 电机协议说明。
#define SET_TRAJ_ACCEL_LIMITS 0x012 // CAN 命令：SET_TRAJ_ACCEL_LIMITS。
// Traj_Accel_Limit, Traj_Decel_Limit

// CAN 电机协议说明。
#define SET_TRAJ_INERTIA 0x013 // CAN 命令：SET_TRAJ_INERTIA。
// Traj_Inertia

// CAN 电机协议说明。
#define GET_IQ 0x014 // CAN 命令：GET_IQ。
// Iq_Setpoint, Iq_Measured

// CAN 电机协议说明。
#define GET_SENSORLESS_ESTIMATES 0x015 // CAN 命令：GET_SENSORLESS_ESTIMATES。
// Pos_Estimate, Vel_Estimate

// 
#define REBOOT 0x016 // CAN 命令：REBOOT。

// CAN 电机协议说明。
#define GET_BUS_VOLTAGE_CURRENT 0x017 // CAN 命令：GET_BUS_VOLTAGE_CURRENT。
// Bus_Voltage, Bus_Current

// CAN 电机协议说明。
#define CLEAR_ERRORS 0x018 // CAN 命令：CLEAR_ERRORS。

// CAN 电机协议说明。
#define SET_LINEAR_COUNT 0x019 // CAN 命令：SET_LINEAR_COUNT。
// Linear_Count

// CAN 电机协议说明。
#define SET_POS_GAIN 0x01A // CAN 命令：SET_POS_GAIN。
// Pos_Gain

// CAN 电机协议说明。
#define SET_VEL_GAINS 0x01B // CAN 命令：SET_VEL_GAINS。
// Vel_Gain, Vel_Integrator_Gain

// CAN 电机协议说明。
#define GET_TORQUES 0x01C // CAN 命令：GET_TORQUES。
// Torque_Setpoint, Torque

// CAN 电机协议说明。
#define GET_POWERS 0x01D // CAN 命令：GET_POWERS。
// Electrical_Power, Mechanical_Power

// CAN 电机协议说明。
#define DISABLE_CAN 0x01E // CAN 命令：DISABLE_CAN。
// 
#define SAVE_CONFIGURATION 0x01F // CAN 命令：SAVE_CONFIGURATION。


#define Motor_calibration 	         {0X04,0x00,0x00,0x00,0x00,0x00,0x00,0x00}  	// CAN 命令：Motor_calibration。
#define Encoder_calibration          {0x07,0x00,0x00,0x00,0x00,0x00,0x00,0x00} 	    // CAN 命令：Encoder_calibration。        

#define Filtering_position           {0x03,0x00,0x00,0x00,0x03,0x00,0x00,0x00}        // CAN 命令：Filtering_position。
#define Periodic_position            {0x03,0x00,0x00,0x00,0x05,0x00,0x00,0x00}        // CAN 命令：Periodic_position。
#define Direct_velocity_control      {0x02,0x00,0x00,0x00,0x01,0x00,0x00,0x00}        // CAN 命令：Direct_velocity_control。
#define Ramp_velocity_control        {0x02,0x00,0x00,0x00,0x02,0x00,0x00,0x00}        // CAN 命令：Ramp_velocity_control。
#define Direct_torque_control        {0x01,0x00,0x00,0x00,0x01,0x00,0x00,0x00}        // CAN 命令：Direct_torque_control。
#define Ramp_torque_control          {0x01,0x00,0x00,0x00,0x06,0x00,0x00,0x00}        // CAN 命令：Ramp_torque_control。
#define MIT_control_mode             {0x03,0x00,0x00,0x00,0x09,0x00,0x00,0x00}        // CAN 命令：MIT_control_mode。

#define Closed_loop_control          {0x08,0x00,0x00,0x00,0x00,0x00,0x00,0x00}        // CAN 命令：Closed_loop_control。
#define Close_control   	         {0x01,0x00,0x00,0x00,0x00,0x00,0x00,0x00}        // CAN 命令：Close_control。

//void motor_init(uint8_t node_id);
void motor_init2(uint8_t node_id);
void motor_init_stop(uint8_t node_id);
void stop(uint8_t node_id);
uint32_t calculate_can_id(uint8_t node_id, uint8_t cmd_id);
void Periodic_location(uint8_t node_id);
void Filtering_location(uint8_t node_id);
void Close_control_mode(uint8_t node_id);
void set_axis_state(uint8_t node_id, uint32_t requested_state);
void set_controller_mode(uint8_t node_id, uint32_t control_mode, uint32_t input_mode);
void clear_errors(uint8_t node_id);
void Direct_Speed(uint8_t node_id);
void Direct_torque(uint8_t node_id);
uint32_t float_to_uint32(float value);
void set_RxSDo(uint8_t node_id, uint8_t op, uint16_t id);
void set_input_pos(uint8_t node_id,float input_pos, int16_t vel_ff, int16_t torque_ff);
void set_input_vel(uint8_t node_id, float input_vel, float torque_ff);
void set_input_torque(uint8_t node_id, float input_torque);
float intToFloat(uint32_t intValue);

void set_limits(uint8_t node_id, float velocity_limit, float current_limit);
void set_linear_count(uint8_t node_id, int32_t linear_count);
uint8_t can_set_axis_state(uint8_t node_id, uint32_t requested_state);
uint8_t can_set_controller_mode(uint8_t node_id, uint32_t control_mode, uint32_t input_mode);
uint8_t can_set_input_pos(uint8_t node_id, float input_pos, int16_t vel_ff, int16_t torque_ff);
uint8_t can_set_pos_gain(uint8_t node_id, float pos_gain);
uint8_t can_set_vel_gains(uint8_t node_id, float vel_gain, float vel_integrator_gain);
uint8_t can_set_limits(uint8_t node_id, float velocity_limit, float current_limit);
uint8_t can_clear_errors(uint8_t node_id);
#endif



