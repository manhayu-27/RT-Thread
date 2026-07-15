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
// źţڵ֮ͨ
#define HEARTBEAT 0x001 // ͸
// Axis_Error, Axis_State, Motor_Flag, Encoder_Flag, Controller_Flag, Traj_Done, Life

// ֹͣźţ͸ֹͣв
#define ESTOP 0x002 // ͸

// ʹϢ
#define GET_ERROR 0x003 // ͸
// Error_Type

// SDOService Data Objectݣڵ
#define RX_SDO 0x004 // ͸

// SDOݣȡ
#define TX_SDO 0x005 // ͸

// ڵIDCANϵ豸ʶ
#define SET_AXIS_NODE_ID 0x006 // ͸
// Axis_Node_ID

// ״̬ƵĲͬ״̬
#define SET_AXIS_STATE 0x007 // ͸
// Axis_Requested_State

// ˳ģʽڽǰĿƲ
#define MIT_CONTROL 0x008 // ͸

// ͱֵλúٶ
#define GET_ENCODER_ESTIMATES 0x009 // ͸
// Pos_Estimate, Vel_Estimate

// ͱϢ
#define GET_ENCODER_COUNT 0x00A // ͸
// Shadow_Count, Count_In_Cpr

// ÿģʽѡͬĿƲ
#define SET_CONTROLLER_MODE 0x00B // ͸
// Control_Mode, Input_Mode

// λãڿƵﵽָλ
#define SET_INPUT_POS 0x00C // ͸
// Input_Pos, Vel_FF, Torque_FF

// ٶȣڿƵָٶ
#define SET_INPUT_VEL 0x00D // ͸
// Input_Vel, Torque_FF

// ŤأֱӿƵŤ
#define SET_INPUT_TORQUE 0x00E // ͸
// Input_Torque

// õеƣٶȺ͵
#define SET_LIMITS 0x00F // ͸
// Velocity_Limit, Current_Limit

// ʼ϶̣ڼٵĳݲЧӦ
#define START_ANTICOGGING 0x010 // ͸

// ù켣ٶƣڹ滮˶켣
#define SET_TRAJ_VEL_LIMIT 0x011 // ͸
// Traj_Vel_Limit

// ù켣ٶƣٺͼ
#define SET_TRAJ_ACCEL_LIMITS 0x012 // ͸
// Traj_Accel_Limit, Traj_Decel_Limit

// ù켣ԣڸȷ˶
#define SET_TRAJ_INERTIA 0x013 // ͸
// Traj_Inertia

// IqϢ趨ֵʵʲֵ
#define GET_IQ 0x014 // ͸
// Iq_Setpoint, Iq_Measured

// ȡ޴ֵ޴ģʽ
#define GET_SENSORLESS_ESTIMATES 0x015 // ͸
// Pos_Estimate, Vel_Estimate

// 
#define REBOOT 0x016 // ͸

// ȡߵѹ͵ϢڼصԴ״̬
#define GET_BUS_VOLTAGE_CURRENT 0x017 // ͸
// Bus_Voltage, Bus_Current

// Ĵ״̬
#define CLEAR_ERRORS 0x018 // ͸

// ԼضԿӦ
#define SET_LINEAR_COUNT 0x019 // ͸
// Linear_Count

// λ棬ڵλÿƵӦ
#define SET_POS_GAIN 0x01A // ͸
// Pos_Gain

// ٶ棬ٶȺͻ
#define SET_VEL_GAINS 0x01B // ͸
// Vel_Gain, Vel_Integrator_Gain

// ŤϢ趨ֵʵֵ
#define GET_TORQUES 0x01C // ͸
// Torque_Setpoint, Torque

// ͹Ϣ繦ʺͻе
#define GET_POWERS 0x01D // ͸
// Electrical_Power, Mechanical_Power

// CANͨţж
#define DISABLE_CAN 0x01E // ͸
// 
#define SAVE_CONFIGURATION 0x01F // ͸


#define Motor_calibration 	         {0X04,0x00,0x00,0x00,0x00,0x00,0x00,0x00}  	/*У׼*/
#define Encoder_calibration          {0x07,0x00,0x00,0x00,0x00,0x00,0x00,0x00} 	    /*У׼*/        

#define Filtering_position           {0x03,0x00,0x00,0x00,0x03,0x00,0x00,0x00}        /*˲λÿƣģʽΪλÿƣ0x03ģʽΪλ˲0x03*/
#define Periodic_position            {0x03,0x00,0x00,0x00,0x05,0x00,0x00,0x00}        /*λÿƣģʽΪλÿƣ0x03ģʽΪλ˲0x05*/
#define Direct_velocity_control      {0x02,0x00,0x00,0x00,0x01,0x00,0x00,0x00}        /*ֱٶȿƣģʽΪٶȿƣ0x02ģʽΪֱӿƣ0x01*/
#define Ramp_velocity_control        {0x02,0x00,0x00,0x00,0x02,0x00,0x00,0x00}        /*бٶȿƣģʽΪٶȿƣ0x02ģʽΪٶб£0x02*/
#define Direct_torque_control        {0x01,0x00,0x00,0x00,0x01,0x00,0x00,0x00}        /*ֱؿƣģʽΪؿƣ0x01ģʽΪֱӿƣ0x01*/
#define Ramp_torque_control          {0x01,0x00,0x00,0x00,0x06,0x00,0x00,0x00}        /*бؿƣģʽΪؿƣ0x01ģʽΪб£0x06*/
#define MIT_control_mode             {0x03,0x00,0x00,0x00,0x09,0x00,0x00,0x00}        /*˶ģʽģʽΪλÿƣ0x03ģʽΪ˶ƣ0x09*/

#define Closed_loop_control          {0x08,0x00,0x00,0x00,0x00,0x00,0x00,0x00}        /*ջ*/
#define Close_control   	         {0x01,0x00,0x00,0x00,0x00,0x00,0x00,0x00}        /*رյ*/

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



