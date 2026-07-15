#include "motor.h"
#include "can_driver.h"
#include "motor.h"
#include "can_driver.h"

/**
 * @brief       CAN_ID
 * @param       node_idڵid
 * @param       cmd_id
 * @retval      can_id
 */
//void motor_init(uint8_t node_id)
//{
//	Filtering_location(node_id);
	
//}
void motor_init2(uint8_t node_id)
{
	Direct_torque(node_id);
	
}
void motor_init_stop(uint8_t node_id)
{
	Close_control_mode(node_id);
}
/**
 * CANController Area Networkʶ
 * CANʶCANΨһʶÿϢĴ
 * ͨϽڵIDIDʶ
 *
 * @param node_id ڵIDʾϢķ߻
 * @param cmd_id IDʾϢͻ
 * @return ؼCANʶ
 */
uint32_t calculate_can_id(uint8_t node_id, uint8_t cmd_id)
{
// ȷnode_idֻռ6λ
//	node_id &= 0x3F; // 0x3F == 0011 1111
// node_id5λΪcmd_idλ
	return ((uint32_t)node_id << 5) | cmd_id;
}
/**
 * ȸתΪ32λ޷
 *
 * ˺ͨ壨unionķʽʵתַƹĳЩֱתơ
 *  float_to_int ͬʱӵһԱ f һ޷Ա iǹͬһڴ档
 *  f ֱֵͨ޷ i ȡڴĶݣӶʵת
 *
 * ע⣺תضϵͳͱܲпֲԡ
 *
 * @param value ȸƱʾתΪ32λ޷
 * @return 32λ޷븡ĶƱʾ
 */
uint32_t float_to_uint32(float value)
{
	// һ壬ڸת
	union
	{
		float f;
		uint32_t i;
	} float_to_int;

	// ĸֵĸԱ
	float_to_int.f = value;
	// ع޷ԱĶƱʾ
	return float_to_int.i;
}

/**
 * 32λ޷תΪ
 *
 * ˺һ32λ޷uint32_tΪ룬ͣ reinterpret castΪһfloat
 * תͨ壨unionʵֵģһڴռӵвͬ
 * ʹֱַڴ漶Ͻ½ΪҪֱӷʸڲʾº
 *
 * @param intValue һ32λ޷תΪ
 * @return תĸֵ
 */
float intToFloat(uint32_t intValue)
{
	// һͨһԱڴ棬ͬʱЩڴ汻Ա
	FloatConverter converter;
	// ֵ
	converter.i = intValue;
	// عĸ֣⽫ֱӷԭʼڴеĸ
	return converter.f;
}

static void pack_u32_le(uint8_t *data, uint32_t value)
{
    data[0] = (uint8_t)((value >> 0) & 0xFFU);
    data[1] = (uint8_t)((value >> 8) & 0xFFU);
    data[2] = (uint8_t)((value >> 16) & 0xFFU);
    data[3] = (uint8_t)((value >> 24) & 0xFFU);
}

static void pack_float_le(uint8_t *data, float value)
{
    pack_u32_le(data, float_to_uint32(value));
}

static void pack_i32_le(uint8_t *data, int32_t value)
{
    pack_u32_le(data, (uint32_t)value);
}

uint8_t can_set_axis_state(uint8_t node_id, uint32_t requested_state)
{
    uint8_t data[8] = {0};
    pack_u32_le(data, requested_state);
    return can1_send_msg(calculate_can_id(node_id, SET_AXIS_STATE), data, 8U);
}

uint8_t can_set_controller_mode(uint8_t node_id, uint32_t control_mode, uint32_t input_mode)
{
    uint8_t data[8] = {0};
    pack_u32_le(&data[0], control_mode);
    pack_u32_le(&data[4], input_mode);
    return can1_send_msg(calculate_can_id(node_id, SET_CONTROLLER_MODE), data, 8U);
}

uint8_t can_clear_errors(uint8_t node_id)
{
    uint8_t data[8] = {0};
    return can1_send_msg(calculate_can_id(node_id, CLEAR_ERRORS), data, 8U);
}

uint8_t can_set_limits(uint8_t node_id, float velocity_limit, float current_limit)
{
    uint8_t data[8] = {0};
    pack_float_le(&data[0], velocity_limit);
    pack_float_le(&data[4], current_limit);
    return can1_send_msg(calculate_can_id(node_id, SET_LIMITS), data, 8U);
}

uint8_t can_set_pos_gain(uint8_t node_id, float pos_gain)
{
    uint8_t data[8] = {0};
    pack_float_le(&data[0], pos_gain);
    return can1_send_msg(calculate_can_id(node_id, SET_POS_GAIN), data, 8U);
}

uint8_t can_set_vel_gains(uint8_t node_id, float vel_gain, float vel_integrator_gain)
{
    uint8_t data[8] = {0};
    pack_float_le(&data[0], vel_gain);
    pack_float_le(&data[4], vel_integrator_gain);
    return can1_send_msg(calculate_can_id(node_id, SET_VEL_GAINS), data, 8U);
}

uint8_t can_set_input_pos(uint8_t node_id, float input_pos, int16_t vel_ff, int16_t torque_ff)
{
    uint8_t data[8] = {0};
    uint32_t pos = float_to_uint32(input_pos);
    uint16_t vel_ff_fixed = (uint16_t)vel_ff;
    uint16_t torque_ff_fixed = (uint16_t)torque_ff;

    data[0] = (uint8_t)((pos >> 0) & 0xFFU);
    data[1] = (uint8_t)((pos >> 8) & 0xFFU);
    data[2] = (uint8_t)((pos >> 16) & 0xFFU);
    data[3] = (uint8_t)((pos >> 24) & 0xFFU);
    data[4] = (uint8_t)((vel_ff_fixed >> 0) & 0xFFU);
    data[5] = (uint8_t)((vel_ff_fixed >> 8) & 0xFFU);
    data[6] = (uint8_t)((torque_ff_fixed >> 0) & 0xFFU);
    data[7] = (uint8_t)((torque_ff_fixed >> 8) & 0xFFU);

    return can1_send_msg(calculate_can_id(node_id, SET_INPUT_POS), data, 8U);
}

void set_limits(uint8_t node_id, float velocity_limit, float current_limit)
{
    (void)can_set_limits(node_id, velocity_limit, current_limit);
}

void set_linear_count(uint8_t node_id, int32_t linear_count)
{
    uint8_t data[8] = {0};
    pack_i32_le(data, linear_count);
    (void)can1_send_msg(calculate_can_id(node_id, SET_LINEAR_COUNT), data, 8U);
}

void set_axis_state(uint8_t node_id, uint32_t requested_state)
{
    (void)can_set_axis_state(node_id, requested_state);
}

void set_controller_mode(uint8_t node_id, uint32_t control_mode, uint32_t input_mode)
{
    (void)can_set_controller_mode(node_id, control_mode, input_mode);
}

void clear_errors(uint8_t node_id)
{
    (void)can_clear_errors(node_id);
}

/**
 *
 * úȼģʽCAN IDȻͿģʽ
 * ż״̬ƵCAN ID״̬ʵλÿ
 *
 * @param node_id ڵIDָĽڵ
 */
void Periodic_location(uint8_t node_id)
{
	// ģʽCAN ID
	uint16_t can_id = calculate_can_id(node_id, SET_CONTROLLER_MODE); // Set_Controller_Mode

	// ׼Ϳģʽ
	uint8_t Set_Controller_Mode[8] = Periodic_position;
	can1_send_msg(can_id, Set_Controller_Mode, 8);
	// ״̬ƵCAN ID
	uint16_t can_id1 = calculate_can_id(node_id, SET_AXIS_STATE); // state_control
	// ׼״̬άֱջ
	uint8_t closed_control[8] = Closed_loop_control;
	can1_send_msg(can_id1, closed_control, 8);
	//printf("hello,Periodic_location");
}

/**
 * @brief ݽڵIDλÿָ
 *
 * ݽڵIDӦCAN IDջָͨCAN߷ͳȥ
 * Ҫʵֶضڵ˲λÿơ
 *
 * @param node_id ڵIDڼΨһCAN ID
 */
void Filtering_location(uint8_t node_id)
{
	uint16_t can_id = calculate_can_id(node_id, SET_CONTROLLER_MODE); // Set_Controller_Mode

	// ׼Ϳģʽ
	uint8_t Set_Controller_Mode[8] = Filtering_position;
	can1_send_msg(can_id, Set_Controller_Mode, 8);
	// ״̬ƵCAN ID
	uint16_t can_id1 = calculate_can_id(node_id, SET_AXIS_STATE); // state_control
	// ׼״̬άֱջ
	uint8_t closed_control[8] = Closed_loop_control;
	can1_send_msg(can_id1, closed_control, 8);// ѾΪ˲λÿģʽ,ҪãֱӱջС
	//HAL_CAN_AddTxMessage(&hcan, &tx_header, tx_data, &tx_mailbox);
	
  //printf("hello,Filtering_location");
  

//uint16_t can_id = calculate_can_id(node_id, 0x01F);

//	// ׼Ϳģʽ
//	uint8_t Set_Controller_Mode[8] ={0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00};
//	can1_send_msg(can_id, Set_Controller_Mode, 8);
}

void Close_control_mode(uint8_t node_id)
{
    /* 进入空闲状态。原代码把 Close_control 数据误发到 Set_Controller_Mode，
     * 随后又发送闭环控制，可能导致电机没有真正停机。
     */
    set_axis_state(node_id, 1U);
}




// * @brief ģʽ
// *
// * ݽڵIDӦCAN IDջָͨCAN߷ͳȥ
// * Ҫʵֶضڵֱؿơ
// *  @param node_id ڵIDڼΨһCAN ID
// */
void Direct_torque(uint8_t node_id)
{
	// ģʽCAN ID
	uint16_t can_id = calculate_can_id(node_id, SET_CONTROLLER_MODE); // Set_Controller_Mode
	// ׼Ϳģʽ
	uint8_t Set_Controller_Mode[8] = Direct_torque_control;
	can1_send_msg(can_id, Set_Controller_Mode, 8);
	// ״̬ƵCAN ID
	uint16_t can_id1 = calculate_can_id(node_id, SET_AXIS_STATE); // state_control
	// ׼״̬άֱջ
	uint8_t closed_control[8] = Closed_loop_control;
	can1_send_msg(can_id1, closed_control, 8);
}

 /* @brief ٶģʽ
 *
 * ݽڵIDӦCAN IDջָͨCAN߷ͳȥ
 * Ҫʵֶضڵֱٶȿơ
 *  @param node_id ڵIDڼΨһCAN ID
 */
void Direct_Speed(uint8_t node_id)
{
	// ģʽCAN ID
	uint16_t can_id = calculate_can_id(node_id, SET_CONTROLLER_MODE); // Set_Controller_Mode
	// ׼Ϳģʽ
	uint8_t Set_Controller_Mode[8] = Direct_velocity_control;
	can1_send_msg(can_id, Set_Controller_Mode, 8);
	// ״̬ƵCAN ID
	uint16_t can_id1 = calculate_can_id(node_id, SET_AXIS_STATE); // state_control
	// ׼״̬άֱջ
	
	uint8_t closed_control[8] = Closed_loop_control;
	can1_send_msg(can_id1, closed_control, 8);
}


/**
 * ýڵĿλ
 *
 * úͨCANָڵ㷢ͿĿλԼٶȺصǰֵ
 *
 * @param node_id ڵIDڱʶеľڵ
 * @param input_pos Ŀλãʾ
 * @param vel_ff ٶǰڸƶ̬Ӧ
 * @param torque_ff ǰڲⲿŶ߿ƾ
 */
void set_input_pos(uint8_t node_id, float input_pos, int16_t vel_ff, int16_t torque_ff)
{
	// һ8ֽڵ飬ڴ洢λáٶȺصϢ
	uint8_t data[8] = {0};
  
  	//޷
//	float limit = 4.0;
//	if(input_pos >  limit)	input_pos =  limit;
//	if(input_pos < -limit)	input_pos = -limit;
  
	// λתΪIEEE 754ʽ32λ洢ǰ4ֽ
	uint32_t pos = float_to_uint32(input_pos);
	data[0] = (pos >> 0) & 0xFF;
	data[1] = (pos >> 8) & 0xFF;
	data[2] = (pos >> 16) & 0xFF;
	data[3] = (pos >> 24) & 0xFF;

	// ٶǰ洢ڵ4͵5ֽڣС˸ʽ
	uint16_t vel_ff_fixed = (uint16_t)vel_ff;
	data[4] = (vel_ff_fixed >> 0) & 0xFF;
	data[5] = (vel_ff_fixed >> 8) & 0xFF;

	// ǰ洢ڵ6͵7ֽڣС˸ʽ
	uint16_t torque_ff_fixed = (uint16_t)torque_ff;
	data[6] = (torque_ff_fixed >> 0) & 0xFF;
	data[7] = (torque_ff_fixed >> 8) & 0xFF;

	// CANϢIDÿģʽ
	uint16_t can_id = calculate_can_id(node_id, SET_INPUT_POS); // Set_Controller_Mode

	// CANϢڵIDݳȺ
	can1_send_msg(can_id, data, 8);
}

void set_input_torque(uint8_t node_id, float input_torque)
{
	// һ8ֽڵ飬ڴ洢λáٶȺصϢ
	uint8_t data[8] = {0};
  
  	//޷
//	float limit = 4.0;
//	if(input_pos >  limit)	input_pos =  limit;
//	if(input_pos < -limit)	input_pos = -limit;
  
	// λתΪIEEE 754ʽ32λ洢ǰ4ֽ
//	uint32_t torque = float_to_uint32(input_torque*0.12f);
  	uint32_t torque = float_to_uint32(input_torque*1.0f);
	data[0] = (torque >> 0) & 0xFF;
	data[1] = (torque >> 8) & 0xFF;
	data[2] = (torque >> 16) & 0xFF;
	data[3] = (torque >> 24) & 0xFF;

	// CANϢIDÿģʽ
	uint16_t can_id = calculate_can_id(node_id, SET_INPUT_TORQUE); // Set_Controller_Mode

	// CANϢڵIDݳȺ
	can1_send_msg(can_id, data, 8);
}


void set_input_vel(uint8_t node_id, float input_vel, float torque_ff)
{
    uint8_t data[8] = {0};

    pack_float_le(&data[0], input_vel);
    pack_float_le(&data[4], torque_ff);

    (void)can1_send_msg(calculate_can_id(node_id, SET_INPUT_VEL), data, 8U);
}


void stop(uint8_t node_id)
{
	uint8_t data[8] = {0};
	uint16_t can_id = calculate_can_id(node_id, ESTOP); 
	can1_send_msg(can_id, data, 8); 
}







///**
// * @brief       CAN_ID
// * @param       node_idڵid
// * @param       cmd_id
// * @retval      can_id
// */
//void motor_init(uint8_t node_id)
//{
//	Filtering_location(node_id);
//	
//}

//void motor_init_stop(uint8_t node_id)
//{
//	Close_control_mode(node_id);
//}
///**
// * CANController Area Networkʶ
// * CANʶCANΨһʶÿϢĴ
// * ͨϽڵIDIDʶ
// *
// * @param node_id ڵIDʾϢķ߻
// * @param cmd_id IDʾϢͻ
// * @return ؼCANʶ
// */
//uint32_t calculate_can_id(uint8_t node_id, uint8_t cmd_id)
//{
//// ȷnode_idֻռ6λ
////	node_id &= 0x3F; // 0x3F == 0011 1111
//// node_id5λΪcmd_idλ
//	return ((uint32_t)node_id << 5) | cmd_id;
//}
///**
// * ȸתΪ32λ޷
// *
// * ˺ͨ壨unionķʽʵתַƹĳЩֱתơ
// *  float_to_int ͬʱӵһԱ f һ޷Ա iǹͬһڴ档
// *  f ֱֵͨ޷ i ȡڴĶݣӶʵת
// *
// * ע⣺תضϵͳͱܲпֲԡ
// *
// * @param value ȸƱʾתΪ32λ޷
// * @return 32λ޷븡ĶƱʾ
// */
//uint32_t float_to_uint32(float value)
//{
//	// һ壬ڸת
//	union
//	{
//		float f;
//		uint32_t i;
//	} float_to_int;

//	// ĸֵĸԱ
//	float_to_int.f = value;
//	// ع޷ԱĶƱʾ
//	return float_to_int.i;
//}

///**
// * 32λ޷תΪ
// *
// * ˺һ32λ޷uint32_tΪ룬ͣ reinterpret castΪһfloat
// * תͨ壨unionʵֵģһڴռӵвͬ
// * ʹֱַڴ漶Ͻ½ΪҪֱӷʸڲʾº
// *
// * @param intValue һ32λ޷תΪ
// * @return תĸֵ
// */
//float intToFloat(uint32_t intValue)
//{
//	// һͨһԱڴ棬ͬʱЩڴ汻Ա
//	FloatConverter converter;
//	// ֵ
//	converter.i = intValue;
//	// عĸ֣⽫ֱӷԭʼڴеĸ
//	return converter.f;
//}

///**
// *
// * úȼģʽCAN IDȻͿģʽ
// * ż״̬ƵCAN ID״̬ʵλÿ
// *
// * @param node_id ڵIDָĽڵ
// */
//void Periodic_location(uint8_t node_id)
//{
//	// ģʽCAN ID
//	uint16_t can_id = calculate_can_id(node_id, SET_CONTROLLER_MODE); // Set_Controller_Mode

//	// ׼Ϳģʽ
//	uint8_t Set_Controller_Mode[8] = Periodic_position;
//	can1_send_msg(can_id, Set_Controller_Mode, 8);
//	// ״̬ƵCAN ID
//	uint16_t can_id1 = calculate_can_id(node_id, SET_AXIS_STATE); // state_control
//	// ׼״̬άֱջ
//	uint8_t closed_control[8] = Closed_loop_control;
//	can1_send_msg(can_id1, closed_control, 8);
//	printf("hello,Periodic_location");
//}

///**
// * @brief ݽڵIDλÿָ
// *
// * ݽڵIDӦCAN IDջָͨCAN߷ͳȥ
// * Ҫʵֶضڵ˲λÿơ
// *
// * @param node_id ڵIDڼΨһCAN ID
// */
//void Filtering_location(uint8_t node_id)
//{
//	uint16_t can_id = calculate_can_id(node_id, SET_CONTROLLER_MODE); // Set_Controller_Mode

//	// ׼Ϳģʽ
//	uint8_t Set_Controller_Mode[8] = Filtering_position;
//	can1_send_msg(can_id, Set_Controller_Mode, 8);
//	// ״̬ƵCAN ID
//	uint16_t can_id1 = calculate_can_id(node_id, SET_AXIS_STATE); // state_control
//	// ׼״̬άֱջ
//	uint8_t closed_control[8] = Closed_loop_control;
//	can1_send_msg(can_id1, closed_control, 8);// ѾΪ˲λÿģʽ,ҪãֱӱջС
////	HAL_CAN_AddTxMessage(&hcan, &tx_header, tx_data, &tx_mailbox);
//	
//  printf("hello,Filtering_location");
//}
///**
// * @brief ٶģʽ
// *
// * ݽڵIDӦCAN IDջָͨCAN߷ͳȥ
// * Ҫʵֶضڵֱٶȿơ
// *  @param node_id ڵIDڼΨһCAN ID
// */
//void Direct_Speed(uint8_t node_id)
//{
//	// ģʽCAN ID
//	uint16_t can_id = calculate_can_id(node_id, SET_CONTROLLER_MODE); // Set_Controller_Mode
//	// ׼Ϳģʽ
//	uint8_t Set_Controller_Mode[8] = Direct_velocity_control;
//	can1_send_msg(can_id, Set_Controller_Mode, 8);
//	// ״̬ƵCAN ID
//	uint16_t can_id1 = calculate_can_id(node_id, SET_AXIS_STATE); // state_control
//	// ׼״̬άֱջ
//	
//	uint8_t closed_control[8] = Closed_loop_control;
//	can1_send_msg(can_id1, closed_control, 8);
//}
///**
// * @brief ģʽ
// *
// * ݽڵIDӦCAN IDջָͨCAN߷ͳȥ
// * Ҫʵֶضڵֱؿơ
// *  @param node_id ڵIDڼΨһCAN ID
// */
//void Direct_torque(uint8_t node_id)
//{
//	// ģʽCAN ID
//	uint16_t can_id = calculate_can_id(node_id, SET_CONTROLLER_MODE); // Set_Controller_Mode
//	// ׼Ϳģʽ
//	uint8_t Set_Controller_Mode[8] = Direct_torque_control;
//	can1_send_msg(can_id, Set_Controller_Mode, 8);
//	// ״̬ƵCAN ID
//	uint16_t can_id1 = calculate_can_id(node_id, SET_AXIS_STATE); // state_control
//	// ׼״̬άֱջ
//	uint8_t closed_control[8] = Closed_loop_control;
//	can1_send_msg(can_id1, closed_control, 8);
//}

//void Close_control_mode(uint8_t node_id)
//{
//	uint16_t can_id = calculate_can_id(node_id, SET_CONTROLLER_MODE); // Set_Controller_Mode

//	// ׼Ϳģʽ
//	uint8_t Set_Controller_Mode[8] = Close_control;
//	can1_send_msg(can_id, Set_Controller_Mode, 8);
//	// ״̬ƵCAN ID
//	uint16_t can_id1 = calculate_can_id(node_id, SET_AXIS_STATE); // state_control
//	// ׼״̬άֱջ
//	uint8_t closed_control[8] = Closed_loop_control;
//	can1_send_msg(can_id1, closed_control, 8);// ѾΪ˲λÿģʽ,ҪãֱӱջС
////	HAL_CAN_AddTxMessage(&hcan, &tx_header, tx_data, &tx_mailbox);
//	
//  printf("hello,Close_control");
//}


///**
// * ýڵĿλ
// *
// * úͨCANָڵ㷢ͿĿλԼٶȺصǰֵ
// *
// * @param node_id ڵIDڱʶеľڵ
// * @param input_pos Ŀλãʾ
// * @param vel_ff ٶǰڸƶ̬Ӧ
// * @param torque_ff ǰڲⲿŶ߿ƾ
// */
//void set_input_pos(uint8_t node_id, float input_pos, int16_t vel_ff, int16_t torque_ff)
//{
//	// һ8ֽڵ飬ڴ洢λáٶȺصϢ
//	uint8_t data[8];

//	// λתΪIEEE 754ʽ32λ洢ǰ4ֽ
//	uint32_t pos = float_to_uint32(input_pos);
//	data[0] = (pos >> 0) & 0xFF;
//	data[1] = (pos >> 8) & 0xFF;
//	data[2] = (pos >> 16) & 0xFF;
//	data[3] = (pos >> 24) & 0xFF;

//	// ٶǰ洢ڵ4͵5ֽڣС˸ʽ
//	uint16_t vel_ff_fixed = (uint16_t)vel_ff;
//	data[4] = (vel_ff_fixed >> 0) & 0xFF;
//	data[5] = (vel_ff_fixed >> 8) & 0xFF;

//	// ǰ洢ڵ6͵7ֽڣС˸ʽ
//	uint16_t torque_ff_fixed = (uint16_t)torque_ff;
//	data[6] = (torque_ff_fixed >> 0) & 0xFF;
//	data[7] = (torque_ff_fixed >> 8) & 0xFF;

//	// CANϢIDÿģʽ
//	uint16_t can_id = calculate_can_id(node_id, SET_INPUT_POS); // Set_Controller_Mode

//	// CANϢڵIDݳȺ
//	can1_send_msg(can_id, data, 8);
//}

////void set_foot_massage(uint8_t node_id, float input_pos, int16_t vel_ff, int16_t torque_ff)
////{
////	// һ8ֽڵ飬ڴ洢λáٶȺصϢ
////	uint8_t data[8];

////	// λתΪIEEE 754ʽ32λ洢ǰ4ֽ
////	uint32_t pos = float_to_uint32(input_pos);
////	data[0] = (pos >> 0) & 0xFF;
////	data[1] = (pos >> 8) & 0xFF;
////	data[2] = (pos >> 16) & 0xFF;
////	data[3] = (pos >> 24) & 0xFF;

////	// ٶǰ洢ڵ4͵5ֽڣС˸ʽ
////	uint16_t vel_ff_fixed = (uint16_t)vel_ff;
////	data[4] = (vel_ff_fixed >> 0) & 0xFF;
////	data[5] = (vel_ff_fixed >> 8) & 0xFF;

////	// ǰ洢ڵ6͵7ֽڣС˸ʽ
////	uint16_t torque_ff_fixed = (uint16_t)torque_ff;
////	data[6] = (torque_ff_fixed >> 0) & 0xFF;
////	data[7] = (torque_ff_fixed >> 8) & 0xFF;

////	// CANϢIDÿģʽ
////	uint16_t can_id = calculate_can_id(node_id, SET_INPUT_POS); // Set_Controller_Mode

////	// CANϢڵIDݳȺ
////	can_send_msg(can_id, data, 8);
////}


//void set_RxSDo(uint8_t node_id, uint8_t op, uint16_t id)
//{
//	// һ8ֽڵ飬ڴ洢λáٶȺصϢ
//	uint8_t data[8];

//	// λתΪIEEE 754ʽ32λ洢ǰ4ֽ
//	data[0] = op;
//	data[1] = id & 0xFF;
//	data[2] = (id >> 8) & 0xFF;
//	data[3] = 0;
//	data[4] = 0;
//	data[5] = 0;
//	data[6] = 0;
//	data[7] = 0;

//	// CANϢIDÿģʽ
//	uint16_t can_id = calculate_can_id(node_id, GET_ENCODER_ESTIMATES); // Set_Controller_Mode

//	// CANϢڵIDݳȺ
//	can1_send_msg(can_id, data, 8);
//}

///**
// * ýڵĿٶ
// *
// * úͨCANָڵ㷢ͿĿٶԼǰֵٶȿģʽ£
// *
// * @param node_id ڵIDڱʶеľڵ
// * @param input_vel Ŀٶȣʾ
// * @param torque_ff ǰڲⲿŶ߿ƾ
// */
//void set_input_vel(uint8_t node_id, float input_vel, int16_t torque_ff)
//{
//    // һ8ֽڵ飬ڴ洢ٶȺصϢ
//    uint8_t data[8];

//    // ٶתΪIEEE 754ʽ32λ洢ǰ4ֽ
//    uint32_t vel = float_to_uint32(input_vel);
//    data[0] = (vel >> 0) & 0xFF;
//    data[1] = (vel >> 8) & 0xFF;
//    data[2] = (vel >> 16) & 0xFF;
//    data[3] = (vel >> 24) & 0xFF;

//    // ǰ洢ڵ6͵7ֽڣС˸ʽ
//    uint16_t torque_ff_fixed = (uint16_t)torque_ff;
//    data[6] = (torque_ff_fixed >> 0) & 0xFF;
//    data[7] = (torque_ff_fixed >> 8) & 0xFF;

//    // CANϢIDٶȿģʽµٶָ
//    uint16_t can_id = calculate_can_id(node_id, SET_INPUT_VEL); 

//    // CANϢڵIDݳȺ
//    can1_send_msg(can_id, data, 8);
//}
///**
// * ýڵĿ
// *
// * úͨCANָڵ㷢ͿĿ
// *
// * @param node_id ڵIDڱʶеľڵ
// * @param input_torque Ŀأʾ
// */
//void set_input_torque(uint8_t node_id, float input_torque)
//{
//    // һ8ֽڵ飬ڴ洢Ϣʵֻõǰ4ֽڣ
//    uint8_t data[8];

//    // תΪIEEE 754ʽ32λ洢ǰ4ֽ
//    uint32_t torque = float_to_uint32(input_torque);
//    data[0] = (torque >> 0) & 0xFF;
//    data[1] = (torque >> 8) & 0xFF;
//    data[2] = (torque >> 16) & 0xFF;
//    data[3] = (torque >> 24) & 0xFF;

//    // CANϢIDؿָ
//    uint16_t can_id = calculate_can_id(node_id, SET_INPUT_TORQUE); 

//    // CANϢڵIDݳȺ
//    can1_send_msg(can_id, data, 4); 
//    // עݳΪ4Ϊֻռǰ4ֽ
//}

//void stop(uint8_t node_id)
//{
//	uint8_t data[8];
//	uint16_t can_id = calculate_can_id(node_id, ESTOP); 
//	can1_send_msg(can_id, data, 8); 
//}
