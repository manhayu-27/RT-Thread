#ifndef __ANKLE_CAN_CONFIG_H__
#define __ANKLE_CAN_CONFIG_H__

#include <stdint.h>

/* Only one CAN ankle motor is used on the CAN bus. */
#define ANKLE_CAN_NODE_ID 1U

/*
 * Set to 1 for the safest first power-on test.
 * 1: after CAN init the ankle motor only holds its current position.
 * 0: run the ankle gait curve after the initial hold period.
 */
#define ANKLE_HOLD_ONLY_TEST 0U

/* Runtime zeroing: set encoder linear count to 0 at the current physical pose. */
#define ANKLE_ZERO_BY_LINEAR_COUNT 0U

/* Position-control input mode. 1 = direct position input, 3 = filtered position input.
 * For a continuously refreshed gait curve, use filtered position input.
 */
#define ANKLE_CAN_CONTROL_MODE      3U
#define ANKLE_CAN_INPUT_MODE        3U

/* Limits used during debug. Increase only after closed-loop hold is confirmed. */
#define ANKLE_SAFE_VEL_LIMIT_TURN_S 4.0f
#define ANKLE_SAFE_CURRENT_LIMIT_A  5.0f

/* Position-loop gains. If the ankle oscillates, reduce these first. */
#define ANKLE_POS_GAIN              8.0f
#define ANKLE_VEL_GAIN              0.050f
#define ANKLE_VEL_INTEGRATOR_GAIN   0.050f

/* Keep the ankle holding the zero pose briefly before any gait command. */
#define ANKLE_PRE_GAIT_HOLD_MS 300U

/* If closed loop is not confirmed, retry the enable sequence instead of going idle. */
#define ANKLE_ENABLE_RETRY_MS        300U
#define ANKLE_ENABLE_ATTEMPTS        20U

/* SG6010C reducer ratio. CAN Set_Input_Pos uses position in turns, so the
 * commanded output-shaft angle must be converted to motor/encoder turns.
 * If your drive has already been configured to report output-shaft turns,
 * change this value to 1.0f.
 */
#define ANKLE_OUTPUT_REDUCTION_RATIO 9.67f

#endif
