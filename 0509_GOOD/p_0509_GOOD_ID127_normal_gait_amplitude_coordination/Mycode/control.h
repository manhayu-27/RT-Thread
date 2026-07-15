#ifndef _CONTROL_H
#define _CONTROL_H

#include "main.h"
#include "motor.h"

extern struct Motors motor[6];

void motor_stop_init(void);
void motor_init(uint8_t node_id, uint8_t mode);
void Calibrate_motor_init(void);

#endif
