/*
 * motor_controller.h
 *
 *  Created on: Nov 7, 2017
 *      Author: jonathanwingfield
 */

#ifndef MOTOR_CONTROLLER_H_
#define MOTOR_CONTROLLER_H_

#include <stdbool.h>
#include <stdint.h>

#include "pid.h"

typedef enum {
	FORWARD_COMMAND,
	BACKWARD_COMMAND,
	TURN_LEFT_COMMAND,
	TURN_RIGHT_COMMAND,
	NUM_COMMANDS,
} motor_controller_command_t;

typedef enum {
	SLOW_SPEED, MEDIUM_SPEED, FAST_SPEED, NUM_SPEEDS,
} motor_controller_speed_t;

void motor_controller_init(void);

void motor_controller_enable(bool enable);

void motor_controller_command(motor_controller_command_t cmd,
		motor_controller_speed_t speed);

// functions to access/modify motor controller's internal pid
float motor_controller_get_pid_target(void);
int8_t motor_controller_get_pid_output(void);
void motor_controller_get_pid_errors(pid_error_t* pid_errors, bool weighted);
void motor_controller_set_pid_target(float target);
void motor_controller_set_pid_gains(float p_gain, float i_gain, float d_gain);

#endif /* MOTOR_CONTROLLER_H_ */
