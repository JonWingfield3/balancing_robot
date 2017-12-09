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

#include <pid.h>

void motor_controller_init(void);

void motor_controller_enable(bool enable);
bool motor_controller_enabled(void);


void motor_controller_drive_command(float drive_pitch);
void motor_controller_steer_command(int8_t steer_dc_diff);
void motor_controller_set_command_timeouts(uint16_t steer_command_timeout,
		uint16_t drive_command_timeout);

// functions to access/modify motor controller's internal pid
float motor_controller_get_pid_target(void);
int8_t motor_controller_get_pid_output(void);
void motor_controller_get_pid_errors(pid_error_t* pid_errors, bool weighted);
void motor_controller_set_pid_gains(float p_gain, float i_gain, float d_gain);

#endif /* MOTOR_CONTROLLER_H_ */
