/*
 * pid.h
 *
 *  Created on: Oct 29, 2017
 *      Author: jonathanwingfield
 */

#ifndef PID_H_
#define PID_H_

#include <stdbool.h>
#include <stdint.h>

typedef struct {
	float p_error;
	float i_error;
	float d_error;
} pid_error_t;

typedef struct {
	float p_gain;
	float i_gain;
	float d_gain;
	float max_output;
	float target;
	float de_dt;
	float i_error;
	float i_max;
	float last_meas;
	uint32_t last_meas_time;
} pid_t;

void pid_init(pid_t* pid, float p_gain, float i_gain, float d_gain, float i_max,
		float max_output, float target);
void pid_add_measurement(pid_t* pid, float new_meas, uint32_t new_meas_time);

// setter methods
void pid_set_target(pid_t* pid, float target);
void pid_set_gains(pid_t* pid, float p_gain, float i_gain, float d_gain);
void pid_reset(pid_t* pid);

// getter methods
float pid_get_output(pid_t* pid);
void pid_get_errors(pid_t* pid, pid_error_t* pid_errors, bool weighted);
float pid_get_target(pid_t* pid);

#endif /* PID_H_ */
