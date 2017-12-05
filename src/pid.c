/*
 * pid.c
 *
 *  Created on: Oct 29, 2017
 *      Author: jonathanwingfield
 */

#include <stdlib.h>

#include "../include/pid.h"
#include "../include/utils.h"

void pid_init(pid_t* pid, float p_gain, float i_gain, float d_gain, float i_max,
		float max_output, float target) {
	pid->p_gain = p_gain;
	pid->i_gain = i_gain;
	pid->d_gain = d_gain;
	pid->i_max = i_max;
	pid->max_output = max_output;
	pid->target = target;
}

void pid_set_gains(pid_t* pid, float p_gain, float i_gain, float d_gain) {
	pid->p_gain = p_gain;
	pid->i_gain = i_gain;
	pid->d_gain = d_gain;
}

void pid_set_target(pid_t* pid, float target) {
	pid->target = target;
}

void pid_reset(pid_t* pid) {
	pid->i_error = 0.0;
	pid->de_dt = 0.0;
}

void pid_add_measurement(pid_t* pid, float new_meas, uint32_t new_meas_time) {
	const float time_delta = (new_meas_time - pid->last_meas_time) / 1000.0;

	const float old_error = pid->target - pid->last_meas;
	const float new_error = pid->target - new_meas;
	const float avg_error = (new_error + old_error) / 2.0;

	pid->i_error += (avg_error * time_delta);
	pid->i_error = fclamp(pid->i_error, -pid->i_max, pid->i_max);
	pid->de_dt = (new_error - old_error) / time_delta;

	pid->last_meas = new_meas;
	pid->last_meas_time = new_meas_time;
}

float pid_get_output(pid_t* pid) {
	const float p_term = pid->p_gain * (pid->target - pid->last_meas);
	const float i_term = pid->i_gain * pid->i_error;
	const float d_term = pid->d_gain * pid->de_dt;
	const float pid_output = p_term + i_term + d_term;
	const float clamped_pid_output = fclamp(pid_output, -pid->max_output,
			pid->max_output);
	return clamped_pid_output;
}

void pid_get_errors(pid_t* pid, pid_error_t* pid_errors, bool weighted) {
	if (weighted) {
		pid_errors->p_error = pid->p_gain * (pid->target - pid->last_meas);
		pid_errors->i_error = pid->i_gain * pid->i_error;
		pid_errors->d_error = pid->d_gain * pid->de_dt;
	} else {
		pid_errors->p_error = pid->target - pid->last_meas;
		pid_errors->i_error = pid->i_error;
		pid_errors->d_error = pid->de_dt;
	}
}

float pid_get_target(pid_t* pid) {
	return pid->target;
}
