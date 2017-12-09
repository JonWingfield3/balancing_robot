/*
 * motor_controller.c
 *
 *  Created on: Nov 7, 2017
 *      Author: jonathanwingfield
 */

#include <motor_controller.h>

#include <math.h>
#include <stdbool.h>
#include <stdint.h>

#include <balancing_robot.h>
#include <filters.h>
#include <motor.h>
#include <mpu6050.h>
#include <pid.h>
#include <profiler.h>
#include <scheduler.h>
#include <utils.h>

// point of no return hysteresis filter.
static hysteresis_filter_t ponr_filter_;
static const float PONR_LOW_THRESHOLD = 0.10;
static const float PONR_HIGH_THRESHOLD = (M_PI / 4);
// fairly arbitrary thresholds. Want high threshold such that robot could
// never recover at that pitch, low threshold such that robot is upright
// again after a fall.

static bool motors_enabled_;
static bool ponr_tripped_;

// used for control commands.
static int8_t dc_dir_modifier_;
static uint32_t steer_command_timeout_ticks_ = 200;
static uint32_t drive_command_timeout_ticks_ = 500;

static pid_t pid_;
static const float INITIAL_P_GAIN = 680.0;
static const float INITIAL_I_GAIN = 400.0;
static const float INITIAL_D_GAIN = 10.0;
static const float INITIAL_I_MAX = 0.3;
static const float INITIAL_PID_TARGET = 0.0;
static const float PID_MAX_OUTPUT = 100.0;

static int motor_control_updater(void) {
	const uint32_t sample_time = scheduler_get_system_time_ms();
	const float pitch = mpu6050_get_pitch();

	const hysteresis_state_t previous_ponr_state = ponr_filter_.state;
	hysteresis_filter_add_sample(&ponr_filter_, fabsf(pitch));
	if (ponr_filter_.state != previous_ponr_state) {
		if (ponr_filter_.state == HYSTERESIS_HIGH_STATE) {
			ponr_tripped_ = true;
		} else {
			// back upright after falling over. Reset PID.
			pid_reset(&pid_);
			ponr_tripped_ = false;
		}
	}

	pid_add_measurement(&pid_, pitch, sample_time);
	const int8_t base_duty_cycle = (int8_t) pid_get_output(&pid_);
	const int8_t left_duty_cycle = base_duty_cycle + dc_dir_modifier_;
	const int8_t right_duty_cycle = base_duty_cycle - dc_dir_modifier_;

	if (motors_enabled_ && !ponr_tripped_) {
		motor_set_duty_cycle(LEFT_MOTOR, left_duty_cycle);
		motor_set_duty_cycle(RIGHT_MOTOR, right_duty_cycle);
	} else {
		motor_halt();
	}
	return 0;
}

static int motor_control_steer_command_terminator(void) {
	dc_dir_modifier_ = 0;
	return 1;
}

static int motor_control_drive_command_terminator(void) {
	pid_set_target(&pid_, 0.0);
	return 1;
}

void motor_controller_init(void) {
	mpu6050_init();
	// worked for a long time at p = 680, i = 400, d = 10, fs = 80
	motor_init();
	motors_enabled_ = false;

	pid_init(&pid_, INITIAL_P_GAIN, INITIAL_I_GAIN, INITIAL_D_GAIN, INITIAL_I_MAX,
			PID_MAX_OUTPUT, INITIAL_PID_TARGET);

	hysteresis_filter_init(&ponr_filter_, PONR_LOW_THRESHOLD, PONR_HIGH_THRESHOLD,
			HYSTERESIS_LOW_STATE);
	ponr_tripped_ = false;

	scheduler_init_task(MOTOR_CONTROL_UPDATER, motor_control_updater, 0);
	scheduler_init_task(MOTOR_CONTROL_STEER_COMMAND_TERMINATOR,
			motor_control_steer_command_terminator, 0);
	scheduler_init_task(MOTOR_CONTROL_DRIVE_COMMAND_TERMINATOR,
			motor_control_drive_command_terminator, 0);
}

void motor_controller_enable(bool enable) {
	motors_enabled_ = enable;
	if (!enable) {
		motor_halt();
	} else {
		pid_reset(&pid_);
	}
}

bool motor_controller_enabled(void) {
	return motors_enabled_;
}

void motor_controller_steer_command(int8_t steer_dc_diff) {
	dc_dir_modifier_ = steer_dc_diff;
	scheduler_set_pending_in(MOTOR_CONTROL_STEER_COMMAND_TERMINATOR,
			steer_command_timeout_ticks_);
}

void motor_controller_drive_command(float drive_pitch) {
	pid_set_target(&pid_, drive_pitch);
	scheduler_set_pending_in(MOTOR_CONTROL_DRIVE_COMMAND_TERMINATOR,
			drive_command_timeout_ticks_);
}

void motor_controller_set_command_timeouts(uint16_t steer_command_timeout,
		uint16_t drive_command_timeout) {
	steer_command_timeout_ticks_ = steer_command_timeout;
	drive_command_timeout_ticks_ = drive_command_timeout;
}

float motor_controller_get_pid_target(void) {
	return pid_get_target(&pid_);
}

int8_t motor_controller_get_pid_output(void) {
	return (int8_t) pid_get_output(&pid_);
}

void motor_controller_get_pid_errors(pid_error_t* pid_errors, bool weighted) {
	pid_get_errors(&pid_, pid_errors, weighted);
}

void motor_controller_set_pid_gains(float p_gain, float i_gain, float d_gain) {
	pid_set_gains(&pid_, p_gain, i_gain, d_gain);
}

