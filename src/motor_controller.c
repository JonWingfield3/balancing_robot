/*
 * motor_controller.c
 *
 *  Created on: Nov 7, 2017
 *      Author: jonathanwingfield
 */

#include "../include/motor_controller.h"

#include <math.h>
#include <stdbool.h>
#include <stdint.h>

#include "../include/balancing_robot.h"
#include "../include/filters.h"
#include "../include/motor.h"
#include "../include/mpu6050.h"
#include "../include/scheduler.h"
#include "../include/pid.h"
#include "../include/utils.h"

// Point Of No Return hysteresis filter.
static hysteresis_filter_t ponr_filter_;
static const float PONR_LOW_THRESHOLD = 0.10;
static const float PONR_HIGH_THRESHOLD = (M_PI / 4);
// fairly arbitrary thresholds. Want high threshold such that robot could
// never recover at that pitch, low threshold such that robot is upright
// again after a fall.

static bool motors_enabled_;
static bool ponr_tripped_;

// used for control commands.
static const uint32_t command_expiration_ticks_ = 200;
static int8_t duty_cycle_direction_modifiers_[NUM_MOTORS];
static const float pitch_target_list_[NUM_SPEEDS] = { 0.01, 0.03, 0.05 };
static const int8_t duty_cycle_direction_modifier_list_[NUM_SPEEDS] = { 15, 25,
		35 };

static pid_t pid_;
static const float INITIAL_P_GAIN = 650.0;
static const float INITIAL_I_GAIN = 400.0;
static const float INITIAL_D_GAIN = 20.0;
static const float INITIAL_I_MAX = 0.3;
static const float INITIAL_PID_TARGET = 0.0;
static const float PID_MAX_OUTPUT = 100.0;

static int motor_control_updater(void) {
	const uint32_t sample_time = scheduler_get_system_time();
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
	// super impose direction modifiers (0 if we haven't received any commands)
	const int8_t left_duty_cycle = base_duty_cycle
			+ duty_cycle_direction_modifiers_[LEFT_MOTOR];
	const int8_t right_duty_cycle = base_duty_cycle
			+ duty_cycle_direction_modifiers_[RIGHT_MOTOR];

	if (motors_enabled_ && !ponr_tripped_) {
		motor_set_duty_cycle(LEFT_MOTOR, left_duty_cycle);
		motor_set_duty_cycle(RIGHT_MOTOR, right_duty_cycle);
	} else {
		motor_halt();
	}

	return 0;
}

static int motor_control_command_terminator(void) {
	pid_set_target(&pid_, 0.0);
	duty_cycle_direction_modifiers_[LEFT_MOTOR] = 0;
	duty_cycle_direction_modifiers_[RIGHT_MOTOR] = 0;
	return 0;
}

void motor_controller_init(void) {
	mpu6050_init();

	motor_init();
	motors_enabled_ = false;

	pid_init(&pid_, INITIAL_P_GAIN, INITIAL_I_GAIN, INITIAL_D_GAIN, INITIAL_I_MAX,
			PID_MAX_OUTPUT, INITIAL_PID_TARGET);

	hysteresis_filter_init(&ponr_filter_, PONR_LOW_THRESHOLD, PONR_HIGH_THRESHOLD,
			HYSTERESIS_LOW_STATE);
	ponr_tripped_ = false;

	scheduler_init_task(MOTOR_CONTROL_UPDATER, motor_control_updater, 0);
	scheduler_init_task(MOTOR_CONTROL_COMMAND_TERMINATOR,
			motor_control_command_terminator, 0);
}

const pid_t* motor_controller_get_pid(void) {
	return (const pid_t*) &pid_;
}

void motor_controller_enable(bool enable) {
	motors_enabled_ = enable;
	if (!enable) {
		motor_halt();
	} else {
		pid_reset(&pid_);
	}
}

void motor_controller_command(motor_controller_command_t cmd,
		motor_controller_speed_t speed) {
	if (motors_enabled_) {
		// destroy state from previous command.
		motor_control_command_terminator();
		switch (cmd) {
		case FORWARD_COMMAND: {
			const float new_pid_target = pitch_target_list_[speed];
			pid_set_target(&pid_, new_pid_target);
		}
			break;
		case BACKWARD_COMMAND: {
			const float new_pid_target = -pitch_target_list_[speed];
			pid_set_target(&pid_, new_pid_target);
		}
			break;
		case TURN_LEFT_COMMAND: {
			duty_cycle_direction_modifiers_[LEFT_MOTOR] =
					duty_cycle_direction_modifier_list_[speed];
			duty_cycle_direction_modifiers_[RIGHT_MOTOR] =
					-duty_cycle_direction_modifier_list_[speed];
		}
			break;
		case TURN_RIGHT_COMMAND: {
			duty_cycle_direction_modifiers_[LEFT_MOTOR] =
					-duty_cycle_direction_modifier_list_[speed];
			duty_cycle_direction_modifiers_[RIGHT_MOTOR] =
					duty_cycle_direction_modifier_list_[speed];
		}
			break;
		default:
			while (1) {
			} // trap execution if we end up here.
			break;
		}
		scheduler_set_pending_in(MOTOR_CONTROL_COMMAND_TERMINATOR,
				command_expiration_ticks_);
	}
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

void motor_controller_set_pid_target(float target) {
	const float clamped_target = fclamp(target, -PONR_HIGH_THRESHOLD,
			PONR_HIGH_THRESHOLD);
	pid_set_target(&pid_, clamped_target);
}

void motor_controller_set_pid_gains(float p_gain, float i_gain, float d_gain) {
	pid_set_gains(&pid_, p_gain, i_gain, d_gain);
}

