/*
 * message_service.c
 *
 *  Created on: Oct 14, 2017
 *      Author: jonathanwingfield
 */

#include "../include/message_service.h"

#include <stdint.h>
#include <stdlib.h>
#include <stdbool.h>

#include "../include/adc.h"
#include "../include/balancing_robot.h"
#include "../include/led.h"
#include "../include/madgwick.h"
#include "../include/message_definitions.h"
#include "../include/motor.h"
#include "../include/motor_controller.h"
#include "../include/mpu6050.h"
#include "../include/packed_float.h"
#include "../include/pid.h"
#include "../include/pwm.h"
#include "../include/scheduler.h"
#include "../include/uart.h"
#include "../include/uart_transport.h"
#include "../include/utils.h"

static int message_handler(void) {
	const message_t* recvd_msg = uart_transport_get_message();

	// by default create simple status response set to NO_ERR.
	message_t rsp_msg;
	rsp_msg.msg_id = recvd_msg->msg_id;
	rsp_msg.pld_len = sizeof(status_response_t);
	status_response_t *status_rsp = (status_response_t*) rsp_msg.pld;
	status_rsp->status = NO_ERR;

	switch (recvd_msg->msg_id) {
	case MOTOR_CONTROL_MESSAGE_SET_DC: {
		if (recvd_msg->pld_len != sizeof(duty_cycle_msg_t)) {
			status_rsp->status = BAD_PLD_LEN;
			break;
		}
		const duty_cycle_msg_t *dc_msg = (duty_cycle_msg_t*) recvd_msg->pld;
		motor_set_duty_cycle(LEFT_MOTOR, dc_msg->duty_cycle_left);
		motor_set_duty_cycle(RIGHT_MOTOR, dc_msg->duty_cycle_right);
	}
		break;
	case MOTOR_CONTROL_MESSAGE_GET_DC: {
		if (recvd_msg->pld_len != 0) {
			status_rsp->status = BAD_PLD_LEN;
			break;
		}
		duty_cycle_msg_t *dc_msg = (duty_cycle_msg_t*) rsp_msg.pld;
		rsp_msg.pld_len = sizeof(duty_cycle_msg_t);
		dc_msg->duty_cycle_left = pwm_get_duty_cycle(PWM_PIN_0);
		dc_msg->duty_cycle_right = pwm_get_duty_cycle(PWM_PIN_1);
	}
		break;
	case MOTOR_CONTROL_MESSAGE_CONFIG: {
		if (recvd_msg->pld_len != sizeof(motor_control_config_msg_t)) {
			status_rsp->status = BAD_PLD_LEN;
			break;
		}
		const motor_control_config_msg_t *mtr_config_msg =
				(motor_control_config_msg_t*) recvd_msg->pld;
		motor_controller_enable(mtr_config_msg->enable_motor);
	}
		break;
	case MOTOR_CONTROL_MESSAGE_COMMAND: {
		if (recvd_msg->pld_len != sizeof(motor_control_command_msg_t)) {
			status_rsp->status = BAD_PLD_LEN;
		}
		motor_control_command_msg_t* mtr_control_cmd_msg =
				(motor_control_command_msg_t*) recvd_msg->pld;
		motor_controller_command(mtr_control_cmd_msg->command,
				mtr_control_cmd_msg->speed);
	}
		break;
	case PID_MESSAGE_SET_GAINS: {
		if (recvd_msg->pld_len != sizeof(pid_set_gains_msg_t)) {
			status_rsp->status = BAD_PLD_LEN;
			break;
		}
		const pid_set_gains_msg_t *pid_msg = (pid_set_gains_msg_t*) recvd_msg->pld;
		const float p_gain = (float) pid_msg->p_gain;
		const float i_gain = (float) pid_msg->i_gain;
		const float d_gain = (float) pid_msg->d_gain;
		motor_controller_set_pid_gains(p_gain, i_gain, d_gain);
	}
		break;
	case PID_MESSAGE_SET_TARGET: {
		if (recvd_msg->pld_len != sizeof(pid_set_target_msg_t)) {
			status_rsp->status = BAD_PLD_LEN;
			break;
		}
		const pid_set_target_msg_t *pid_target_msg =
				(const pid_set_target_msg_t*) recvd_msg->pld;
		const float pid_target = unpack_float(pid_target_msg->target,
		PID_TARGET_MIN, PID_TARGET_MAX, PID_TARGET_BYTES);
		motor_controller_set_pid_target(pid_target);
	}
		break;
	case PID_MESSAGE_GET_STATE: {
		if (recvd_msg->pld_len != sizeof(pid_state_request_msg_t)) {
			status_rsp->status = BAD_PLD_LEN;
			break;
		}
		const pid_state_request_msg_t* pid_state_request =
				(const pid_state_request_msg_t*) recvd_msg->pld;

		const float pid_target = motor_controller_get_pid_target();
		pid_error_t pid_errors;
		motor_controller_get_pid_errors(&pid_errors,
				pid_state_request->weight_errors);

		if (pid_state_request->weight_errors) {
			rsp_msg.pld_len = sizeof(pid_weighted_state_msg_t);
			pid_weighted_state_msg_t* pid_state_msg =
					(pid_weighted_state_msg_t*) rsp_msg.pld;

			pack_float(pid_target, pid_state_msg->pid_target, PID_TARGET_MIN,
			PID_TARGET_MAX, PID_TARGET_BYTES);
			pack_float(pid_errors.p_error, pid_state_msg->p_error,
			PID_ERROR_WEIGHTED_MIN, PID_ERROR_WEIGHTED_MAX,
			PID_ERROR_WEIGHTED_BYTES);
			pack_float(pid_errors.i_error, pid_state_msg->i_error,
			PID_ERROR_WEIGHTED_MIN, PID_ERROR_WEIGHTED_MAX,
			PID_ERROR_WEIGHTED_BYTES);
			pack_float(pid_errors.d_error, pid_state_msg->d_error,
			PID_ERROR_WEIGHTED_MIN, PID_ERROR_WEIGHTED_MAX,
			PID_ERROR_WEIGHTED_BYTES);

			const float pitch = mpu6050_get_pitch();
			pack_float(pitch, pid_state_msg->pid_input, PID_INPUT_MIN, PID_INPUT_MAX,
			PID_INPUT_BYTES);
			pid_state_msg->pid_output = motor_controller_get_pid_output();
		} else { // unweighted errors.
			rsp_msg.pld_len = sizeof(pid_unweighted_state_msg_t);
			pid_unweighted_state_msg_t* pid_state_msg =
					(pid_unweighted_state_msg_t*) rsp_msg.pld;

			pack_float(pid_target, pid_state_msg->pid_target, PID_TARGET_MIN,
			PID_TARGET_MAX, PID_TARGET_BYTES);
			pack_float(pid_errors.p_error, pid_state_msg->p_error,
			P_ERROR_UNWEIGHTED_MIN,
			P_ERROR_UNWEIGHTED_MAX, P_ERROR_UNWEIGHTED_BYTES);
			pack_float(pid_errors.i_error, pid_state_msg->i_error,
			I_ERROR_UNWEIGHTED_MIN,
			I_ERROR_UNWEIGHTED_MAX, I_ERROR_UNWEIGHTED_BYTES);
			pack_float(pid_errors.d_error, pid_state_msg->d_error,
			D_ERROR_UNWEIGHTED_MIN,
			D_ERROR_UNWEIGHTED_MAX, D_ERROR_UNWEIGHTED_BYTES);

			const float pitch = mpu6050_get_pitch();
			pack_float(pitch, pid_state_msg->pid_input, PID_INPUT_MIN, PID_INPUT_MAX,
			PID_INPUT_BYTES);
			pid_state_msg->pid_output = motor_controller_get_pid_output();
		}
	}
		break;
	case MPU_MESSAGE_GET_IMU_DATA: {
		if (recvd_msg->pld_len != 0) {
			status_rsp->status = BAD_PLD_LEN;
			break;
		}
		mpu_data_msg_t* mpu_msg = (mpu_data_msg_t*) rsp_msg.pld;
		rsp_msg.pld_len = sizeof(mpu_data_msg_t);

		gyro_t gyro_data;
		mpu6050_get_gyro(&gyro_data);
		pack_float(gyro_data.gx, mpu_msg->gyro_x, GYRO_MIN, GYRO_MAX, GYRO_BYTES);
		pack_float(gyro_data.gy, mpu_msg->gyro_y, GYRO_MIN, GYRO_MAX, GYRO_BYTES);
		pack_float(gyro_data.gz, mpu_msg->gyro_z, GYRO_MIN, GYRO_MAX, GYRO_BYTES);

		accel_t accel_data;
		mpu6050_get_accel(&accel_data);
		pack_float(accel_data.ax, mpu_msg->accel_x, ACCEL_MIN, ACCEL_MAX,
		ACCEL_BYTES);
		pack_float(accel_data.ay, mpu_msg->accel_y, ACCEL_MIN, ACCEL_MAX,
		ACCEL_BYTES);
		pack_float(accel_data.az, mpu_msg->accel_z, ACCEL_MIN, ACCEL_MAX,
		ACCEL_BYTES);

		const float temp_data = mpu6050_get_temp();
		pack_float(temp_data, mpu_msg->temp, TEMP_MIN, TEMP_MAX, TEMP_BYTES);
	}
		break;
	case MPU_MESSAGE_CALIBRATE: {
		if (recvd_msg->pld_len != 0) {
			status_rsp->status = BAD_PLD_LEN;
			break;
		}
		mpu6050_calibrate();
	}
		break;
	case MPU_MESSAGE_SET_SAMPLING_FREQ: {
		if (recvd_msg->pld_len != sizeof(mpu_sample_frequency_msg_t)) {
			status_rsp->status = BAD_PLD_LEN;
			break;
		}
		const mpu_sample_frequency_msg_t* sys_freq_msg =
				(mpu_sample_frequency_msg_t*) recvd_msg->pld;
		const uint32_t sample_frequency = sys_freq_msg->sample_frequency;
		mpu6050_set_sampling_frequency(sample_frequency);
	}
		break;
	case MADGWICK_MESSAGE_SET_BETA_GAIN: {
		if (recvd_msg->pld_len != sizeof(madgwick_beta_gain_msg_t)) {
			status_rsp->status = BAD_PLD_LEN;
			break;
		}
		const madgwick_beta_gain_msg_t* madgwick_beta_gain_msg =
				(madgwick_beta_gain_msg_t*) recvd_msg->pld;
		const float beta_gain = unpack_float(madgwick_beta_gain_msg->beta_gain,
		MADGWICK_BETA_GAIN_MIN, MADGWICK_BETA_GAIN_MAX,
		MADGWICK_BETA_GAIN_BYTES);
		madgwick_set_beta_gain(beta_gain);
	}
		break;
	default:
		while (1) {
		} // trap execution if we end up here.
		break;
	}
	uart_transport_send_message(&rsp_msg);
	return 0;
}

void message_service_init(void) {
	uart_transport_init();
	scheduler_init_task(MESSAGE_HANDLER, message_handler, 0);
}

