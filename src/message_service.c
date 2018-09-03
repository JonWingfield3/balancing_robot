/*
 * message_service.c
 *
 *  Created on: Oct 14, 2017
 *      Author: jonathanwingfield
 */

#include <message_service.h>

#include <stdint.h>
#include <stdlib.h>
#include <stdbool.h>

#include <balancing_robot.h>
#include <led.h>
#include <madgwick.h>
#include <message_definitions.h>
#include <motor.h>
#include <motor_controller.h>
#include <mpu6050.h>
#include <packed_float.h>
#include <pid.h>
#include <pwm.h>
#include <scheduler.h>
#include <message_transport.h>
#include <utils.h>

static int message_handler(void) {
	const message_t* recvd_msg = message_transport_get_message();

	// by default create simple status response set to NO_ERR.
	message_t rsp_msg;
	rsp_msg.msg_id = recvd_msg->msg_id;
	rsp_msg.pld_len = sizeof(status_response_t);
	status_response_t *status_rsp = (status_response_t*) rsp_msg.pld;
	status_rsp->status = system_panicked() ? PANIC_MODE : NO_ERR;
	status_rsp->motors_enabled = motor_controller_enabled();

	switch (recvd_msg->msg_id) {
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
	case MOTOR_CONTROL_MESSAGE_STEER_COMMAND: {
		if (recvd_msg->pld_len != sizeof(motor_control_steer_msg_t)) {
			status_rsp->status = BAD_PLD_LEN;
			break;
		}
		motor_control_steer_msg_t* motor_control_steer_msg =
				(motor_control_steer_msg_t*) recvd_msg->pld;
		motor_controller_steer_command(motor_control_steer_msg->steer_dc_diff);
	}
		break;
	case MOTOR_CONTROL_MESSAGE_DRIVE_COMMAND: {
		if (recvd_msg->pld_len != sizeof(motor_control_drive_msg_t)) {
			status_rsp->status = BAD_PLD_LEN;
			break;
		}
		motor_control_drive_msg_t* motor_control_drive_msg =
				(motor_control_drive_msg_t*) recvd_msg->pld;
		const float drive_pitch = decompress(motor_control_drive_msg->drive_pitch,
		DRIVE_PITCH_MIN, DRIVE_PITCH_MAX, DRIVE_PITCH_BYTES);
		motor_controller_drive_command(drive_pitch);
	}
		break;
	case MOTOR_CONTROL_MESSAGE_SET_CMD_TIMEOUTS: {
		if (recvd_msg->pld_len != sizeof(motor_control_timeout_msg_t)) {
			status_rsp->status = BAD_PLD_LEN;
			break;
		}
		motor_control_timeout_msg_t* motor_control_timeout_msg =
				(motor_control_timeout_msg_t*) recvd_msg->pld;
		motor_controller_set_command_timeouts(
				motor_control_timeout_msg->steer_cmd_timeout,
				motor_control_timeout_msg->drive_cmd_timeout);
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

			compress(pid_target, pid_state_msg->pid_target, PID_TARGET_MIN,
			PID_TARGET_MAX, PID_TARGET_BYTES);
			compress(pid_errors.p_error, pid_state_msg->p_error,
			PID_ERROR_WEIGHTED_MIN, PID_ERROR_WEIGHTED_MAX,
			PID_ERROR_WEIGHTED_BYTES);
			compress(pid_errors.i_error, pid_state_msg->i_error,
			PID_ERROR_WEIGHTED_MIN, PID_ERROR_WEIGHTED_MAX,
			PID_ERROR_WEIGHTED_BYTES);
			compress(pid_errors.d_error, pid_state_msg->d_error,
			PID_ERROR_WEIGHTED_MIN, PID_ERROR_WEIGHTED_MAX,
			PID_ERROR_WEIGHTED_BYTES);

			const float pitch = mpu6050_get_pitch();
			compress(pitch, pid_state_msg->pid_input, PID_INPUT_MIN, PID_INPUT_MAX,
			PID_INPUT_BYTES);
			pid_state_msg->pid_output = motor_controller_get_pid_output();
		} else { // unweighted errors.
			rsp_msg.pld_len = sizeof(pid_unweighted_state_msg_t);
			pid_unweighted_state_msg_t* pid_state_msg =
					(pid_unweighted_state_msg_t*) rsp_msg.pld;

			compress(pid_target, pid_state_msg->pid_target, PID_TARGET_MIN,
			PID_TARGET_MAX, PID_TARGET_BYTES);
			compress(pid_errors.p_error, pid_state_msg->p_error,
			P_ERROR_UNWEIGHTED_MIN,
			P_ERROR_UNWEIGHTED_MAX, P_ERROR_UNWEIGHTED_BYTES);
			compress(pid_errors.i_error, pid_state_msg->i_error,
			I_ERROR_UNWEIGHTED_MIN,
			I_ERROR_UNWEIGHTED_MAX, I_ERROR_UNWEIGHTED_BYTES);
			compress(pid_errors.d_error, pid_state_msg->d_error,
			D_ERROR_UNWEIGHTED_MIN,
			D_ERROR_UNWEIGHTED_MAX, D_ERROR_UNWEIGHTED_BYTES);

			const float pitch = mpu6050_get_pitch();
			compress(pitch, pid_state_msg->pid_input, PID_INPUT_MIN, PID_INPUT_MAX,
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
		compress(gyro_data.gx, mpu_msg->gyro_x, GYRO_MIN, GYRO_MAX, GYRO_BYTES);
		compress(gyro_data.gy, mpu_msg->gyro_y, GYRO_MIN, GYRO_MAX, GYRO_BYTES);
		compress(gyro_data.gz, mpu_msg->gyro_z, GYRO_MIN, GYRO_MAX, GYRO_BYTES);

		accel_t accel_data;
		mpu6050_get_accel(&accel_data);
		compress(accel_data.ax, mpu_msg->accel_x, ACCEL_MIN, ACCEL_MAX,
		ACCEL_BYTES);
		compress(accel_data.ay, mpu_msg->accel_y, ACCEL_MIN, ACCEL_MAX,
		ACCEL_BYTES);
		compress(accel_data.az, mpu_msg->accel_z, ACCEL_MIN, ACCEL_MAX,
		ACCEL_BYTES);

		const float temp_data = mpu6050_get_temp();
		compress(temp_data, mpu_msg->temp, TEMP_MIN, TEMP_MAX, TEMP_BYTES);
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
		const float beta_gain = decompress(madgwick_beta_gain_msg->beta_gain,
		MADGWICK_BETA_GAIN_MIN, MADGWICK_BETA_GAIN_MAX,
		MADGWICK_BETA_GAIN_BYTES);
		madgwick_set_beta_gain(beta_gain);
	}
		break;
	case SYSTEM_MESSAGE_RESET: {
		if (recvd_msg->pld_len != 0) {
			status_rsp->status = BAD_PLD_LEN;
			break;
		}
		message_transport_send_message(&rsp_msg);
		// give system time to send response out before resetting.
		scheduler_delay_ms(10);
		system_reset();
		while (true) {
		} // we shouldn't return but just to be safe.
	}
		break;
	case SYSTEM_MESSAGE_STATUS: {
		if (recvd_msg->pld_len != 0) {
			status_rsp->status = BAD_PLD_LEN;
			break;
		}
		// Nothing to do here. Just want a status response.
	}
		break;
	}

	message_transport_send_message(&rsp_msg);
	return 0;
}

void message_service_init(void) {
	message_transport_init();
	scheduler_init_task(MESSAGE_HANDLER, message_handler, 0);
}

