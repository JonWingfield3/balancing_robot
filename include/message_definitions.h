/*
 * message_definitions.h
 *
 *  Created on: Oct 14, 2017
 *      Author: jonathanwingfield
 */

#ifndef MESSAGE_DEFINITIONS_H_
#define MESSAGE_DEFINITIONS_H_

#include <motor_controller.h>
#include <pid.h>
#include <scheduler.h>
#include <utils.h>

/* General Message Definitions */
///////////////////////////////////////////////////////////////////////////////
#define MAX_PLD_LEN (15)

typedef enum {
	MESSAGE_ID_FIRST = 0,

	MOTOR_CONTROL_MESSAGE_GET_DC = MESSAGE_ID_FIRST,
	MOTOR_CONTROL_MESSAGE_CONFIG,
	MOTOR_CONTROL_MESSAGE_STEER_COMMAND,
	MOTOR_CONTROL_MESSAGE_DRIVE_COMMAND,
  MOTOR_CONTROL_MESSAGE_SET_CMD_TIMEOUTS,

	PID_MESSAGE_SET_GAINS,
  PID_MESSAGE_GET_STATE,

	MPU_MESSAGE_GET_IMU_DATA,
  MPU_MESSAGE_CALIBRATE,
  MPU_MESSAGE_SET_SAMPLING_FREQ,

	MADGWICK_MESSAGE_SET_BETA_GAIN,

	SYSTEM_MESSAGE_RESET,
  SYSTEM_MESSAGE_STATUS,

	MESSAGE_ID_LAST,
	NUM_MESSAGE_IDS = MESSAGE_ID_LAST
} message_id_t;

typedef struct PACKED {
	union PACKED {
		struct PACKED {
	    uint8_t msg_id : 4;
	    uint8_t pld_len : 4;
		};
		uint8_t msg_header; // stores msg_id as lower 4 bits of msg_header
	};
	uint8_t pld[MAX_PLD_LEN];
} message_t;

typedef enum {
	NO_ERR,
	BAD_PLD_LEN,
	PANIC_MODE,
}status_t;

typedef struct PACKED {
	status_t status : 7;
	bool motors_enabled : 1;
} status_response_t;
///////////////////////////////////////////////////////////////////////////////

/* Motor Type Message Definitions */
///////////////////////////////////////////////////////////////////////////////
#define DRIVE_PITCH_MIN (-0.25)
#define DRIVE_PITCH_MAX (0.25)
#define DRIVE_PITCH_BYTES (1)

typedef struct PACKED {
	int8_t duty_cycle_left;
	int8_t duty_cycle_right;
} duty_cycle_msg_t;

typedef struct PACKED {
	bool enable_motor;
} motor_control_config_msg_t;

typedef struct PACKED {
  uint8_t drive_pitch[DRIVE_PITCH_BYTES];
} motor_control_drive_msg_t;

typedef struct PACKED {
	int8_t steer_dc_diff;
	// to be added to left motor, subtracted from right motor
} motor_control_steer_msg_t;

typedef struct PACKED {
	uint16_t steer_cmd_timeout;
	uint16_t drive_cmd_timeout;
} motor_control_timeout_msg_t;
///////////////////////////////////////////////////////////////////////////////

/* PID Type Message Definitions*/
///////////////////////////////////////////////////////////////////////////////
#define PID_TARGET_MIN (-0.8)
#define PID_TARGET_MAX  (0.8)
#define PID_TARGET_BYTES (2)

#define PID_INPUT_MIN (-2.0)
#define PID_INPUT_MAX  (2.0)
#define PID_INPUT_BYTES (2)

#define P_ERROR_UNWEIGHTED_MIN (-2.0)
#define P_ERROR_UNWEIGHTED_MAX  (2.0)
#define P_ERROR_UNWEIGHTED_BYTES (1)

#define I_ERROR_UNWEIGHTED_MIN (-5.0)
#define I_ERROR_UNWEIGHTED_MAX  (5.0)
#define I_ERROR_UNWEIGHTED_BYTES (1)

#define D_ERROR_UNWEIGHTED_MIN (-10.0)
#define D_ERROR_UNWEIGHTED_MAX  (10.0)
#define D_ERROR_UNWEIGHTED_BYTES (1)

#define PID_ERROR_WEIGHTED_MIN (-100.0)
#define PID_ERROR_WEIGHTED_MAX  (100.0)
#define PID_ERROR_WEIGHTED_BYTES (2)

typedef struct PACKED {
	uint16_t p_gain;
  uint16_t i_gain;
	uint16_t d_gain;
} pid_set_gains_msg_t;

typedef struct PACKED {
  uint8_t target[PID_TARGET_BYTES];
}pid_set_target_msg_t;

typedef struct PACKED {
	uint8_t pid_target[PID_TARGET_BYTES];
  uint8_t pid_input[PID_INPUT_BYTES];
  uint8_t p_error[P_ERROR_UNWEIGHTED_BYTES];
  uint8_t i_error[I_ERROR_UNWEIGHTED_BYTES];
  uint8_t d_error[D_ERROR_UNWEIGHTED_BYTES];
  int8_t pid_output;
}pid_unweighted_state_msg_t;

typedef struct PACKED {
	uint8_t pid_target[PID_TARGET_BYTES];
  uint8_t pid_input[PID_INPUT_BYTES];
  uint8_t p_error[PID_ERROR_WEIGHTED_BYTES];
  uint8_t i_error[PID_ERROR_WEIGHTED_BYTES];
  uint8_t d_error[PID_ERROR_WEIGHTED_BYTES];
  int8_t pid_output;
}pid_weighted_state_msg_t;

typedef struct PACKED {
  bool weight_errors;
}pid_state_request_msg_t;
///////////////////////////////////////////////////////////////////////////////

/* MPU6050 Type Message Definitions */
///////////////////////////////////////////////////////////////////////////////
#define GYRO_MIN (-15.0)
#define GYRO_MAX (15.0)
#define GYRO_BYTES (2)

#define ACCEL_MIN (-4.0)
#define ACCEL_MAX (4.0)
#define ACCEL_BYTES (2)

#define TEMP_MIN (0)
#define TEMP_MAX (100)
#define TEMP_BYTES (1)

typedef struct PACKED {
	uint8_t gyro_x[GYRO_BYTES];
	uint8_t gyro_y[GYRO_BYTES];
	uint8_t gyro_z[GYRO_BYTES];
  uint8_t accel_x[ACCEL_BYTES];
  uint8_t accel_y[ACCEL_BYTES];
  uint8_t accel_z[ACCEL_BYTES];
  uint8_t temp[TEMP_BYTES];
} mpu_data_msg_t;

typedef struct PACKED {
	uint16_t sample_frequency;
} mpu_sample_frequency_msg_t;
///////////////////////////////////////////////////////////////////////////////

/* Madgwick Type Message Definitions */
///////////////////////////////////////////////////////////////////////////////
#define MADGWICK_BETA_GAIN_MIN (0.0)
#define MADGWICK_BETA_GAIN_MAX (1.0)
#define MADGWICK_BETA_GAIN_BYTES (2)

typedef struct PACKED {
	uint8_t beta_gain[MADGWICK_BETA_GAIN_BYTES];
} madgwick_beta_gain_msg_t;
///////////////////////////////////////////////////////////////////////////////

#endif /* MESSAGE_DEFINITIONS_H_ */
