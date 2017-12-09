/*
 * mpu6050.c
 *
 *  Created on: Oct 28, 2017
 *      Author: jonathanwingfield
 */

#include <mpu6050.h>

#include <math.h>

#include <balancing_robot.h>
#include <i2c.h>
#include <madgwick.h>
#include <profiler.h>
#include <scheduler.h>
#include <utils.h>

typedef enum {
	SELF_TEST_X = 13,
	SELF_TEST_Y,
	SELF_TEST_Z,
	SELF_TEST_A,
	SMPLRT_DIV = 25,
	CONFIG,
	GYRO_CONFIG,
	ACCEL_CONFIG,
	MOT_THR = 31,
	FIFO_EN = 35,
	I2C_MST_CTRL,
	I2C_SLV0_ADDR,
	I2C_SLV0_REG,
	I2C_SLV0_CTRL,
	I2C_SLV1_ADDR,
	I2C_SLV1_REG,
	I2C_SLV1_CTRL,
	I2C_SLV2_ADDR,
	I2C_SLV2_REG,
	I2C_SLV2_CTRL,
	I2C_SLV3_ADDR,
	I2C_SLV3_REG,
	I2C_SLV3_CTRL,
	I2C_SLV4_ADDR,
	I2C_SLV4_REG,
	I2C_SLV4_DO,
	I2C_SLV4_CTRL,
	I2C_SLV4_DI,
	I2C_MST_STATUS,
	INT_PIN_CFG,
	INT_ENABLE,
	INT_STATUS = 58,
	ACCEL_XOUT_H,
	ACCEL_XOUT_L,
	ACCEL_YOUT_H,
	ACCEL_YOUT_L,
	ACCEL_ZOUT_H,
	ACCEL_ZOUT_L,
	TEMP_OUT_H,
	TEMP_OUT_L,
	GYRO_XOUT_H,
	GYRO_XOUT_L,
	GYRO_YOUT_H,
	GYRO_YOUT_L,
	GYRO_ZOUT_H,
	GYRO_ZOUT_L,
	EXT_SENS_DATA_00,
	EXT_SENS_DATA_01,
	EXT_SENS_DATA_02,
	EXT_SENS_DATA_03,
	EXT_SENS_DATA_04,
	EXT_SENS_DATA_05,
	EXT_SENS_DATA_06,
	EXT_SENS_DATA_07,
	EXT_SENS_DATA_08,
	EXT_SENS_DATA_09,
	EXT_SENS_DATA_10,
	EXT_SENS_DATA_11,
	EXT_SENS_DATA_12,
	EXT_SENS_DATA_13,
	EXT_SENS_DATA_14,
	EXT_SENS_DATA_15,
	EXT_SENS_DATA_16,
	EXT_SENS_DATA_17,
	EXT_SENS_DATA_18,
	EXT_SENS_DATA_19,
	EXT_SENS_DATA_20,
	EXT_SENS_DATA_21,
	EXT_SENS_DATA_22,
	EXT_SENS_DATA_23,
	I2C_SLV0_DO = 99,
	I2C_SLV1_DO,
	I2C_SLV2_DO,
	I2C_SLV3_DO,
	I2C_MST_DELAY_CTRL,
	SIGNAL_PATH_RESET,
	MOT_DETECT_CTRL,
	USER_CTRL,
	PWR_MGMT_1,
	PWR_MGMT_2,
	FIFO_COUNTH = 114,
	FIFO_COUNTL,
	FIFO_R_W,
	WHO_AM_I,
} mpu6050_register_t;

static const uint8_t MPU6050_SLAVE_ADDR = 0x68;

static accel_t accel_;
static gyro_t gyro_;
static quat_t quat_;
static float temp_;
static float pitch_;
static float pitch_offset_;

static inline int reg_to_buf_index(int reg) {
	return (reg - ACCEL_XOUT_H);
}

void mpu6050_write_reg(mpu6050_register_t reg, uint8_t data) {
	uint8_t write_buf[2] = { reg, data };
	i2c_write(MPU6050_SLAVE_ADDR, write_buf, 2, true);
}

static int mpu6050_read_reg(mpu6050_register_t reg, uint8_t* data) {
	uint8_t* ui8reg = (uint8_t*) &reg;
	if (i2c_write(MPU6050_SLAVE_ADDR, ui8reg, 1, false) == -1) {
		return -1;
	}
	if (i2c_read(MPU6050_SLAVE_ADDR, data, 1) == -1) {
		return -1;
	}
	return 0;
}

static int mpu6050_read_reg_n(mpu6050_register_t reg, uint8_t* data, int n) {
	uint8_t* ui8reg = (uint8_t*) &reg;
	if (i2c_write(MPU6050_SLAVE_ADDR, ui8reg, 1, false) == -1) {
		return -1;
	}
	if (i2c_read(MPU6050_SLAVE_ADDR, data, n) == -1) {
		return -1;
	}
	return 0;
}

static int mpu6050_data_collector(void) {
	static uint32_t fault_counter = 0;
	uint8_t mpu_data_buf[(GYRO_ZOUT_L - ACCEL_XOUT_H) + 1];
	if (mpu6050_read_reg_n(ACCEL_XOUT_H, mpu_data_buf, ARRAY_SIZE(mpu_data_buf))
			== 0) {
		fault_counter = 0;

		const int16_t ax_raw = ((int16_t) ((mpu_data_buf[reg_to_buf_index(
				ACCEL_XOUT_H)] << 8) + mpu_data_buf[reg_to_buf_index(ACCEL_XOUT_L)]));

		const int16_t ay_raw = ((int16_t) ((mpu_data_buf[reg_to_buf_index(
				ACCEL_YOUT_H)] << 8) + mpu_data_buf[reg_to_buf_index(ACCEL_YOUT_L)]));

		const int16_t az_raw = ((int16_t) ((mpu_data_buf[reg_to_buf_index(
				ACCEL_ZOUT_H)] << 8) + mpu_data_buf[reg_to_buf_index(ACCEL_ZOUT_L)]));

		// convert accel data to g's
		accel_.ax = ((float) ax_raw) / 8192.0;
		accel_.ay = ((float) ay_raw) / 8192.0;
		accel_.az = ((float) az_raw) / 8192.0;

		const int16_t gx_raw = ((int16_t) ((mpu_data_buf[reg_to_buf_index(
				GYRO_XOUT_H)] << 8) + mpu_data_buf[reg_to_buf_index(GYRO_XOUT_H)]));

		const int16_t gy_raw = ((int16_t) ((mpu_data_buf[reg_to_buf_index(
				GYRO_YOUT_H)] << 8) + mpu_data_buf[reg_to_buf_index(GYRO_YOUT_H)]));

		const int16_t gz_raw = ((int16_t) ((mpu_data_buf[reg_to_buf_index(
				GYRO_ZOUT_H)] << 8) + mpu_data_buf[reg_to_buf_index(GYRO_ZOUT_H)]));

		// convert gyro data to rad/s
		gyro_.gx = ((float) gx_raw) / 939.65;
		gyro_.gy = ((float) gy_raw) / 939.65;
		gyro_.gz = ((float) gz_raw) / 939.65;

		// temperature sort of just comes along for the ride. Not currently using.
		const int16_t temp_raw = ((int16_t) ((mpu_data_buf[reg_to_buf_index(
				TEMP_OUT_H)] << 8) + mpu_data_buf[reg_to_buf_index(TEMP_OUT_L)]));

		temp_ = (temp_raw / 340.0) + 36.53;

		madgwick_filter(&gyro_, &accel_, &quat_);
		pitch_ = -asinf(-2.0f * ((quat_.q1) * (quat_.q3) - (quat_.q0) * (quat_.q2)))
				- pitch_offset_;

		scheduler_set_pending(MOTOR_CONTROL_UPDATER);
	} else {
		fault_counter++;
		if (fault_counter >= 100) {
			scheduler_set_pending(SYSTEM_PANIC);
		}
	}
	return 0;
}

static bool comm_test(void) {
	uint8_t data_buf;
	mpu6050_read_reg(WHO_AM_I, &data_buf);
	return (data_buf == MPU6050_SLAVE_ADDR);
}

void mpu6050_init(void) {
	i2c_init();
	if (!comm_test()) {
		scheduler_set_pending(SYSTEM_PANIC);
		return;
	}

	mpu6050_write_reg(PWR_MGMT_1, 0x00);   // exit sleep mode.
	mpu6050_write_reg(SMPLRT_DIV, 0x01);   // configure 1kHz for gyro and accel
	mpu6050_write_reg(CONFIG, 0x01);       // enable DLPF
	mpu6050_write_reg(GYRO_CONFIG, 0x18);  // set to 2000 deg/s
	mpu6050_write_reg(ACCEL_CONFIG, 0x08); // set to +-4g sensitivity.

	scheduler_init_task(MPU_DATA_COLLECTOR, mpu6050_data_collector,
			(SCHEDULER_FREQUENCY / MPU6050_SAMPLE_FREQUENCY));
}

void mpu6050_get_accel(accel_t* accel) {
	const uint8_t* src_ite = (const uint8_t*) &accel_;
	uint8_t* dst_ite = (uint8_t*) accel;
	memcopy(src_ite, dst_ite, sizeof(accel_t));
}

void mpu6050_get_gyro(gyro_t* gyro) {
	const uint8_t* src_ite = (const uint8_t*) &gyro_;
	uint8_t* dst_ite = (uint8_t*) gyro;
	memcopy(src_ite, dst_ite, sizeof(gyro_t));
}

float mpu6050_get_pitch(void) {
	return pitch_;
}

float mpu6050_get_temp(void) {
	return temp_;
}

void mpu6050_calibrate(void) {
	const int num_samples_for_calibration = 16;
	float pitch_offset = 0.0;
	for (int i = 0; i < num_samples_for_calibration; ++i) {
		mpu6050_data_collector();
		pitch_offset += pitch_;
	}
	pitch_offset_ += pitch_offset / num_samples_for_calibration;
}

void mpu6050_set_sampling_frequency(uint32_t frequency) {
	const uint32_t clamped_frequency = clamp(frequency, 10, 100);
	madgwick_set_sample_frequency(clamped_frequency);

	const uint32_t sampling_periodicity = 1000.0 / (float) clamped_frequency;
	scheduler_set_task_periodicity(MPU_DATA_COLLECTOR, sampling_periodicity);
}
