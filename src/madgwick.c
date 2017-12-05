//=====================================================================================================
// madgwick.c
//=====================================================================================================
//
// Implementation of Madgwick's IMU and AHRS algorithms.
// See: http://www.x-io.co.uk/node/8#open_source_ahrs_and_imu_algorithms
//
//    Date			 Author          Notes
// 29/09/2011	SOH Madgwick    Initial release
// 02/10/2011	SOH Madgwick	  Optimised for reduced CPU load
// 19/02/2012	SOH Madgwick	  Magnetometer measurement is normalised
//
//=====================================================================================================

#include "../include/madgwick.h"

#include <math.h>

#include "../include/utils.h"

static float sample_frequency_ = 75.0;
static float beta_ = 0.1f; // 2 * proportional gain (Kp)

// quaternion of sensor frame relative to auxiliary frame
static quat_t quat_ = { .q0 = 1.0, .q1 = 0.0, .q2 = 0.0, .q3 = 0.0 };

void madgwick_set_sample_frequency(float fs) {
	sample_frequency_ = fs;
}

void madgwick_set_beta(float beta) {
	beta_ = beta;
}

void madgwick_filter(const gyro_t* gyro, const accel_t* accel, quat_t* quat) {
	// First alias variables to shorter names
	float q0 = quat_.q0;
	float q1 = quat_.q1;
	float q2 = quat_.q2;
	float q3 = quat_.q3;

	const float ax = accel->ax;
	const float ay = accel->ay;
	const float az = accel->az;

	const float gx = gyro->gx;
	const float gy = gyro->gy;
	const float gz = gyro->gz;

	// Rate of change of quaternion from gyroscope
	float qDot1 = 0.5f * (-q1 * gx - q2 * gy - q3 * gz);
	float qDot2 = 0.5f * (q0 * gx + q2 * gz - q3 * gy);
	float qDot3 = 0.5f * (q0 * gy - q1 * gz + q3 * gx);
	float qDot4 = 0.5f * (q0 * gz + q1 * gy - q2 * gx);

	// Compute feedback only if accelerometer measurement valid (avoids NaN in accelerometer normalisation)
	if (!((ax == 0.0f) && (ay == 0.0f) && (az == 0.0f))) {

		// Normalize accelerometer measurement
		const float a_recip_norm = fast_inv_sqrt(ax * ax + ay * ay + az * az);
		const float norm_ax = ax * a_recip_norm;
		const float norm_ay = ay * a_recip_norm;
		const float norm_az = az * a_recip_norm;

		// Auxiliary variables to avoid repeated arithmetic
		const float _2q0 = 2.0f * q0;
		const float _2q1 = 2.0f * q1;
		const float _2q2 = 2.0f * q2;
		const float _2q3 = 2.0f * q3;
		const float _4q0 = 4.0f * q0;
		const float _4q1 = 4.0f * q1;
		const float _4q2 = 4.0f * q2;
		const float _8q1 = 8.0f * q1;
		const float _8q2 = 8.0f * q2;
		const float q0q0 = q0 * q0;
		const float q1q1 = q1 * q1;
		const float q2q2 = q2 * q2;
		const float q3q3 = q3 * q3;

		// Gradient decent algorithm corrective step
		const float s0 = _4q0 * q2q2 + _2q2 * norm_ax + _4q0 * q1q1
				- _2q1 * norm_ay;

		const float s1 = _4q1 * q3q3 - _2q3 * norm_ax + 4.0f * q0q0 * q1
				- _2q0 * norm_ay - _4q1 + _8q1 * q1q1 + _8q1 * q2q2 + _4q1 * norm_az;

		const float s2 = 4.0f * q0q0 * q2 + _2q0 * norm_ax + _4q2 * q3q3
				- _2q3 * norm_ay - _4q2 + _8q2 * q1q1 + _8q2 * q2q2 + _4q2 * norm_az;

		const float s3 = 4.0f * q1q1 * q3 - _2q1 * norm_ax + 4.0f * q2q2 * q3
				- _2q2 * norm_ay;

		// normalise step magnitude
		const float s_recip_norm = fast_inv_sqrt(
				s0 * s0 + s1 * s1 + s2 * s2 + s3 * s3);
		const float norm_s0 = s0 * s_recip_norm;
		const float norm_s1 = s1 * s_recip_norm;
		const float norm_s2 = s2 * s_recip_norm;
		const float norm_s3 = s3 * s_recip_norm;

		// Apply feedback step
		qDot1 -= beta_ * norm_s0;
		qDot2 -= beta_ * norm_s1;
		qDot3 -= beta_ * norm_s2;
		qDot4 -= beta_ * norm_s3;
	}

	// Integrate rate of change of quaternion to yield quaternion
	q0 += qDot1 * (1.0f / sample_frequency_);
	q1 += qDot2 * (1.0f / sample_frequency_);
	q2 += qDot3 * (1.0f / sample_frequency_);
	q3 += qDot4 * (1.0f / sample_frequency_);

	// Normalize quaternion
	const float q_recip_norm = fast_inv_sqrt(
			q0 * q0 + q1 * q1 + q2 * q2 + q3 * q3);
	q0 *= q_recip_norm;
	q1 *= q_recip_norm;
	q2 *= q_recip_norm;
	q3 *= q_recip_norm;

	// Write results into callee-supplied quaternion
	quat->q0 = quat_.q0 = q0;
	quat->q1 = quat_.q1 = q1;
	quat->q2 = quat_.q2 = q2;
	quat->q3 = quat_.q3 = q3;
}
