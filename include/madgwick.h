//=====================================================================================================
// madgwick.h
//=====================================================================================================
//
// Implementation of Madgwick's IMU and AHRS algorithms.
// See: http://www.x-io.co.uk/node/8#open_source_ahrs_and_imu_algorithms
//
//    Date			    Author          Notes
// 29/09/2011	   SOH Madgwick    Initial release
// 02/10/2011	   SOH Madgwick	   Optimised for reduced CPU load
//
//=====================================================================================================

#ifndef MADGWICK_H_
#define MADGWICK_H_

#include <mpu6050.h>

typedef struct {
	float q0;
	float q1;
	float q2;
	float q3;
} quat_t;

// the beast itself
void madgwick_filter(const gyro_t* gyro, const accel_t* accel, quat_t* quat);

// tuning functions
void madgwick_set_sample_frequency(float fs);
void madgwick_set_beta_gain(float beta);

#endif /* MADGWICK_H_ */
