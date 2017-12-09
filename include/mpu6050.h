/*
 * mpu6050.h
 *
 *  Created on: Oct 28, 2017
 *      Author: jonathanwingfield
 */

#ifndef MPU6050_H_
#define MPU6050_H_

#include <stdint.h>

#define MPU6050_SAMPLE_FREQUENCY (66.6)

typedef struct {
	float ax;
	float ay;
	float az;
} accel_t;

typedef struct {
	float gx;
	float gy;
	float gz;
} gyro_t;

void mpu6050_init(void);
void mpu6050_calibrate(void);
void mpu6050_set_sampling_frequency(uint32_t frequency);

void mpu6050_get_accel(accel_t* accel);
void mpu6050_get_gyro(gyro_t* gyro);
float mpu6050_get_pitch(void);
float mpu6050_get_temp(void);

#endif /* MPU6050_H_ */
