/*
 * motor.h
 *
 *  Created on: Nov 6, 2017
 *      Author: jonathanwingfield
 */

#ifndef MOTOR_H_
#define MOTOR_H_

#include <stdbool.h>
#include <stdint.h>

typedef enum {
	LEFT_MOTOR, RIGHT_MOTOR, NUM_MOTORS
} motor_t;

void motor_init(void);
// negative duty cycles correspond to spinning motors in "reverse"
void motor_set_duty_cycle(motor_t motor, int8_t duty_cycle);
int8_t motor_get_duty_cycle(motor_t motor);
void motor_halt(void);

#endif /* MOTOR_H_ */
