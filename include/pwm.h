/*
 * pwm.h
 *
 *  Created on: Oct 29, 2017
 *      Author: jonathanwingfield
 */

#ifndef PWM_H_
#define PWM_H_

#include <stdint.h>
#include "../include/LPC11xx.h"

typedef enum {
	PWM_PIN_0, PWM_PIN_1
} pwm_pin_t;

void pwm_init(void);
void pwm_set_duty_cycle(pwm_pin_t pwm_pin, uint8_t duty_cycle);
uint8_t pwm_get_duty_cycle(pwm_pin_t pwm_pin);

#endif /* PWM_H_ */
