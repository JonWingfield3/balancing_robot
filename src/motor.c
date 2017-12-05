/*
 * motor.c
 *
 *  Created on: Nov 6, 2017
 *      Author: jonathanwingfield
 */

#include <motor.h>

#include <stdbool.h>
#include <stdint.h>
#include <stdlib.h>

#include <LPC11xx.h>

#include <gpio.h>
#include <pwm.h>
#include <scheduler.h>
#include <utils.h>

typedef enum {
	FORWARD_DIRECTION, REVERSE_DIRECTION, NUM_DIRECTIONS, HALTED
} motor_direction_t;

static const uint8_t MAX_DC_MAG = 85;
static motor_direction_t directions_[NUM_MOTORS] = { 0, 0 };

static const gpio_port_t MOTOR_IN_GPIO_BASE = GPIO_PORT2;
static const uint16_t MOTOR_IN_GPIO_PINS[NUM_MOTORS][NUM_DIRECTIONS] = { { PIN(
		6), PIN(7) }, { PIN(8), PIN(9) } };

static const pwm_pin_t MOTOR_PWMS[NUM_MOTORS] = { PWM_PIN_0, PWM_PIN_1 };

static const gpio_port_t STANDBY_GPIO_BASE = GPIO_PORT2;
static const uint16_t STANDBY_GPIO_PIN = PIN(10);

static void set_direction(motor_t motor, motor_direction_t direction) {
	if (direction == FORWARD_DIRECTION) {
		gpio_pin_write(MOTOR_IN_GPIO_BASE,
				(MOTOR_IN_GPIO_PINS[motor][0] | MOTOR_IN_GPIO_PINS[motor][1]),
				MOTOR_IN_GPIO_PINS[motor][0]);
	} else {
		gpio_pin_write(MOTOR_IN_GPIO_BASE,
				(MOTOR_IN_GPIO_PINS[motor][0] | MOTOR_IN_GPIO_PINS[motor][1]),
				MOTOR_IN_GPIO_PINS[motor][1]);
	}
}

void motor_init(void) {
	pwm_init();
	gpio_pin_configure(MOTOR_IN_GPIO_BASE,
			(MOTOR_IN_GPIO_PINS[LEFT_MOTOR][0] | MOTOR_IN_GPIO_PINS[RIGHT_MOTOR][0]
					| MOTOR_IN_GPIO_PINS[LEFT_MOTOR][1]
					| MOTOR_IN_GPIO_PINS[RIGHT_MOTOR][1]), GPIO_OUTPUT);
	gpio_pin_configure(STANDBY_GPIO_BASE, STANDBY_GPIO_PIN, GPIO_OUTPUT);
	gpio_pin_write(STANDBY_GPIO_BASE, STANDBY_GPIO_PIN, STANDBY_GPIO_PIN);
	motor_halt();
}

void motor_set_duty_cycle(motor_t motor, int8_t duty_cycle) {
	const motor_direction_t new_dir =
			(duty_cycle > 0) ? FORWARD_DIRECTION : REVERSE_DIRECTION;
	const bool dir_change = (new_dir != directions_[motor]);
	if (dir_change) {
		set_direction(motor, new_dir);
		directions_[motor] = new_dir;
	}
	const int8_t clamped_dc = clamp(abs(duty_cycle), 0, MAX_DC_MAG);
	pwm_set_duty_cycle(MOTOR_PWMS[motor], clamped_dc);
}

int8_t motor_get_duty_cycle(motor_t motor) {
	if (directions_[motor] == FORWARD_DIRECTION) {
		return pwm_get_duty_cycle(MOTOR_PWMS[motor]);
	} else {
		return -pwm_get_duty_cycle(MOTOR_PWMS[motor]);
	}
}

void motor_halt(void) {
	// pull both in pins low.
	gpio_pin_write(MOTOR_IN_GPIO_BASE,
			(MOTOR_IN_GPIO_PINS[LEFT_MOTOR][0] | MOTOR_IN_GPIO_PINS[RIGHT_MOTOR][0]
					| MOTOR_IN_GPIO_PINS[LEFT_MOTOR][1]
					| MOTOR_IN_GPIO_PINS[RIGHT_MOTOR][1]), 0);
	pwm_set_duty_cycle(MOTOR_PWMS[LEFT_MOTOR], 0);
	pwm_set_duty_cycle(MOTOR_PWMS[RIGHT_MOTOR], 0);
	directions_[LEFT_MOTOR] = HALTED;
	directions_[RIGHT_MOTOR] = HALTED;
}
