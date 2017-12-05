/*
 * balancing_robot.c
 *
 *  Created on: Dec 5, 2017
 *      Author: jonathanwingfield
 */

#include <balancing_robot.h>

#include <stdbool.h>
#include <stdint.h>

#include <LPC11xx.h>
#include <system_LPC11xx.h>

#include <led.h>
#include <motor_controller.h>
#include <scheduler.h>

static bool system_panicked_;

static int system_panic(void) {
	motor_controller_enable(false);
	led_set_heartbeat_color(RED_LED);
	system_panicked_ = true;
	return 0;
}

bool system_panicked(void) {
	return system_panicked_;
}

void system_reset(void) {
	NVIC_SystemReset();
}

void system_init(void) {
	scheduler_init_task(SYSTEM_PANIC, system_panic, 0);
	system_panicked_ = false;
}
