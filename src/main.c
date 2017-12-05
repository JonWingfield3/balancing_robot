/*
 ===============================================================================
 Name        : main.c
 Author      : jon wingfield
 ===============================================================================
 */

#include <stdbool.h>

#include "../include/balancing_robot.h"
#include "../include/gpio.h"
#include "../include/interrupts.h"
#include "../include/led.h"
#include "../include/message_service.h"
#include "../include/motor_controller.h"
#include "../include/profiler.h"
#include "../include/scheduler.h"

int main(void) {
	disable_interrupts();
	gpio_init();
	scheduler_init();
	profiler_init();
	message_service_init();
	led_init();
	enable_interrupts();
	motor_controller_init();

	while (true) {
		scheduler_run();
	}
}
