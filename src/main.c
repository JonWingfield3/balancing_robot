/*
 ===============================================================================
 Name        : main.c
 Author      : Jon Wingfield
 ===============================================================================
 */

#include <stdbool.h>

#include <balancing_robot.h>
#include <gpio.h>
#include <interrupts.h>
#include <message_service.h>
#include <motor_controller.h>
#include <profiler.h>
#include <scheduler.h>

int main(void) {
	disable_interrupts();
	gpio_init();
	scheduler_init();
	system_init();
	message_service_init();
	enable_interrupts();
	motor_controller_init();

	while (true) {
		scheduler_run();
	}
}
