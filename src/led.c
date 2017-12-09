/*
 * led.c
 *
 *  Created on: Oct 14, 2017
 *      Author: jonathanwingfield
 */

#include <led.h>

#include <stdbool.h>
#include <stdint.h>

#include <balancing_robot.h>
#include <gpio.h>
#include <scheduler.h>
#include <utils.h>

static const gpio_port_t LED_GPIO_BASE = GPIO_PORT0;
static const uint16_t LED_ALL_MASK = (PIN(7) | PIN(8) | PIN(9));

#define LED_RED_MASK (PIN(7))
#define LED_GREEN_MASK (PIN(8))
#define LED_BLUE_MASK (PIN(9))
#define LED_WHITE_MASK (LED_RED_MASK | LED_BLUE_MASK | LED_GREEN_MASK)
#define LED_PURPLE_MASK (LED_RED_MASK | LED_BLUE_MASK)
#define LED_YELLOW_MASK (LED_RED_MASK | LED_GREEN_MASK)
#define LED_TEAL_MASK (LED_BLUE_MASK | LED_GREEN_MASK)

static const uint16_t LED_MASKS[NUM_LEDS] = { LED_RED_MASK, LED_GREEN_MASK,
LED_BLUE_MASK, LED_WHITE_MASK, LED_PURPLE_MASK, LED_TEAL_MASK,
LED_YELLOW_MASK };

static led_t heartbeat_led_;
static bool heartbeat_enabled_;

static int led_heartbeat_handler(void) {
	static bool heartbeat_led_on = false;
	led_set_state(heartbeat_led_, heartbeat_led_on);
	heartbeat_led_on = !heartbeat_led_on;
	return 0;
}

void led_init(void) {
	gpio_pin_configure(LED_GPIO_BASE, LED_ALL_MASK, GPIO_OUTPUT);
	gpio_pin_write(LED_GPIO_BASE, LED_ALL_MASK, LED_ALL_MASK);
	heartbeat_enabled_ = false;
}

void led_set_state(led_t led, bool on) {
	gpio_pin_write(LED_GPIO_BASE, LED_ALL_MASK,
			on ? ~LED_MASKS[led] : LED_ALL_MASK);
}

void led_heartbeat_init(led_t heartbeat_led, uint32_t heartbeat_frequency) {
	heartbeat_led_ = heartbeat_led;
	const uint32_t led_heartbeat_periodicity =
			(uint32_t) (((float) SCHEDULER_FREQUENCY) / ((float) heartbeat_frequency));
	scheduler_init_task(LED_HEARTBEAT_HANDLER, led_heartbeat_handler,
			led_heartbeat_periodicity);
	heartbeat_enabled_ = true;
}

void led_set_heartbeat_color(led_t led) {
	heartbeat_led_ = led;
}

void led_set_heartbeat_frequency(uint32_t heartbeat_frequency) {
	const uint32_t led_heartbeat_periodicity =
			(uint32_t) (((float) SCHEDULER_FREQUENCY) / ((float) heartbeat_frequency));
	scheduler_set_task_periodicity(LED_HEARTBEAT_HANDLER,
			led_heartbeat_periodicity);
}
