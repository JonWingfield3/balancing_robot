/*
 * led.c
 *
 *  Created on: Oct 14, 2017
 *      Author: jonathanwingfield
 */

#include "../include/led.h"

#include <stdbool.h>
#include <stdint.h>

#include "../include/balancing_robot.h"
#include "../include/gpio.h"
#include "../include/scheduler.h"
#include "../include/utils.h"

#define LED_GPIO_BASE (GPIO_PORT0)
#define LED_ALL_MASK (PIN(7) | PIN(8) | PIN(9))

#define LED_RED_MASK (PIN(7))
#define LED_GREEN_MASK (PIN(8))
#define LED_BLUE_MASK (PIN(9))
#define LED_WHITE_MASK (LED_RED_MASK | LED_BLUE_MASK | LED_GREEN_MASK)
#define LED_PURPLE_MASK (LED_RED_MASK | LED_BLUE_MASK)
#define LED_YELLOW_MASK (LED_RED_MASK | LED_GREEN_MASK)
#define LED_TEAL_MASK (LED_BLUE_MASK | LED_GREEN_MASK)

static const uint16_t led_masks_[NUM_LEDS] = { LED_RED_MASK, LED_GREEN_MASK,
LED_BLUE_MASK, LED_WHITE_MASK, LED_PURPLE_MASK, LED_TEAL_MASK,
LED_YELLOW_MASK };

static led_t heartbeat_led_;

static int led_heartbeat_handler(void) {
	static bool heartbeat_led_on = false;
	led_set_state(heartbeat_led_, heartbeat_led_on);
	heartbeat_led_on = !heartbeat_led_on;
	return 0;
}

static void led_heartbeat_init(led_t heartbeat_led, uint32_t periodicity) {
	heartbeat_led_ = heartbeat_led;
	scheduler_init_task(LED_HEARTBEAT_HANDLER, led_heartbeat_handler,
			periodicity);
}

void led_init(void) {
	gpio_pin_configure(LED_GPIO_BASE, LED_ALL_MASK, GPIO_OUTPUT);
	gpio_pin_write(LED_GPIO_BASE, LED_ALL_MASK, LED_ALL_MASK);
	led_heartbeat_init(GREEN_LED, SCHEDULER_FREQUENCY / 2);
}

void led_set_state(led_t led, bool on) {
	gpio_pin_write(LED_GPIO_BASE, LED_ALL_MASK,
			on ? ~led_masks_[led] : LED_ALL_MASK);
}

void led_set_heartbeat_color(led_t led) {
	heartbeat_led_ = led;
}
