/*
 * led.h
 *
 *  Created on: Oct 14, 2017
 *      Author: jonathanwingfield
 */

#ifndef LED_H_
#define LED_H_

#include <stdbool.h>
#include <stdint.h>

typedef enum {
	LED_FIRST,
	RED_LED = LED_FIRST,
	GREEN_LED,
	BLUE_LED,
	WHITE_LED,
	PURPLE_LED,
	TEAL_LED,
	YELLOW_LED,
	NUM_LEDS,
} led_t;

void led_init(void);
void led_set_state(led_t led, bool on);

void led_heartbeat_init(led_t heartbeat_led, uint32_t heartbeat_frequency);
void led_set_heartbeat_color(led_t led);
void led_set_heartbeat_frequency(uint32_t heatbeat_frequency);

#endif /* LED_H_ */
