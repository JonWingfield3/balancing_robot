/*
 * gpio.h
 *
 *  Created on: Nov 7, 2017
 *      Author: jonathanwingfield
 */

#ifndef GPIO_H_
#define GPIO_H_

#include <stdint.h>

typedef enum {
	GPIO_INPUT = 0, GPIO_OUTPUT = 1
} gpio_direction_t;

typedef enum {
	GPIO_PORT0, GPIO_PORT1, GPIO_PORT2, GPIO_PORT3, NUM_GPIO_PORTS
} gpio_port_t;

void gpio_init(void);

void gpio_pin_write(gpio_port_t gpio_port, uint16_t write_mask,
		uint16_t write_val);

uint16_t gpio_pin_read(gpio_port_t gpio_port, uint16_t read_mask);

void gpio_pin_configure(gpio_port_t gpio_port, uint16_t gpio_pin_mask,
		gpio_direction_t direction);

#endif /* GPIO_H_ */
