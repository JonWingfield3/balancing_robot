/*
 * gpio.c
 *
 *  Created on: Nov 7, 2017
 *      Author: jonathanwingfield
 */

#include "../include/gpio.h"

#include <stdint.h>

#include "../include/LPC11xx.h"
#include "../include/utils.h"

static const uint32_t gpio_ports_[NUM_GPIO_PORTS] = {
LPC_GPIO0_BASE, LPC_GPIO1_BASE, LPC_GPIO2_BASE, LPC_GPIO3_BASE };

void gpio_init(void) {
	LPC_SYSCON->SYSAHBCLKCTRL |= BIT(6);
}

void gpio_pin_write(gpio_port_t gpio_port, uint16_t write_mask,
		uint16_t write_val) {
	const uint16_t masked_state =
			(((LPC_GPIO_TypeDef*) gpio_ports_[gpio_port])->DATA & ~write_mask);
	((LPC_GPIO_TypeDef*) gpio_ports_[gpio_port])->DATA = (masked_state
			| (write_mask & write_val));
}

uint16_t gpio_pin_read(gpio_port_t gpio_port, uint16_t read_mask) {
	return (((LPC_GPIO_TypeDef*) gpio_ports_[gpio_port])->DATA & read_mask);
}

void gpio_pin_configure(gpio_port_t gpio_port, uint16_t gpio_pin_mask,
		gpio_direction_t direction) {
	if (direction == GPIO_INPUT) {
		((LPC_GPIO_TypeDef*) gpio_ports_[gpio_port])->DIR &= ~gpio_pin_mask;
	} else {
		((LPC_GPIO_TypeDef*) gpio_ports_[gpio_port])->DIR |= gpio_pin_mask;
	}
}
