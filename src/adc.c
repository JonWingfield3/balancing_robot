/*
 * adc.c
 *
 *  Created on: Oct 14, 2017
 *      Author: jonathanwingfield
 */

#include "../include/adc.h"

#include <stdbool.h>
#include <stdint.h>
#include <stdlib.h>

#include "../include/balancing_robot.h"
#include "../include/LPC11xx.h"
#include "../include/scheduler.h"
#include "../include/utils.h"

static int last_adc_read_;

static int adc_task_callback(void) {
	// check if last sample is done.
	if (LPC_ADC->GDR & BIT(31)) {
		// stop channel.
		LPC_ADC->CR &= 0xF8FFFFFF;
		// read latest sample.
		volatile uint32_t* ch0_data_reg = ((uint32_t*) (LPC_ADC_BASE + 0x10));
		last_adc_read_ = (*ch0_data_reg >> 6) & 0x3FF;
		// sets start bit and tells it to use ad0.
		LPC_ADC->CR |= BIT(24) | BIT(0);
	}
	return 0;
}

void adc_init(void) {
	LPC_SYSCON->PDRUNCFG &= ~BIT(4);
	LPC_SYSCON->SYSAHBCLKCTRL |= BIT(13);
	// ioconfig for adc ch0
	LPC_IOCON->R_PIO0_11 = 2;
	// set up clock at 2.4 MHz
	LPC_ADC->CR = ((SystemCoreClock / LPC_SYSCON->SYSAHBCLKDIV) / 2400000 - 1)
			<< 8;
	// sets start bit and tells it to use ad0.
	LPC_ADC->CR |= BIT(24) | BIT(0);
}

uint16_t adc_read(void) {
	//return (ADC_REF / ADC_RES_SCALE) * last_adc_read_;
	return last_adc_read_;
}
