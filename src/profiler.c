/*
 * profiler.c
 *
 *  Created on: Nov 8, 2017
 *      Author: jonathanwingfield
 */

#include <profiler.h>

#include <stdint.h>

#include <LPC11xx.h>
#include <system_LPC11xx.h>

#include <scheduler.h>
#include <utils.h>

static uint32_t start_ticks_;

#define PROFILER_TMR (LPC_TMR32B0)

void profiler_init(void) {
	LPC_SYSCON->SYSAHBCLKCTRL |= BIT(9);
	// Divide core frequency by 1M to achieve 1 us resolution.
	PROFILER_TMR->PR = (uint32_t) (((float) SystemCoreClock / 1000000.0) - 1);
	// enable timer/counter for counting
	PROFILER_TMR->TCR = BIT(0);
}

void profiler_start_measurement(void) {
	start_ticks_ = PROFILER_TMR->TC;
}

uint32_t profiler_stop_measurement(void) {
	const uint32_t current_ticks = PROFILER_TMR->TC;
	if (current_ticks < start_ticks_) {
		// overflow has occurred during profiler measurement
		return ((1 << 16) - start_ticks_) + current_ticks;
	} else {
		return current_ticks - start_ticks_;
	}
}

