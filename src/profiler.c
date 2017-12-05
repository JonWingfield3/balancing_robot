/*
 * profiler.c
 *
 *  Created on: Nov 8, 2017
 *      Author: jonathanwingfield
 */

#include <profiler.h>

#include <stdint.h>

#include <LPC11xx.h>

#include <scheduler.h>
#include <utils.h>

static uint32_t start_ticks_;

#define PROFILER_TMR (LPC_TMR32B0)

void profiler_init(void) {
	LPC_SYSCON->SYSAHBCLKCTRL |= BIT(9);
	// enable timer/counter for counting
	PROFILER_TMR->PR = 48; // system clk is 48 MHz. want 1 us resolution.
	PROFILER_TMR->TCR = BIT(0);
}

void profiler_start_measurement(void) {
	start_ticks_ = PROFILER_TMR->TC;
}

uint32_t profiler_stop_measurement(void) {
	const uint32_t current_ticks = PROFILER_TMR->TC;
	if (current_ticks < start_ticks_) {
		// overflow has occurred during profiler measurement
		return ((1 << 31) - start_ticks_) + current_ticks;
	} else {
		return current_ticks - start_ticks_;
	}
}

