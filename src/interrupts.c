/*
 * interrupts.c
 *
 *  Created on: Oct 29, 2017
 *      Author: jonathanwingfield
 */

#include "../include/interrupts.h"

#include "../include/system_LPC11xx.h"
#include "../include/core_cmFunc.h"

static bool ints_enabled_;

void enable_interrupts(void) {
	__enable_irq();
	ints_enabled_ = true;
}

void disable_interrupts(void) {
	__disable_irq();
	ints_enabled_ = false;
}

bool start_critical_section(void) {
	const bool ints_enabled = ints_enabled_;
	disable_interrupts();
	return ints_enabled;
}

void end_critical_section(bool enable_ints) {
	if (enable_ints) {
		enable_interrupts();
	}
}
