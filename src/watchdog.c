/*
 * watchdog.c
 *
 *  Created on: Nov 9, 2017
 *      Author: jonathanwingfield
 */

#include "../include/watchdog.h"

#include "../include/LPC11xx.h"
#include "../include/utils.h"

void watchdog_init(void) {
	LPC_SYSCON->SYSAHBCLKCTRL |= BIT(15);
	// use main clock for watchdog
	LPC_SYSCON->WDTCLKSEL = 0x1;
	// 8 bit div val
	LPC_SYSCON->WDTCLKDIV = 0x1;
	LPC_SYSCON->WDTCLKUEN = 0x00;
	LPC_SYSCON->WDTCLKUEN = 0x01;

//	LPC_WDT->MOD = BIT(1) |
}

void watchdog_feed(void) {
	// feed sequence as defined in TRM.
	LPC_WDT->FEED = 0xAA;
	LPC_WDT->FEED = 0x55;
}
