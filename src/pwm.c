/*
 * pwm.c
 *
 *  Created on: Oct 29, 2017
 *      Author: jonathanwingfield
 */

#include <pwm.h>

#include <LPC11xx.h>

#include <utils.h>

#define NUM_PWM_PINS (2)
#define PWM_TIMER (LPC_TMR16B1)
#define PWM_FREQ  (5000.0) // H bridge tolerates up to 10 kHz

static uint8_t duty_cycles_[NUM_PWM_PINS];

void pwm_init(void) {
	// enable clock gating for TMR16B1
	LPC_SYSCON->SYSAHBCLKCTRL |= BIT(8);
	// configures P1.9 for MAT0 mode
	LPC_IOCON->PIO1_9 = 1;
	// configures P1.10 for MAT1 mode
	LPC_IOCON->PIO1_10 = 2;

	PWM_TIMER->TCR = BIT(1);
	// Timer clocked by PCLK/(PR+1)
	PWM_TIMER->PR = (uint32_t) ((float) SystemCoreClock / (PWM_FREQ * 100.0));
	// configure as a timer (not a counter).
	PWM_TIMER->CTCR = 0;
	// enable reset on match for MAT2 (100 count val, sets DC freq)
	PWM_TIMER->MCR = BIT(7);
	// enable MAT[0:2] as PWM outputs.
	PWM_TIMER->PWMC = (BIT(0) | BIT(1) | BIT(2));
	// set divider values. use MR2 to set 0-99 count duty cycle.
	PWM_TIMER->MR0 = 100;
	PWM_TIMER->MR1 = 100;
	PWM_TIMER->MR2 = 99;
	PWM_TIMER->TCR = BIT(0);
}

void pwm_set_duty_cycle(pwm_pin_t pwm_pin, uint8_t duty_cycle) {
	const uint8_t clamped_dc = clamp(duty_cycle, 0, 100);
	const uint8_t match_write_val = 100 - clamped_dc;
	if (pwm_pin == 0) {
		PWM_TIMER->MR0 = match_write_val;
	} else {
		PWM_TIMER->MR1 = match_write_val;
	}
	duty_cycles_[pwm_pin] = clamped_dc;
}

uint8_t pwm_get_duty_cycle(pwm_pin_t pwm_pin) {
	return duty_cycles_[pwm_pin];
}

