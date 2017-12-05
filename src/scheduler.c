/*
 * scheduler.c
 *
 *  Created on: Nov 7, 2017
 *      Author: jonathanwingfield
 */

#include <scheduler.h>

#include <stdbool.h>
#include <stdint.h>
#include <stdlib.h>

#include <balancing_robot.h>
#include <interrupts.h>
#include <led.h>
#include <LPC11xx.h>
#include <motor_controller.h>
#include <profiler.h>
#include <utils.h>

#define NULL_CALLBACK (0)

typedef struct {
	task_callback_t task_callback;
	uint32_t periodicity;
	uint32_t ticks_til_next_call;
} task_t;

static volatile task_t tasks_[NUM_TASKS];
static volatile uint32_t enabled_task_mask_;
static volatile uint32_t pending_task_mask_;

void TIMER16_0_IRQHandler(void) {
	// clear the MR0 interrupt by writing 1 to register
	LPC_TMR16B0->IR |= BIT(0);
	// The loop decrements a tasks's timer only if task is enabled,
	// and it is either periodic or has a non-zero timer associated with it,
	// We set the task as pending if timer has expired. A periodic task's
	// timer will be reset after it's callback has successfully ran.
	for (int task_idx = TASK_FIRST; task_idx < NUM_TASKS; ++task_idx) {
		if ((enabled_task_mask_ & BIT(task_idx))
				&& ((tasks_[task_idx].periodicity != 0)
						|| (tasks_[task_idx].ticks_til_next_call > 0))
				&& (--tasks_[task_idx].ticks_til_next_call == 0)) {
			pending_task_mask_ |= BIT(task_idx);
		}
	}
}

void scheduler_init(void) {
	for (int task_idx = TASK_FIRST; task_idx < NUM_TASKS; ++task_idx) {
		tasks_[task_idx].task_callback = NULL_CALLBACK;
	}
	enabled_task_mask_ = 0;
	pending_task_mask_ = 0;

	// enable clock gating for 16 bit timer 0
	LPC_SYSCON->SYSAHBCLKCTRL |= BIT(7);
	// enable interrupts upon matching with MR0, reset TC when match occurs.
	LPC_TMR16B0->MCR = BIT(0) | BIT(1);
	// set PR so that PC counts microseconds. TC counts in milliseconds.
	LPC_TMR16B0->PR = (SystemCoreClock / 1000000 - 1);
	LPC_TMR16B0->MR0 = (1000000 / SCHEDULER_FREQUENCY - 1);
	// reset counter and enable timer/counter for counting
	LPC_TMR16B0->TC = 0;
	LPC_TMR16B0->TCR = BIT(0);
	// enable interrupts from timer in NVIC. Set to medium priority
	NVIC_EnableIRQ(TIMER_16_0_IRQn);
	NVIC_SetPriority(TIMER_16_0_IRQn, 2);
}

void scheduler_run(void) {
	for (int task_idx = TASK_FIRST; task_idx < NUM_TASKS; ++task_idx) {
		if ((enabled_task_mask_ & BIT(task_idx)) & pending_task_mask_) {

			pending_task_mask_ &= ~BIT(task_idx);
			const int task_ret_val = tasks_[task_idx].task_callback();
			if (task_ret_val == -1) {
				scheduler_disable_task(task_idx);
			}
			if (tasks_[task_idx].periodicity != 0) {
				tasks_[task_idx].ticks_til_next_call = tasks_[task_idx].periodicity;
			}

		}
	}
}

int scheduler_set_task_periodicity(task_id_t task_id, uint32_t periodicity) {
	int ret_val = 0;
	if (enabled_task_mask_ & BIT(task_id)) {
		const bool int_state = start_critical_section();
		tasks_[task_id].periodicity = periodicity;
		end_critical_section(int_state);
	} else {
		ret_val = -1;
	}
	return ret_val;
}

int scheduler_init_task(task_id_t task_id, task_callback_t task_callback,
		uint32_t periodicity) {
	int ret_val = 0;
	if (enabled_task_mask_ & BIT(task_id)) {
		ret_val = 1;
	}
	const bool int_state = start_critical_section();
	tasks_[task_id].task_callback = task_callback;
	tasks_[task_id].periodicity = periodicity;
	tasks_[task_id].ticks_til_next_call = periodicity;
	pending_task_mask_ &= ~BIT(task_id);
	enabled_task_mask_ |= BIT(task_id);
	end_critical_section(int_state);
	return ret_val;
}

int scheduler_disable_task(task_id_t task_id) {
	int ret_val = 0;
	if (enabled_task_mask_ & BIT(task_id)) {
		const bool int_state = start_critical_section();
		enabled_task_mask_ &= ~BIT(task_id);
		end_critical_section(int_state);
	} else {
		ret_val = -1;
	}
	return ret_val;
}

int scheduler_set_pending(task_id_t task_id) {
	int ret_val = 0;
	if (enabled_task_mask_ & BIT(task_id)) {
		const bool int_state = start_critical_section();
		pending_task_mask_ |= BIT(task_id);
		end_critical_section(int_state);
	} else {
		ret_val = -1;
	}
	return ret_val;
}

int scheduler_set_pending_in(task_id_t task_id, uint32_t ticks_til_pending) {
	int ret_val = 0;
	if (enabled_task_mask_ & BIT(task_id)) {
		const bool int_state = start_critical_section();
		tasks_[task_id].ticks_til_next_call = ticks_til_pending;
		end_critical_section(int_state);
	} else {
		ret_val = -1;
	}
	return ret_val;
}

int scheduler_reset_task_timer(task_id_t task_id) {
	int ret_val = 0;
	if (enabled_task_mask_ & BIT(task_id)) {
		const bool int_state = start_critical_section();
		tasks_[task_id].ticks_til_next_call = tasks_[task_id].periodicity;
		end_critical_section(int_state);
	} else {
		ret_val = -1;
	}
	return ret_val;
}

uint32_t scheduler_get_system_time_ms(void) {
	return LPC_TMR16B0->TC;
}

void scheduler_delay_ms(uint32_t ms) {
	float system_time_ms = LPC_TMR16B0->TC + (LPC_TMR16B0->PC / 1000.0);
	const float system_stop_time_ms = system_time_ms + ms;
	while (system_time_ms < system_stop_time_ms) {
		system_time_ms = LPC_TMR16B0->TC + (LPC_TMR16B0->PC / 1000.0);
	}
}

void scheduler_delay_us(uint32_t us) {
	float system_time_ms = LPC_TMR16B0->TC + (LPC_TMR16B0->PC / 1000.0);
	const float system_stop_time_ms = system_time_ms + (us / 1000.0);
	while (system_time_ms < system_stop_time_ms) {
		system_time_ms = LPC_TMR16B0->TC + (LPC_TMR16B0->PC / 1000.0);
	}
}
