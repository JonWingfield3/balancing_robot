/*
 * scheduler.h
 *
 *  Created on: Nov 7, 2017
 *      Author: jonathanwingfield
 */

#ifndef SCHEDULER_H_
#define SCHEDULER_H_

#include <stdint.h>

// tasks listed from highest to lowest priority.
typedef enum {
	TASK_FIRST,
	MPU_DATA_COLLECTOR = TASK_FIRST,
	MOTOR_CONTROL_COMMAND_TERMINATOR,
	MOTOR_CONTROL_UPDATER,
	UART_DATA_BYTE_HANDLER,
	MESSAGE_HANDLER,
	LED_HEARTBEAT_HANDLER,
	SYSTEM_BOOKKEEPING_TASK,
	SYSTEM_IDLE_TASK, // should be lowest priority task.
	NUM_TASKS,
} task_id_t;

typedef int (*task_callback_t)(void);

void scheduler_init(void);
int scheduler_init_task(task_id_t task_id, task_callback_t task_callback,
		uint32_t periodicity);

// scheduler task management functions.
int scheduler_disable_task(task_id_t task_id);
int scheduler_set_pending(task_id_t task_id);
int scheduler_set_pending_in(task_id_t task_id, uint32_t ticks_til_pending);
int scheduler_reset_task_timer(task_id_t task_id);
int scheduler_set_task_periodicity(task_id_t task_id, uint32_t periodicity);

void scheduler_run(void);

// scheduler utility functions.
uint32_t scheduler_get_system_time(void);

#endif /* SCHEDULER_H_ */
