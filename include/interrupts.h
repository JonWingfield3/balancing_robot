/*
 * interrupts.h
 *
 *  Created on: Oct 29, 2017
 *      Author: jonathanwingfield
 */

#ifndef INTERRUPTS_H_
#define INTERRUPTS_H_

#include <stdbool.h>

void enable_interrupts(void);
void disable_interrupts(void);

bool start_critical_section(void);
void end_critical_section(bool enable_ints);

#endif /* INTERRUPTS_H_ */
