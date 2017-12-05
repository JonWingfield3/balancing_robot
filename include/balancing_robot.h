/*
 * lab5.h
 *
 *  Created on: Oct 14, 2017
 *      Author: jonathanwingfield
 */

#ifndef BALANCING_ROBOT_H_
#define BALANCING_ROBOT_H_

#include <stdbool.h>

#ifndef M_PI
#define M_PI (3.141592653589793)
#endif

void system_init(void);
void system_reset(void);
bool system_panicked(void);

#endif /* BALANCING_ROBOT_H_ */
