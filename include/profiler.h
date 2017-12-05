/*
 * profiler.h
 *
 *  Created on: Nov 8, 2017
 *      Author: jonathanwingfield
 */

#ifndef PROFILER_H_
#define PROFILER_H_

#include <stdint.h>

void profiler_init(void);
void profiler_start_measurement(void);
uint32_t profiler_stop_measurement(void);

#endif /* PROFILER_H_ */
