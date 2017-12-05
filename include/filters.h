/*
 * filters.h
 *
 *  Created on: Nov 14, 2017
 *      Author: jonathanwingfield
 */

#ifndef FILTERS_H_
#define FILTERS_H_

#include <stdint.h>

typedef struct {
	float f_c_norm;
	float* taps;
	float* samples;
	uint32_t order;
	uint32_t write_index;
} fir_filter_t;

void fir_filter_init(fir_filter_t* fir_filter, float sampling_frequency,
		float cutoff_frequency, float filter_order);
void fir_filter_add_sample(fir_filter_t* fir_filter, float new_sample);
float fir_filter_get_output(fir_filter_t* fir_filter);
void fir_filter_deinit(fir_filter_t* fir_filter);

typedef struct {
	float* samples;
	float average;
	uint32_t N;
	uint32_t write_index;
} moving_avg_filter_t;

void moving_avg_filter_init(moving_avg_filter_t* moving_avg_filter, uint32_t N);
void moving_avg_filter_add_sample(moving_avg_filter_t* moving_avg_filter,
		float new_sample);
void moving_avg_filter_deinit(moving_avg_filter_t* moving_avg_filter);

typedef enum {
	HYSTERESIS_LOW_STATE, HYSTERESIS_HIGH_STATE
} hysteresis_state_t;

typedef struct {
	float low_threshold;
	float high_threshold;
	hysteresis_state_t state;
} hysteresis_filter_t;

void hysteresis_filter_init(hysteresis_filter_t* hysteresis_filter,
		float low_threshold, float high_threshold, hysteresis_state_t initial_state);
void hysteresis_filter_add_sample(hysteresis_filter_t* hysteresis_filter,
		float new_sample);
float hysteresis_filter_get_output(hysteresis_filter_t* hysteresis_filter);

#endif /* FILTERS_H_ */
