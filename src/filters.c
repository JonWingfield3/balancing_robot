/*
 * filter.c
 *
 *  Created on: Nov 14, 2017
 *      Author: jonathanwingfield
 */

#include "../include/filters.h"

#include <math.h>
#include <stdint.h>
#include <stdlib.h>

#include "../include/utils.h"

#ifndef M_PI
#define M_PI (3.141592653589793)
#endif

void fir_filter_init(fir_filter_t* fir_filter, float sampling_frequency,
		float cutoff_frequency, float filter_order) {
	fir_filter->f_c_norm = cutoff_frequency / sampling_frequency;
	fir_filter->taps = (float*) malloc((filter_order + 1) * sizeof(float));
	fir_filter->samples = (float*) malloc((filter_order + 1) * sizeof(float));
	fir_filter->order = filter_order;
	fir_filter->write_index = 0;
	float K = 0.0; // used to normalize taps
	for (int i = 0; i < (filter_order + 1); ++i) {
		if (i == filter_order / 2) {
			fir_filter->taps[i] = 2 * M_PI * fir_filter->f_c_norm;
		} else {
			fir_filter->taps[i] = ((sinf(
					2 * M_PI * fir_filter->f_c_norm * (i - (filter_order / 2)))
					/ (i - (filter_order / 2)))
					* (0.54 - 0.46 * cosf(2 * M_PI * i / filter_order)));
		}
		K += fir_filter->taps[i];
	}

	for (int i = 0; i < (filter_order + 1); ++i) {
		fir_filter->taps[i] /= K;
		fir_filter->samples[i] = 0.0;
	}
}

void fir_filter_add_sample(fir_filter_t* fir_filter, float new_sample) {
	fir_filter->samples[fir_filter->write_index] = new_sample;
	fir_filter->write_index = (fir_filter->write_index + 1)
			% (fir_filter->order + 1);
}

float fir_filter_get_output(fir_filter_t* fir_filter) {
	float fir_output = 0.0;
	for (int i = 0; i <= fir_filter->order; ++i) {
		fir_output += (fir_filter->taps[i]
				* fir_filter->samples[(fir_filter->write_index + i)
						% (fir_filter->order + 1)]);
	}
	return fir_output;
}

void fir_filter_deinit(fir_filter_t* fir_filter) {
	free((void*) fir_filter->taps);
	free((void*) fir_filter->samples);
}

void moving_avg_filter_init(moving_avg_filter_t* moving_avg_filter, uint32_t N) {
	moving_avg_filter->samples = (float*) malloc(N * sizeof(float));
	moving_avg_filter->N = N;
	for (int i = 0; i < N; ++i) {
		moving_avg_filter->samples[i] = 0.0;
	}
	moving_avg_filter->average = 0.0;
}

void moving_avg_filter_add_sample(moving_avg_filter_t* moving_avg_filter,
		float new_sample) {
	const float oldest_sample =
			moving_avg_filter->samples[moving_avg_filter->write_index];
	moving_avg_filter->write_index = (moving_avg_filter->write_index + 1)
			% moving_avg_filter->N;
	moving_avg_filter->samples[moving_avg_filter->write_index] = new_sample;
	moving_avg_filter->average += (new_sample - oldest_sample)
			/ moving_avg_filter->N;
}

void moving_avg_filter_deinit(moving_avg_filter_t* moving_avg_filter) {
	free((void*) moving_avg_filter->samples);
}

void hysteresis_filter_init(hysteresis_filter_t* hysteresis_filter,
		float low_threshold, float high_threshold, hysteresis_state_t initial_state) {
	hysteresis_filter->high_threshold = high_threshold;
	hysteresis_filter->low_threshold = low_threshold;
	hysteresis_filter->state = initial_state;
}

void hysteresis_filter_add_sample(hysteresis_filter_t* hysteresis_filter,
		float new_sample) {
	if ((hysteresis_filter->state == HYSTERESIS_HIGH_STATE)
			&& (new_sample <= hysteresis_filter->low_threshold)) {
		hysteresis_filter->state = HYSTERESIS_LOW_STATE;
	} else if ((hysteresis_filter->state == HYSTERESIS_LOW_STATE)
			&& (new_sample >= hysteresis_filter->high_threshold)) {
		hysteresis_filter->state = HYSTERESIS_HIGH_STATE;
	}
}

float hysteresis_filter_get_output(hysteresis_filter_t* hysteresis_filter) {
	return
			(hysteresis_filter->state == HYSTERESIS_HIGH_STATE) ?
					hysteresis_filter->high_threshold : hysteresis_filter->low_threshold;
}

