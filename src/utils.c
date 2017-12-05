/*
 * utils.c
 *
 *  Created on: Oct 29, 2017
 *      Author: jonathanwingfield
 */

#include "../include/utils.h"

#include <stdbool.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>

int clamp(int32_t value, int32_t min, int32_t max) {
	if (value < min) {
		return min;
	}
	if (value > max) {
		return max;
	}
	return value;
}

float fclamp(float value, float min, float max) {
	if (value < min) {
		return min;
	}
	if (value > max) {
		return max;
	}
	return value;
}

int memcopy(const uint8_t* src, uint8_t* dst, uint32_t n) {
	for (uint32_t i = 0; i < n; ++i) {
		dst[i] = src[i];
	}
	return n;
}

// http://en.wikipedia.org/wiki/Fast_inverse_square_root
float fast_inv_sqrt(float x) {
	const float halfx = 0.5f * x;
	float y = x;
	long i = *(long*) &y;
	i = 0x5f3759df - (i >> 1);
	y = *(float*) &i;
	y = y * (1.5f - (halfx * y * y));
	return y;
}
