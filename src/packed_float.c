/*
 * packed_float.c
 *
 *  Created on: Nov 5, 2017
 *      Author: jonathanwingfield
 */

#include "../include/packed_float.h"

#include "../include/utils.h"

void pack_float(float val, uint8_t* data_array, float min_val, float max_val,
		uint32_t num_bytes) {
	const uint32_t scaler = (1 << (num_bytes << 3)) - 1;
	const float clamped_val = fclamp(val, min_val, max_val);
	const float range = max_val - min_val;
	const uint32_t packed_float = ((clamped_val - min_val) / range) * scaler;
	memcopy((const uint8_t*) &packed_float, data_array, num_bytes);
}

float unpack_float(const uint8_t* data_array, float min_val, float max_val,
		uint32_t num_bytes) {
	const uint32_t scaler = (1 << (num_bytes << 3)) - 1;
	const float range = max_val - min_val;
	uint32_t packed_float = 0;
	memcopy(data_array, (uint8_t*) &packed_float, num_bytes);
	const float original_float = ((((float) packed_float) / ((float) scaler))
			* range) + min_val;
	return original_float;
}
