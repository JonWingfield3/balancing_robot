/*
 * packed_float.h
 *
 *  Created on: Nov 5, 2017
 *      Author: jonathanwingfield
 */

#ifndef PACKED_FLOAT_H_
#define PACKED_FLOAT_H_

#include <stdint.h>

void pack_float(float val, uint8_t* data_array, float min_val, float max_val,
		uint32_t num_bytes);

float unpack_float(const uint8_t* data_array, float min_val, float max_val,
		uint32_t num_bytes);

#endif /* PACKED_FLOAT_H_ */
