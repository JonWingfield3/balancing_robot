/*
 * compression.h
 *
 *  Created on: Nov 5, 2017
 *      Author: jonathanwingfield
 */

#ifndef COMPRESSION_H_
#define COMPRESSION_H_

#include <stdint.h>

void compress(float val, uint8_t* data_array, float min_val, float max_val,
		uint32_t num_bytes);

float decompress(const uint8_t* data_array, float min_val, float max_val,
		uint32_t num_bytes);

#endif /* COMPRESSION_H_ */
