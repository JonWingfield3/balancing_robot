/*
 * utils.h
 *
 *  Created on: Oct 12, 2017
 *      Author: jonathanwingfield
 */

#ifndef UTILS_H_
#define UTILS_H_

#include <stdbool.h>
#include <stdint.h>

#define BIT(x) (((uint32_t)(0x1 << (x))))
#define PIN(x) ((uint16_t)(BIT((x))))
#define ARRAY_SIZE(array) (sizeof((array))/sizeof((array[0])))
#define PACKED __attribute__((__packed__))
#define in_range(val, min, max) (((val) >= (min)) && ((val) <= (max)))

int clamp(int32_t value, int32_t min, int32_t max);
float fclamp(float value, float min, float max);

int memcopy(const uint8_t* src, uint8_t* dst, uint32_t n);

float fast_inv_sqrt(float x);

#endif /* UTILS_H_ */
