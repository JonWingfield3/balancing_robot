/*
 * ring_buffer.h
 *
 *  Created on: Oct 27, 2017
 *      Author: jonathanwingfield
 */

#ifndef RING_BUFFER_H_
#define RING_BUFFER_H_

#include <stdbool.h>
#include <stdint.h>

typedef struct {
	uint32_t write_index;
	uint32_t read_index;
	uint32_t count;
	uint32_t size;
	uint8_t* buffer;
} ring_buffer_t;

void ring_buffer_init(volatile ring_buffer_t* ring_buf, uint32_t size);
void ring_buffer_deinit(volatile ring_buffer_t* ring_buf);

// This function makes no effort to protect users from themselves.
// Always check if a ring buffer is full before performing a write.
void ring_buffer_write(volatile ring_buffer_t* ring_buf, uint8_t data);
bool ring_buffer_full(volatile ring_buffer_t* ring_buf);

// This function makes no effort to protect users from themselves.
// Always check if a ring buffer is empty before performing a read/peek.
uint8_t ring_buffer_read(volatile ring_buffer_t* ring_buf);
uint8_t ring_buffer_peek(volatile ring_buffer_t* ring_buf);
bool ring_buffer_empty(volatile ring_buffer_t* ring_buf);

uint32_t ring_buffer_available(volatile ring_buffer_t* ring_buf);
uint32_t ring_buffer_remaining(volatile ring_buffer_t* ring_buf);

#endif /* RING_BUFFER_H_ */
