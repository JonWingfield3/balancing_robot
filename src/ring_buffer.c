/*
 * ring_buffer.c
 *
 *  Created on: Oct 27, 2017
 *      Author: jonathanwingfield
 */

#include <ring_buffer.h>

#include <stdbool.h>
#include <stdint.h>
#include <stdlib.h>

#include <interrupts.h>

void ring_buffer_init(volatile ring_buffer_t* ring_buf, uint32_t size) {
	ring_buf->buffer = (uint8_t*) malloc(size);
	ring_buf->size = size;
	ring_buf->write_index = 0;
	ring_buf->read_index = 0;
	ring_buf->count = 0;
}

void ring_buffer_deinit(volatile ring_buffer_t* ring_buf) {
	free((void*) ring_buf->buffer);
	ring_buf->size = 0;
	ring_buf->write_index = 0;
	ring_buf->read_index = 0;
	ring_buf->count = 0;
}

void ring_buffer_write(volatile ring_buffer_t* ring_buf, uint8_t data) {
	const bool int_state = start_critical_section();
	ring_buf->buffer[ring_buf->write_index] = data;
	ring_buf->write_index = (ring_buf->write_index + 1) % ring_buf->size;
	ring_buf->count++;
	end_critical_section(int_state);
}

uint8_t ring_buffer_read(volatile ring_buffer_t* ring_buf) {
	const bool int_state = start_critical_section();
	const uint8_t read_data = ring_buf->buffer[ring_buf->read_index];
	if (ring_buf->count) {
		--ring_buf->count;
	}
	ring_buf->read_index = (ring_buf->read_index + 1) % ring_buf->size;
	end_critical_section(int_state);
	return read_data;
}

uint8_t ring_buffer_peek(volatile ring_buffer_t* ring_buf) {
	const bool int_state = start_critical_section();
	const uint8_t read_data = ring_buf->buffer[ring_buf->read_index];
	end_critical_section(int_state);
	return read_data;
}

bool ring_buffer_empty(volatile ring_buffer_t* ring_buf) {
	const bool int_state = start_critical_section();
	const bool buf_empty = (ring_buf->count == 0);
	end_critical_section(int_state);
	return buf_empty;
}

bool ring_buffer_full(volatile ring_buffer_t* ring_buf) {
	const bool int_state = start_critical_section();
	const bool buf_full = (ring_buf->count == ring_buf->size);
	end_critical_section(int_state);
	return buf_full;
}

uint32_t ring_buffer_available(volatile ring_buffer_t* ring_buf) {
	const bool int_state = start_critical_section();
	const uint32_t data_bytes_available = ring_buf->count;
	end_critical_section(int_state);
	return data_bytes_available;
}

uint32_t ring_buffer_remaining(volatile ring_buffer_t* ring_buf) {
	const bool int_state = start_critical_section();
	const uint32_t open_spots = (ring_buf->size - ring_buf->count);
	end_critical_section(int_state);
	return open_spots;
}
