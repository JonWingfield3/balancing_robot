/*
 * uart.h
 *
 *  Created on: Oct 12, 2017
 *      Author: jonathanwingfield
 */

#ifndef UART_H_
#define UART_H_

#include <stdint.h>

#define BAUD_RATE (115200)

void uart_init(void);
void uart_deinit(void);

void uart_write(const uint8_t* buf, uint32_t n);
void uart_write_byte(uint8_t byte);

void uart_read(uint8_t* buf, uint32_t n);
uint8_t uart_peek(void);
int uart_available(void);

void UART_IRQHandler(void);

#endif /* UART_H_ */
