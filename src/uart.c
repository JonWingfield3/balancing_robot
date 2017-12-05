/*
 * uart.c
 *
 *  Created on: Oct 12, 2017
 *      Author: jonathanwingfield
 */

#include <uart.h>

#include <stdbool.h>
#include <stdint.h>

#include <LPC11xx.h>

#include <ring_buffer.h>
#include <scheduler.h>
#include <utils.h>

static volatile ring_buffer_t outbound_ring_buf_;
static const uint32_t OUTBOUND_RING_BUF_SIZE = 256;

static volatile ring_buffer_t inbound_ring_buf_;
static const uint32_t INBOUND_RING_BUF_SIZE = 64;

static inline void enable_uart_tx_ints(bool enable) {
	if (enable) {
		LPC_UART->IER |= BIT(1);
	} else {
		LPC_UART->IER &= ~BIT(1);
	}
}

static inline bool uart_tx_ints_enabled(void) {
	return (LPC_UART->IER & BIT(1));
}

static inline bool uart_tx_buf_empty(void) {
	return (LPC_UART->LSR & BIT(5));
}

void uart_init(void) {
	ring_buffer_init(&outbound_ring_buf_, OUTBOUND_RING_BUF_SIZE);
	ring_buffer_init(&inbound_ring_buf_, INBOUND_RING_BUF_SIZE);

	LPC_SYSCON->SYSAHBCLKCTRL |= BIT(6);
	NVIC_DisableIRQ(UART_IRQn);
	// mux gpio pins to uart
	LPC_IOCON->PIO1_6 &= ~0x7;
	LPC_IOCON->PIO1_6 |= 0x1;
	LPC_IOCON->PIO1_7 &= ~0x7;
	LPC_IOCON->PIO1_7 |= 0x1;
	// enable clock gating to uart
	LPC_SYSCON->SYSAHBCLKCTRL |= BIT(12);
	// enable uart clock, set to system core clock freq.
	LPC_SYSCON->UARTCLKDIV = 1;
	// 8N1 uart, enable divisor latches
	LPC_UART->LCR = 0x3 | BIT(7);
	// set up baud rate (needs to be written over two registers)
	const uint32_t scale = (((SystemCoreClock * LPC_SYSCON->SYSAHBCLKDIV)
			/ LPC_SYSCON->UARTCLKDIV) / 16) / BAUD_RATE;
	LPC_UART->DLL = scale & 0xff;
	LPC_UART->DLM = (scale >> 8) & 0xff;
	// disable divisor latches (must do this for uart to work properly).
	LPC_UART->LCR = 0x3;
	// enable fifos and reset them (set to size 1)
	LPC_UART->FCR = 7;
	// enable rx interrupts in uart
	LPC_UART->IER = BIT(0);
	NVIC_EnableIRQ(UART_IRQn);
	NVIC_SetPriority(UART_IRQn, 1);
}

void uart_deinit(void) {
	return;
}

void uart_write(const uint8_t* buf, uint32_t n) {
	uart_write_byte(*buf++);
	const uint32_t clamped_n = clamp((n - 1), 0,
			ring_buffer_remaining(&outbound_ring_buf_));
	for (uint32_t i = 0; i < clamped_n; i++) {
		ring_buffer_write(&outbound_ring_buf_, buf[i]);
	}
}

void uart_write_byte(uint8_t byte) {
	if (ring_buffer_empty(&outbound_ring_buf_) && uart_tx_buf_empty()) {
		LPC_UART->THR = byte;
	} else {
		if (!ring_buffer_full(&outbound_ring_buf_)) {
			ring_buffer_write(&outbound_ring_buf_, byte);
		}
		enable_uart_tx_ints(true);
	}
}

void uart_read(uint8_t* buf, uint32_t n) {
	const uint32_t clamped_n = clamp(n, 0,
			ring_buffer_available(&inbound_ring_buf_));
	for (uint32_t i = 0; i < clamped_n; ++i) {
		buf[i] = ring_buffer_read(&inbound_ring_buf_);
	}
}

uint8_t uart_peek(void) {
	return ring_buffer_peek(&inbound_ring_buf_);
}

int uart_available(void) {
	return ring_buffer_available(&inbound_ring_buf_);
}

void UART_IRQHandler(void) {
// bit is active low, indicates interrupt has occurred.
	uint32_t uart_int_status = LPC_UART->IIR;
	if ((uart_int_status & BIT(0)) == 0) {
		// three bits contain cause of interrupts
		uart_int_status = ((uart_int_status >> 1) & 0x7);
		switch (uart_int_status) {
		case 1: {
			// transmit ring_buffer empty
			const bool outbound_data_available = ring_buffer_available(
					&outbound_ring_buf_);

			if (outbound_data_available) {
				if (uart_tx_buf_empty()) {
					const volatile uint8_t outbound_data = ring_buffer_read(
							&outbound_ring_buf_);
					LPC_UART->THR = outbound_data;
				}
			} else {
				// disable tx ints if outbound ring buf is empty,
				enable_uart_tx_ints(false);
			}
		}
			break;
		case 2: {
			if (!ring_buffer_full(&inbound_ring_buf_)) {
				const volatile uint8_t inbound_data = LPC_UART->RBR;
				ring_buffer_write(&inbound_ring_buf_, inbound_data);
				scheduler_set_pending(UART_DATA_BYTE_HANDLER);
			} else {
				while (1) {
				} // remove at some point. Curious to see if we ever hit this.
			}
		}
			break;
		default: {
			// manual recommends reading line status reg in this case.
			const volatile uint8_t scratch_byte = LPC_UART->LSR;
		}
			break;
		}
	}
}

