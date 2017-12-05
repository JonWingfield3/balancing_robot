/*
 * uart_transport.c
 *
 *  Created on: Nov 4, 2017
 *      Author: jonathanwingfield
 */

#include <uart_transport.h>

#include <message_definitions.h>
#include <scheduler.h>
#include <uart.h>

static message_t msgs_[2];
static int last_valid_msg_idx_;
static int incoming_msg_idx_;

#define SYNC_BYTE (0x55)
#define ACK_BYTE (0x22)
#define ERR_BYTE (0x33)
#define TRANSPORT_TIMEOUT_TICKS (000)

typedef enum {
	AWAITING_SYNC, AWAITING_HEADER, AWAITING_PLD, AWAITING_CRC, TRANSPORT_ERROR
} uart_transport_state_t;

static int uart_data_byte_handler(void) {
	static uart_transport_state_t msg_state = 0;
	static int pld_byte_cntr = 0;

	while (uart_available() > 0) {
		uint8_t rx_byte;
		uart_read(&rx_byte, 1);
		switch (msg_state) {
		case AWAITING_SYNC: {
			if (rx_byte == SYNC_BYTE) {
				msg_state = AWAITING_HEADER;
			} // else: assume out of sync reception. Don't send error byte.
		}
			break;
		case AWAITING_HEADER: {
			// received the msg_header
			const uint8_t pld_len = (rx_byte >> 4) & 0xf;
			const uint8_t msg_id = rx_byte & 0xf;
			if (in_range(msg_id, MESSAGE_ID_FIRST, MESSAGE_ID_LAST)) {
				msgs_[incoming_msg_idx_].msg_id = msg_id;
				msgs_[incoming_msg_idx_].pld_len = pld_len;
				if (pld_len > 0) {
					pld_byte_cntr = 0;
					msg_state = AWAITING_PLD;
				} else {
					msg_state = AWAITING_CRC;
				}
			} else {
				msg_state = TRANSPORT_ERROR;
			}
		}
			break;
		case AWAITING_PLD: {
			msgs_[incoming_msg_idx_].pld[pld_byte_cntr++] = rx_byte;
			if (pld_byte_cntr == msgs_[incoming_msg_idx_].pld_len) {
				msg_state = AWAITING_CRC;
			}
		}
			break;
		case AWAITING_CRC: {
			// verify checksum. If valid update msg pointers.
			uint8_t calc_checksum = msgs_[incoming_msg_idx_].msg_header ^ SYNC_BYTE;
			for (int i = 0; i < msgs_[incoming_msg_idx_].pld_len; ++i) {
				calc_checksum ^= msgs_[incoming_msg_idx_].pld[i];
			}
			if (calc_checksum == rx_byte) {
				incoming_msg_idx_ = (incoming_msg_idx_ + 1) & 1;
				last_valid_msg_idx_ = (last_valid_msg_idx_ + 1) & 1;
				uart_write_byte(ACK_BYTE);
				scheduler_set_pending(MESSAGE_HANDLER);
				msg_state = AWAITING_SYNC;
			} else {
				msg_state = TRANSPORT_ERROR;
			}
		}
			break;
		default:
			while (1) {
			} // trap execution if we end up here
			break;
		}

		if (msg_state == TRANSPORT_ERROR) {
			uart_write_byte(ERR_BYTE);
			msg_state = AWAITING_SYNC;
		}
	}
	return 0;
}

void uart_transport_init(void) {
	incoming_msg_idx_ = 0;
	last_valid_msg_idx_ = 1;
	scheduler_init_task(UART_DATA_BYTE_HANDLER, uart_data_byte_handler,
	TRANSPORT_TIMEOUT_TICKS);
	uart_init();
}

message_t* uart_transport_get_message(void) {
	return &msgs_[last_valid_msg_idx_];
}

void uart_transport_send_message(message_t* msg) {
	// calculate checksum using bytewise XOR.
	uint8_t checksum = msg->msg_header ^ SYNC_BYTE;
	for (int i = 0; i < msg->pld_len; ++i) {
		checksum ^= msg->pld[i];
	}
	const uint8_t* msg_ite = (uint8_t*) msg;
	uart_write_byte(SYNC_BYTE);
	uart_write(msg_ite, msg->pld_len + 1); // + 1 for header
	uart_write_byte(checksum);
}
