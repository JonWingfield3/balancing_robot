/*
 * uart_transport.h
 *
 *  Created on: Nov 4, 2017
 *      Author: jonathanwingfield
 */

#ifndef UART_TRANSPORT_H_
#define UART_TRANSPORT_H_

#include <message_definitions.h>

void uart_transport_init(void);
message_t* uart_transport_get_message(void);
void uart_transport_send_message(message_t* msg);

#endif /* UART_TRANSPORT_H_ */
