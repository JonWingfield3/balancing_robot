/*
 * message_transport.h
 *
 *  Created on: Nov 4, 2017
 *      Author: jonathanwingfield
 */

#ifndef MESSAGE_TRANSPORT_H_
#define MESSAGE_TRANSPORT_H_

#include <message_definitions.h>

void message_transport_init(void);
message_t* message_transport_get_message(void);
void message_transport_send_message(message_t* msg);

#endif /* MESSAGE_TRANSPORT_H_ */
