/*
 * Serial.h
 *
 *  Created on: Aug 13, 2020
 *      Author: academic
 */

#ifndef INC_SERIAL_H_
#define INC_SERIAL_H_
#include "Usart.h"
void freeMessage(char * msg);
void serialSend(UsartRegs * usart,char *message,...);
void usartSendMessage(UsartRegs * usart,char *message);
#endif /* INC_SERIAL_H_ */
