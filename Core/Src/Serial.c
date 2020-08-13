/*
 * Serial.c
 *
 *  Created on: Aug 13, 2020
 *      Author: academic
 */
#include "Irq.h"
#include "Nvic.h"
#include "Serial.h"
#include <stdio.h>
#include <stdint.h>
#include <malloc.h>
#include <stdarg.h>

volatile int usartTurn = 0;
UsartRegs * sharedUsart;
char * messageToSend;

void freeMessage(char * msg){
    if(msg)
        free(msg);
}

void serialSend(UsartRegs * usart,char *message,...){
	disableIRQ();
    int actualLength;
    char* buffer;


    va_list arg;
    va_start(arg, message);

    actualLength = vsnprintf(NULL,0, message, arg);
    buffer =malloc(actualLength + 1);
    vsnprintf(buffer,actualLength + 1, message, arg);
    va_end(arg);
    usartSendMessage(usart,buffer);
    freeMessage(buffer);
    enableIRQ();
}

void usartSendMessage(UsartRegs * usart,char *message){
	usartEnableInterrupt(uart5,TRANS_COMPLETE);
	sharedUsart = usart;
	messageToSend =message;
	usartTurn = 1;
}
