/*
 * Irq.c
 *
 *  Created on: Jun 27, 2020
 *      Author: owen
 */

#include "stm32f4xx_hal.h"
#include "Irq.h"

int disableIrqCount = 0 ;
void disableIRQ(){
	__disable_irq();
    disableIrqCount++;
}
void enableIRQ(){
	if(disableIrqCount > 0){
		disableIrqCount--;
	}
	if(disableIrqCount == 0){
		__enable_irq();
	}
}
