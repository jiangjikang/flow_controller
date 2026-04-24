
#ifndef __UART_H
#define __UART_H


#include "stm32h7xx_hal.h"



enum serial
{
	SERIAL_3 = 1,
	SERIAL_4,
	SERIAL_6,
	SERIAL_7,
};

#define RCV_EVENT_FLAG_SERIAL(i) 1 << i


#define SERIAL_NUM_MAX	4


#endif
