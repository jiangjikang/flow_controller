
#ifndef __UART_H
#define __UART_H


#include "stm32h7xx_hal.h"
#include <rtthread.h>


enum serial
{
	SERIAL_3 = 1,
	SERIAL_4,
	SERIAL_6,
	SERIAL_7,
};

#define RCV_EVENT_FLAG_SERIAL(i) (1 << i)
#define SERIAL_NUM_MAX	4


rt_size_t serial_recv(enum serial serial_num, void *buf, rt_size_t size, uint32_t timeout);
rt_size_t serial_send(enum serial serial_num,void *buf,rt_size_t size);

#endif
