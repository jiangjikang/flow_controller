#ifndef __UART_H
#define __UART_H

#include "stm32h7xx.h"
#include <rtthread.h>


enum serial_num
{
	SERIAL_1 = 1,
	SERIAL_2,
	SERIAL_3,
	SERIAL_4,
	SERIAL_5,
};



#define SERIAL_NUM_MAX  4	//最大串口号
#define RCV_EVENT_FLAG_SERIAL(i)  (1 << i)  //接收事件标志


rt_size_t serial_send(enum serial_num serial_nu,void *buf,rt_size_t size);
rt_size_t serial_recv(enum serial_num serial_nu,void *buf,rt_size_t size,uint32_t timeout);


#endif

