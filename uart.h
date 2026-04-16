
#ifndef __MY_DAC_H
#define __MY_DAC_H

#include <rtthread.h>
#include "crc.h"

#define UART4						"uart4"


rt_err_t uart_send(const void* buffer, rt_size_t len);
rt_err_t uart4_rx_callback(rt_device_t dev);




#endif