#include "uart.h"
#include "modbus.h"

static rt_device_t uart4_dev = RT_NULL;

rt_err_t uart_send(const void* buffer, rt_size_t len)
{
	uart4_dev = rt_device_find(UART4);
	if (uart4_dev)
	{
		rt_device_open(uart4_dev, RT_DEVICE_FLAG_INT_RX | RT_DEVICE_FLAG_INT_TX);
		rt_device_write(uart4_dev, 0, buffer, len);
		
		return RT_EOK;
	}
	
	return RT_ERROR;
}































