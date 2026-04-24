#include "uart.h"
#include "board.h"

#define UART7_RX_BUFFER_SIZE	128
uint8_t uart7_rx_buffer[UART7_RX_BUFFER_SIZE];
uint16_t uart7_rx_count = 0;

char *serial_dev_name[SERIAL_NUM_MAX] = {"uart3", "uart4", "uart6", "uart7"};
static rt_device_t serial_dev[SERIAL_NUM_MAX];
static struct rt_event uart_rcv_event;
static rt_timer_t timer[SERIAL_NUM_MAX] = {RT_NULL};



static rt_err_t uart_input(rt_device_t dev, rt_size_t size)
{
	rt_err_t result = RT_EOK;
	uint8_t i = 0;
	
	for (i = 0; i < SERIAL_NUM_MAX; i++)
	{
		if (dev == serial_dev[i])
		{
			rt_timer_start(timer[i]);
		}
	}
	return result;
}



/**
 * @brief    
 *
 * @param    serial_num 用于统一保存设备编号
 * @param    buf 接收数据缓存
 * @param    size 数据接收大小
 * @param    timeout 最大超时时间
 *
 * @return   成功返回数据长度
 */
rt_size_t serial_recv(enum serial serial_num, void *buf, rt_size_t size, uint32_t timeout)
{
	rt_err_t result;
	rt_size_t rx_length;
	
	result = rt_event_recv(&uart_rcv_event, RCV_EVENT_FLAG_SERIAL(serial_num), RT_EVENT_FLAG_OR | RT_EVENT_FLAG_CLEAR, timeout, RT_NULL);
	
	if (result == RT_EOK)
	{
		rx_length = rt_device_read(serial_dev[serial_num], 0, buf, size);
		return rx_length;
	}
	else 
	{
		return 0;
	}
}


uint8_t buf_[256];
void clear_rxbuffer(enum serial serial_num)
{
	rt_event_recv(&uart_rcv_event, RCV_EVENT_FLAG_SERIAL(serial_num), RT_EVENT_FLAG_OR | RT_EVENT_FLAG_CLEAR, 0, RT_NULL);
	rt_device_read(serial_dev[serial_num], 0, buf_, sizeof(buf_));
}



rt_size_t serial_send(enum serial serial_num,void *buf,rt_size_t size)
{
#ifdef HALF_DUPLEX_MODE
    rt_size_t ret;
    USART_TypeDef * puart = NULL;
    if(serial_nu == SERIAL_2)
    {
        puart = USART2;
    }
    else if(serial_nu == SERIAL_3)
    {
        puart = USART3;
    }
    else if(serial_nu == SERIAL_4)
    {
        puart = UART4;
    }
    // HAL_HalfDuplex_EnableTransmitter(&huart1);
    uint32_t tmpreg = 0x00U;
    if(puart != NULL)
    {
        tmpreg = puart->CR1;
        tmpreg &= (uint32_t)~((uint32_t)(USART_CR1_TE | USART_CR1_RE));
        tmpreg |= (uint32_t)USART_CR1_TE;
        puart->CR1 = tmpreg;
    }

    ret = rt_device_write(serial_dev[serial_nu], 0, buf,size);
    if(puart != NULL)
    {
        tmpreg = puart->CR1;
        tmpreg &= (uint32_t)~((uint32_t)(USART_CR1_TE | USART_CR1_RE));
        tmpreg |= (uint32_t)USART_CR1_RE;
        puart->CR1 = tmpreg;
    }
    return ret;
	
#else 
	return rt_device_write(serial_dev[serial_num], 0, buf,size);
	
#endif
	
}





static void timeout1(void *parameter)
{
    rt_event_send(&uart_rcv_event, RCV_EVENT_FLAG_SERIAL(0));
}
static void timeout2(void *parameter)
{
    rt_event_send(&uart_rcv_event, RCV_EVENT_FLAG_SERIAL(1));
}
static void timeout3(void *parameter)
{
    rt_event_send(&uart_rcv_event, RCV_EVENT_FLAG_SERIAL(2));
}
static void timeout4(void *parameter)
{
    rt_event_send(&uart_rcv_event, RCV_EVENT_FLAG_SERIAL(3));
}



#define DELAY_BETWEEN_POLLS   40
static int uart_dma_init(void)
{
	rt_err_t ret = RT_EOK;
  rt_err_t result;
	
	struct serial_configure config = RT_SERIAL_CONFIG_DEFAULT;
	for (uint8_t i = 0; i < SERIAL_NUM_MAX; i++)
	{
		serial_dev[i] = rt_device_find(serial_dev_name[i]);
		if (!serial_dev[i])
		{
			rt_kprintf("find %s failed!\n", serial_dev_name[i]);
			ret = RT_ERROR;
			goto cmd_fail;
		}
	}
	
	config.baud_rate = BAUD_RATE_115200;
	config.data_bits = DATA_BITS_8;
	config.stop_bits = STOP_BITS_1;
	config.parity = PARITY_NONE;
	config.bufsz = 256;
	
	for (uint8_t i = 0; i < SERIAL_NUM_MAX; i++)
	{
		rt_device_control(serial_dev[i], RT_DEVICE_CTRL_CONFIG, &config);
	}
	
	result = rt_event_init(&uart_rcv_event, "uart_rcv_event", RT_IPC_FLAG_PRIO);
	if (result != RT_EOK)
	{
		rt_kprintf("init uart_rcv_event failed.\n");
    ret = RT_ERROR;
    goto cmd_fail;
	}
	
	for( uint16_t i = 0; i < SERIAL_NUM_MAX; i++)
  {
        /* 以 DMA 接收及轮询发送方式打开串口设备 */
        rt_device_open(serial_dev[i], RT_DEVICE_FLAG_DMA_RX);
        /* 设置接收回调函数 */
        rt_device_set_rx_indicate(serial_dev[i], uart_input);
  }
	
	timer[0] = rt_timer_create("timer1", timeout1, RT_NULL, 30, RT_TIMER_FLAG_ONE_SHOT);
	if (timer[0] == RT_NULL)
  {
        rt_kprintf("create timer1 failed.\n");
        ret = RT_ERROR;
        goto cmd_fail;
  }
	timer[1] = rt_timer_create("timer2", timeout2, RT_NULL, DELAY_BETWEEN_POLLS, RT_TIMER_FLAG_ONE_SHOT);
	if (timer[0] == RT_NULL)
  {
        rt_kprintf("create timer2 failed.\n");
        ret = RT_ERROR;
        goto cmd_fail;
  }
	timer[2] = rt_timer_create("timer3", timeout3, RT_NULL, DELAY_BETWEEN_POLLS, RT_TIMER_FLAG_ONE_SHOT);
	if (timer[0] == RT_NULL)
  {
        rt_kprintf("create timer3 failed.\n");
        ret = RT_ERROR;
        goto cmd_fail;
  }
	timer[3] = rt_timer_create("timer4", timeout4, RT_NULL, DELAY_BETWEEN_POLLS, RT_TIMER_FLAG_ONE_SHOT);
	if (timer[0] == RT_NULL)
  {
        rt_kprintf("create timer4 failed.\n");
        ret = RT_ERROR;
        goto cmd_fail;
  }
	
	
cmd_fail:
	return ret;
}
INIT_ENV_EXPORT(uart_dma_init);









