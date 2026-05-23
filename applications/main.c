/*
 * Copyright (c) 2006-2018, RT-Thread Development Team
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Change Logs:
 * Date           Author       Notes
 * 2019-03-05     whj4674672   first version
 */

#include <rtthread.h>
#include <board.h>
#include "modbus.h"
#include "sensor_calib.h"
#include "filter.h"




/* Ïß³Ì¾ä±ú */
rt_thread_t start_thread = RT_NULL;
rt_thread_t flow_con = RT_NULL;


int main(void)
{
	
	
	
	static uint8_t reg_val = 1;
	
	mb_write_holding_register(SERIAL_6, 1, 50, 1234, 100);
	for (uint8_t i = 0; i < 100; i++){}
	mb_parallel_read_holding_register(1, 0, 100, 100);
		
	
		
	flow_con = rt_thread_create("flow_control", flow_control, RT_NULL, 2048, 3, 3);
	if (flow_con != RT_NULL)
	{
		rt_thread_startup(flow_con);
	}
	
	
	
	while (1);
	return RT_EOK;
}













