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
#include "sensor_calib.h"
#include "filter.h"
#include "modbus.h"




/* 线程句柄 */
rt_thread_t start_thread = RT_NULL;
rt_thread_t flow_con = RT_NULL;
rt_thread_t flow_calib = RT_NULL;


void start_task_entry(void *parameter)
{
	/* 创建流量控制任务 */
	flow_con = rt_thread_create("flow_control", flowControl_thread_entry, RT_NULL, 1024, 3, 3);
	if (flow_con != RT_NULL)
	{
		rt_thread_startup(flow_con);
	}
	
	
	/* 创建流量标定任务 */
//	flow_calib = rt_thread_create("calib_thread_entry", calib_thread_entry, RT_NULL, 1024, 2, 2);
//	if (flow_calib != RT_NULL)
//	{
//		rt_thread_startup(flow_calib);
//	}
}


int main(void)
{
	start_thread = rt_thread_create("start_task_entry", start_task_entry, RT_NULL, 2048, 3, 3);
	if (start_thread != RT_NULL)
	{
		rt_thread_startup(start_thread);
	}
	
	
	
	while (1)
	{
		rt_thread_mdelay(1000);
	}
	return RT_EOK;
}














