/*
 * Copyright (c) 2006-2018, RT-Thread Development Team
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Change Logs:
 * Date           Author       Notes
 * 2019-03-05     whj4674672   first version
 */

#include "ADS8688.h"
#include "sensor_calib.h"

static void start_thread_entry(void *parameter);






int main(void)
{
	rt_thread_t thread_start = rt_thread_create("start_thread_entry", start_thread_entry, RT_NULL, 2048, 10, 10);
	if (thread_start != RT_NULL)
	{
		rt_thread_startup(thread_start);
	}
	
		
  return 0;
}


static void start_thread_entry(void *parameter)
{
    rt_thread_t flow;
    rt_thread_t calib;

    /* 流量控制线程 */
    flow = rt_thread_create("flow_controller_thread", flow_controller_thread, RT_NULL, 1024, 15, 10);

    if (flow != RT_NULL)
    {
        rt_thread_startup(flow);
    }

    /* 传感器标定线程 */
    calib = rt_thread_create("calib_thread_entry", calib_thread_entry, RT_NULL, 1024, 16, 10);

    if (calib != RT_NULL)
    {
        rt_thread_startup(calib);
    }


}















