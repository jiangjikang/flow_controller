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
#include "my_dac.h"
#include "modbus.h"
#include "ADS8688.h"








int main(void)
{
	
	uint16_t adc_data_tmp = 0;
	uint16_t adc_data = 0;

	uint8_t dis_buf[40];
	float volt_mV;
	float current_mA;
	float flow_rate = 0;
	
	rt_err_t err;

	float vlot_set_value_mV = 0;	
	static uint16_t count = 0;
	
	int32_t adc_value_unfiltered = 0;
	
	DAC1_Init();		// ≥ı ºªØDAC1
	 /* set LED0 pin mode to output */	
    rt_pin_mode(LED0_PIN, PIN_MODE_OUTPUT);	
	rt_hw_ads8688_config();
	median_filter_init(&adc_median_filter, 10);
	
			mb_write_holding_register(SERIAL_6, 1, 50, 1234, 100);
			for (uint8_t i = 0; i < 100; i++){}
			mb_parallel_read_holding_register(1, 0, 100, 100);
//			for (uint8_t i = 0; i < 100; i++);
			
		
    while (1)
    {
			
			err = ads8688_get_man_ch_data(MAN_CH_0,&adc_data_tmp);
			if(err == RT_EOK)
			{
				adc_data = median_filter(&adc_median_filter, adc_data_tmp);
			}
			else
			{
				rt_kprintf("adc read error!\r\n");
			}
			volt_mV = ((float)adc_data-32767)*20480.0/65536;
			current_mA = volt_mV / 499;
			
			flow_rate = convert_current_to_flow(current_mA);
			
			flow_rate *= g_k_factor;
			
			if(flow_rate < 0)
			{
				flow_rate = 0;
			}
			
			static uint8_t reg_val = 1;
			if(count++ >= 20)
			{
				count = 0;
				sprintf ((char *)dis_buf,"CH0: %10.4lfmV  D: %04X", volt_mV , (uint16_t)adc_data);
				rt_kprintf("%s\r\n", (char *)dis_buf);
				rt_kprintf("flow = %d\r\n", (int32_t)flow_rate);
				
				mb_write_holding_register(SERIAL_6, 1, 34, reg_val, 100);
				reg_val = (reg_val == 1) ? 2 : 1;
				
			}
			
			
			if(flow_rate < g_set_flow_val - 500)
			{
				vlot_set_value_mV += 50;
			}
			else if(flow_rate < g_set_flow_val-100)
			{
				vlot_set_value_mV += 5;
			}
			else if(flow_rate < g_set_flow_val-20)
			{
				vlot_set_value_mV += 2;
			}
			else if(flow_rate < g_set_flow_val - 2)
			{
				vlot_set_value_mV += 0.1;
			}
			
			else if(flow_rate > g_set_flow_val + 500)
			{	
				vlot_set_value_mV -= 50;
			}
			else if(flow_rate > g_set_flow_val + 100)
			{	
				vlot_set_value_mV -= 5;
			}
			else if(flow_rate > g_set_flow_val + 20)
			{	
				vlot_set_value_mV -= 2;
			}
			else if(flow_rate > g_set_flow_val + 2)
			{	
				vlot_set_value_mV -= 0.1;
			}
			else if(g_set_flow_val == 0)
			{
				vlot_set_value_mV = 0;
			}
				
			
			if(vlot_set_value_mV < 0)
			{
				vlot_set_value_mV = 0;
			}
			else if(vlot_set_value_mV > 7000)
			{
				vlot_set_value_mV = 7000;
			}
			
			set_dac_output_voltage(vlot_set_value_mV);

			rt_thread_mdelay(100);
    }
    return RT_EOK;
}
