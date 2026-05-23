
#include "filter.h"
#include "string.h"
#include "stdlib.h"
#include "my_dac.h"
#include "ADS8688.h"



void median_filter_init(struct median_filter *filter, uint16_t len)
{
    filter->len = len;
    filter->buf = rt_malloc(len * sizeof(int32_t));
    filter->last_value = 0;
    filter->cnt = 0;
    filter->full_flag = RT_FALSE;
}


int32_t median_filter(struct median_filter *filter, int32_t value)
{
    uint16_t i, j;
    int32_t temp;

    int32_t *buf_copy = rt_malloc(filter->len * sizeof(int32_t));

    filter->buf[filter->cnt++] = value;

    if((filter->full_flag == RT_FALSE) && (filter->cnt >= filter->len))
    {
        filter->full_flag = RT_TRUE;
    }

    if (filter->full_flag != RT_TRUE)    // 数据开始未填满输出实时值
    {
        return value;
    }

    if(filter->cnt >= filter->len)
    {
        filter->cnt = 0;
    }

    rt_memcpy(buf_copy,filter->buf,filter->len * sizeof(int32_t));

    for (i = 0; i < filter->len - 1; i++)
    {
        for (j = 0; j < filter->len - i - 1; j++)
        {
            if (buf_copy[j] > buf_copy[j + 1])
            {
                temp = buf_copy[j];
                buf_copy[j] = buf_copy[j + 1];
                buf_copy[j + 1] = temp;
            }
        }
    }

    uint16_t mid_index = (filter->len - 1) >> 1;
    if((filter->len & 0x01) > 0)
    {
        filter->last_value = ( buf_copy[mid_index-1] + buf_copy[mid_index] + buf_copy[mid_index+1] )/3;
    }
    else
    {
        filter->last_value = (buf_copy[mid_index-1] + buf_copy[mid_index] + buf_copy[mid_index + 1] + buf_copy[mid_index + 2]) >> 2;
    }

    rt_free(buf_copy);

    return filter->last_value;
}


float sliding_average_filter(struct sliding_average_filter *filter, float k)
{
        filter->cache[filter->head] = k;
        filter->head = (filter->head + 1) % filter->w_size;
        filter->sum = filter->sum + k - filter->cache[filter->head];
        return (filter->sum / (float)(filter->w_size - 1));
}


void sliding_average_filter_init(struct sliding_average_filter *filter, int16_t w_size)
{
        w_size += 1;
        filter->cache = (float*)malloc(sizeof(float) * w_size);
        memset(filter->cache, 0.0f, sizeof(float) * w_size);
        filter->w_size = w_size;
        filter->sum = 0;
        filter->head = 0;
}


void flow_control(void *parameter)
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
	
	DAC1_Init();		// 初始化DAC1
	/* set LED0 pin mode to output */	
  rt_pin_mode(LED0_PIN, PIN_MODE_OUTPUT);	
	rt_hw_ads8688_config();
	median_filter_init(&adc_median_filter, 10);
	

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
		
		
		if(count++ >= 20)
		{
			count = 0;
			sprintf ((char *)dis_buf,"CH0: %10.4lfmV  D: %04X", volt_mV , (uint16_t)adc_data);
			rt_kprintf("%s\r\n", (char *)dis_buf);
			rt_kprintf("flow = %d\r\n", (int32_t)flow_rate);
			
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
}



