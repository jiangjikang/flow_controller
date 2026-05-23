#include "sensor_calib.h"






/**
	标定线程入口函数
*/
void calib_thread_entry(void *parameter)
{
		static uint8_t reg_val = 1;
	
		mb_write_holding_register(SERIAL_6, 1, 50, 1234, 100);
		for (uint8_t i = 0; i < 100; i++){}
		mb_parallel_read_holding_register(1, 0, 100, 100);
    while (1)
    {
			mb_write_holding_register(SERIAL_6, 1, 34, reg_val, 100);
			reg_val = (reg_val == 1) ? 2 : 1;
			rt_thread_mdelay(1000); 
    }
}



