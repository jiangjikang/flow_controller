#include "sensor_calib.h"


rt_uint16_t temp_flow;


/**
 * @brief  检测LED是否焊接不良
 *
 *
 * @retval NUL
 */
void led_check(void)
{
	static int led_reg = 1; // 初始化LED寄存器数值
	mb_write_holding_register(SERIAL_6, 1, 34, led_reg, 0xF);	// 向该函数以1s为周期发送1和2
	led_reg = (led_reg == 1) ? 2 : 1;	// 改变led_reg值
}





/**
 * @brief  标定线程入口函数。
 *
 * 线程启动后延时等待系统稳定，然后执行一次完整标定流程；如果失败则延时后重试。
 *
 * @param[in] parameter  线程参数，当前未使用。
 */
void calib_thread_entry(void *parameter)
{
		
		START_CALIB();	// 宏定义函数：向50号寄存器输入1234启动标定
    
    while (1)
    {
			led_check();	// 单一功能函数用于检测LED是否焊接不良
			
		
      rt_thread_mdelay(1000);	// 周期运行任务，延时用于让线程进入阻塞态并释放CPU
    }
}



