
#ifndef __SENSOR_CALIB_H
#define __SENSOR_CALIB_H


#include "stm32h7xx_hal.h"
#include <rtthread.h>
#include "modbus.h"


#define START_CALIB()	mb_write_holding_register(SERIAL_6, 1, 50, 1234, 0xF);
#define LED_CHECK()	

/**
 * @brief  执行完整传感器标定流程。
 *
 * @param[in] serial_num  串口编号。
 * @param[in] slave_addr  从站地址。
 *
 * @return RT_EOK 表示标定成功，RT_ERROR 表示标定失败。
 */
rt_err_t sensor_calib_run(enum serial serial_num, uint8_t slave_addr);

/**
 * @brief  标定线程入口函数。
 *
 * @param[in] parameter  线程参数，当前未使用。
 */
void calib_thread_entry(void *parameter);



#endif

