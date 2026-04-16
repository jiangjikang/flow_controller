#ifndef __ANALOG_H
#define __ANALOG_H

#include "stm32h7xx.h"
#include <rtthread.h>



enum analog_type
{
	OUTPUT_TYPE_NONE = 0,
	UART_TYPE = 1,
	RS485_TYPE = 2,
	PWM_TYPE = 3,
	VOLT_TYPE = 4,
	CURRENT_TYPE = 5,
	NPN_TYPE = 6,
	PNP_TYPE = 7,
};


enum analog_ch
{
	ANCH_1 = 1,
	ANCH_2,
	ANCH_3,
	ANCH_4,
};



uint16_t get_analog_value_uncalib(uint8_t channel);
uint16_t adc_read_voltage(uint8_t channel);
uint16_t get_analog_value_calib(uint8_t channel);
uint16_t analog_calib(enum analog_type type,uint16_t value);


#endif

