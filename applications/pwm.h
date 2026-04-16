#ifndef __PWM_H
#define __PWM_H


#include "rtthread.h"
#include "stm32h7xx_hal.h"




#define PULSE_CAP_1_PIN_NUM            	GET_PIN(A, 6)
#define PULSE_CAP_2_PIN_NUM            	GET_PIN(A, 7)
#define PULSE_CAP_3_PIN_NUM            	GET_PIN(B, 0)
#define PULSE_CAP_4_PIN_NUM 		   	GET_PIN(B, 8)
#define PULSE_CAP_5_PIN_NUM            	GET_PIN(B, 7)
#define PULSE_CAP_6_PIN_NUM 		   	GET_PIN(B, 6)




rt_err_t get_pwm_count(uint16_t *buf);




#endif





