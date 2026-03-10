
#ifndef __MY_DAC_H
#define __MY_DAC_H


#include "stm32h7xx_hal.h"



extern DAC_HandleTypeDef DAC1_Handler;//DAC¾ä±ú

void DAC1_Init(void);
void DAC1_Set_Vol(uint16_t vol);
void DAC1_Set_Vo2(uint16_t vo2);


#endif
