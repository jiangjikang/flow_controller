#ifndef __USER_TABLE_H
#define __USER_TABLE_H



#include "stm32h7xx.h"
#include <rtthread.h>

struct slave_table
{
    uint16_t serial;  // 串口号
    uint16_t channel; // 模拟量采集通道
};




extern struct slave_table slave[];



#endif
