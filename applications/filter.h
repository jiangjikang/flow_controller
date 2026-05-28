
#ifndef __FILTER_H
#define __FILTER_H




#include "stm32h7xx_hal.h"

#include <rthw.h>
#include <rtthread.h>



struct median_filter
{
    int32_t *buf;
    int32_t len;
    int32_t last_value;
    uint16_t cnt;
    uint8_t full_flag;
};




void median_filter_init(struct median_filter *filter, uint16_t len);
int32_t median_filter(struct median_filter *filter, int32_t value);





#endif


