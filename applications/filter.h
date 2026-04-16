
#ifndef __FILTER_H
#define __FILTER_H




#include "stm32h7xx_hal.h"
#include <rtthread.h>
#include <rthw.h>




struct median_filter
{
    int32_t *buf;
    int32_t len;
    int32_t last_value;
    uint16_t cnt;
    uint8_t full_flag;
};

struct sliding_average_filter
{
    int16_t w_size;
    int16_t head;
    float sum;
    float *cache;
};

void sliding_average_filter_init(struct sliding_average_filter *filter, int16_t w_size);
float sliding_average_filter(struct sliding_average_filter *filter, float k);





void median_filter_init(struct median_filter *filter, uint16_t len);
int32_t median_filter(struct median_filter *filter, int32_t value);





#endif


