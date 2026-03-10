
#include "filter.h"



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
        filter->last_value = buf_copy[mid_index];
    }
    else
    {
        filter->last_value = (buf_copy[mid_index] + buf_copy[mid_index + 1]) >> 1;
    }

    rt_free(buf_copy);

    return filter->last_value;
}


