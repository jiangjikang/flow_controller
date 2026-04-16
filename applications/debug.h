#ifndef __DEBUG_H
#define __DEBUG_H


#include "stm32h7xx.h"


typedef enum {
    MB_INFO = (1 << 0),
	SLAVE_INFO =(1 << 1),
	AN_INFO = (1 << 2),
    OTHER_INFO = (1 << 3),
    ERR = (1 << 4),
	CAL_INFO = (1 << 5),

} LOG_LEVEL;

extern uint32_t log_level;

#define log_develop(level,format, ...)  \
do { \
    if((uint32_t)level & log_level)\
    rt_kprintf(format,##__VA_ARGS__);\
} while (0)






#endif


