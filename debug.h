
#ifndef __DEBUG_H
#define __DEBUG_H


#include "stm32h7xx_hal.h"
#include <rtthread.h>

typedef enum {
    MB_INFO = (1 << 0),
	SLAVE_INFO =(1 << 1),
	AN_INFO = (1 << 2),
    OTHER_INFO = (1 << 3),
    ERR = (1 << 4),
	CAL_INFO = (1 << 5),

} LOG_LEVEL;



#endif
