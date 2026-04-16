#include "user_table.h"
#include "modbus.h"
#include "analog.h"




struct slave_table slave[MB_MASTER_TOTAL_SLAVE_NUM + 10] =
{
	{RT_NULL,RT_NULL},
    {SERIAL_2,ANCH_1},
    {SERIAL_3,ANCH_2},
    {SERIAL_4,ANCH_3},
	{SERIAL_5,ANCH_4},
};


