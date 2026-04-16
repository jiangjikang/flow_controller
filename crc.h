#ifndef __CRC_H
#define __CRC_H


#include <rtthread.h>

rt_uint16_t modbus_crc16(rt_uint8_t *buf, rt_uint16_t len);



#endif