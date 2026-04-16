#ifndef __MODBUS_H
#define __MODBUS_H


#include <rtthread.h>

#define MODBUS_OK          0
#define MODBUS_ERROR      -1
#define MODBUS_CRC_ERROR  -2
#define MODBUS_EXCEPT     -3

void modbus_read_holding_registers(rt_uint8_t slave,
                                   rt_uint16_t addr,
                                   rt_uint16_t num); 

void modbus_write_single_register(rt_uint8_t slave,
                                  rt_uint16_t addr,
                                  rt_uint16_t value);

void modbus_write_multiple_registers(rt_uint8_t slave,
                                     rt_uint16_t addr,
                                     rt_uint16_t num,
                                     rt_uint16_t *data);

int modbus_parse(rt_uint8_t *rx_buf, rt_uint16_t len);


#endif

