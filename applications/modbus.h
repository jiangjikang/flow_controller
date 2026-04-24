
#ifndef __MODBUS_H
#define __MODBUS_H


#include "stm32h7xx_hal.h"
#include "uart.h"

#define M_REG_HOLDING_NREGS	100
#define RX_BUF_LEN	(M_REG_HOLDING_NREGS*2 + 5)



rt_err_t mb_read_holding_register(enum serial serial_num,uint8_t slave_addr, uint16_t reg_addr, uint16_t reg_num, uint32_t timeout);
rt_err_t mb_parallel_read_holding_register(uint8_t slave_addr, uint16_t reg_addr, uint16_t reg_num, uint32_t timeout);
rt_err_t mb_write_holding_register(enum serial serial_num,uint8_t slave_addr, uint16_t reg_addr, uint16_t data, uint32_t timeout);
rt_err_t mb_rewrite_holding_register(enum serial serial_num,uint8_t slave_addr, uint16_t reg_addr, uint16_t data, uint32_t timeout);
rt_err_t mb_reread_holding_register(enum serial serial_num,uint8_t slave_addr, uint16_t reg_addr, uint16_t reg_num, uint32_t timeout);

rt_err_t mb_write_8_holding_register(enum serial serial_num,uint8_t slave_addr, uint16_t reg_addr, uint16_t data[], uint32_t timeout);
rt_err_t mb_write_holding_register_2(uint8_t slave_addr, uint16_t reg_addr, uint16_t data[], uint32_t timeout);



#endif
