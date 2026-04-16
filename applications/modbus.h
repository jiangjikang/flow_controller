#ifndef __MODBUS_H
#define __MODBUS_H

#include "stm32h7xx.h"
#include "uart.h"



#define BOARD_ID	0x01
#define SENSOR_ID 0x01
#define AN_VOLT_MODULE_ID  0x02
#define AN_CURRENT_MODULE_ID  0x03
#define ANLONG_CHANNEL_NUM   3

#define M_REG_HOLDING_NREGS 100
#define RX_BUF_LEN (M_REG_HOLDING_NREGS*2 + 5)
#define MB_MASTER_TOTAL_SLAVE_NUM  ( 3 )


extern uint16_t user_reg_hold_buf_2[MB_MASTER_TOTAL_SLAVE_NUM][M_REG_HOLDING_NREGS];
extern uint16_t user_reg_hold_buf[];

#define MB_TIMEOUT 300


rt_err_t mb_read_holding_register(enum serial_num serial_nu,uint8_t slave_addr, uint16_t reg_addr, uint16_t reg_num, uint32_t timeout);
rt_err_t mb_parallel_read_holding_register(uint8_t slave_addr, uint16_t reg_addr, uint16_t reg_num, uint32_t timeout);
rt_err_t mb_write_holding_register(enum serial_num serial_nu,uint8_t slave_addr, uint16_t reg_addr, uint16_t data, uint32_t timeout);
rt_err_t mb_rewrite_holding_register(enum serial_num serial_nu,uint8_t slave_addr, uint16_t reg_addr, uint16_t data, uint32_t timeout);
rt_err_t mb_reread_holding_register(enum serial_num serial_nu,uint8_t slave_addr, uint16_t reg_addr, uint16_t reg_num, uint32_t timeout);

rt_err_t mb_write_8_holding_register(enum serial_num serial_nu,uint8_t slave_addr, uint16_t reg_addr, uint16_t data[], uint32_t timeout);
rt_err_t mb_write_holding_register_2(uint8_t slave_addr, uint16_t reg_addr, uint16_t data[], uint32_t timeout);


#endif

