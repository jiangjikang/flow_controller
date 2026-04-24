#include "modbus.h"
#include "crc16.h"
#include "rtthread.h"


void clear_rxbuffer(enum serial serial_num);

rt_err_t mb_read_holding_register(enum serial serial_num,uint8_t slave_addr, uint16_t reg_addr, uint16_t reg_num, uint32_t timeout)
{
    uint16_t rx_length;
    uint8_t rx_buffer[RX_BUF_LEN];
    uint16_t crc_code;
    uint8_t cmd[] = {0x01, 0x03, 0x00, 0x00, 0x00, 0x01, 0x00, 0x00};
    rt_err_t res = RT_EOK;
    if (reg_num > M_REG_HOLDING_NREGS)
        goto cmd_fail;
    cmd[0] = slave_addr;
    cmd[2] = (uint8_t)(reg_addr >> 8);
    cmd[3] = (uint8_t)(reg_addr & 0x00ff);
    cmd[4] = (uint8_t)(reg_num >> 8);
    cmd[5] = (uint8_t)(reg_num & 0x00ff);
    crc_code = crc_16(cmd, sizeof(cmd) - 2);
    cmd[6] = (uint8_t)(crc_code & 0x00FF);
    cmd[7] = (uint8_t)(crc_code >> 8);
    // rs485_tx_mode();
    // rt_thread_mdelay(1);
		clear_rxbuffer(serial_num);
    serial_send(serial_num,cmd, sizeof(cmd));
    //  rs485_rx_mode();
    rx_length = serial_recv(serial_num, rx_buffer, sizeof(rx_buffer),timeout);

    if (rx_length < 2)
    {
			log_develop(MB_INFO,"length error\n");
			res = RT_ERROR;
			goto cmd_fail;
    }

    if(rx_buffer[0] !=  slave_addr) {
			log_develop(MB_INFO,"addr error\n");
			res = RT_ERROR;
			goto cmd_fail;
    }
    crc_code = ((uint16_t)rx_buffer[rx_length - 1] << 8) | rx_buffer[rx_length - 2];
    if (crc_code != crc_16(rx_buffer, rx_length - 2)) {
        log_develop(MB_INFO,"crc error\n");
        res = RT_ERROR;
        goto cmd_fail;
    }
    if (rx_buffer[2] != 2 * reg_num) {
        res = RT_ERROR;
        goto cmd_fail;
    }
    for (uint8_t i = 0, j = 0; j < reg_num; i += 2, j += 1) {
      user_reg_hold_buf[reg_addr + j] = ((uint16_t)rx_buffer[i + 3] << 8) | (uint16_t)rx_buffer[i + 4];
			log_develop(MB_INFO,"ID:%d ",serial_nu-1);
      log_develop(MB_INFO, "%04x ", user_reg_hold_buf[j]);
    }
    log_develop(MB_INFO,"\n");

    return RT_EOK;

cmd_fail:
    return  res;
}



