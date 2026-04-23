#include "modbus.h"
#include "crc16.h"
#include "rtthread.h"
#include "debug.h"
#include "uart.h"
#include "user_table.h"


void clear_rxbuffer(enum serial_num serial_nu);

uint16_t user_reg_hold_buf_2[MB_MASTER_TOTAL_SLAVE_NUM][M_REG_HOLDING_NREGS];
uint16_t user_reg_hold_buf[M_REG_HOLDING_NREGS];



/**
  * @brief  Modbus RTU 主站读取保持寄存器（Holding Register） 的实现函数。
	*					它通过串口向从站发送 03H 功能码命令，并接收返回数据，把读取到的寄存器值存入缓冲区。
  * @param  serial_nu: 		使用哪个串口
	*	@param	slave_addr: 	Modbus 从机地址
	*	@param	reg_addr: 		起始寄存器地址
	*	@param	reg_num:			要读取的寄存器数量
  * @param  timeout:			接收超时时间
  *          
  * @retval 
  */
rt_err_t mb_read_holding_register(enum serial_num serial_nu,uint8_t slave_addr, uint16_t reg_addr, uint16_t reg_num, uint32_t timeout)
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
		clear_rxbuffer(serial_nu);
    serial_send(serial_nu,cmd, sizeof(cmd));
    rx_length = serial_recv(serial_nu, rx_buffer, sizeof(rx_buffer),timeout);

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



/**
  * @brief  同时向多个从机发送 Modbus 03H 读取命令，然后统一等待，再逐个读取各串口返回数据。
	*	@param	slave_addr: 	Modbus 从机地址
	*	@param	reg_addr: 		起始寄存器地址
	*	@param	reg_num:			要读取的寄存器数量
  * @param  timeout:			接收超时时间
  *          
  * @retval 
  */
rt_err_t mb_parallel_read_holding_register(uint8_t slave_addr, uint16_t reg_addr, uint16_t reg_num, uint32_t timeout)
{
    uint16_t rx_length;
    uint8_t rx_buffer[RX_BUF_LEN];
    uint16_t crc_code;
    uint8_t cmd[] = {0x01, 0x03, 0x00, 0x00, 0x00, 0x01, 0x00, 0x00};
    rt_err_t res = RT_EOK;
    cmd[0] = slave_addr;
    cmd[2] = (uint8_t)(reg_addr >> 8);
    cmd[3] = (uint8_t)(reg_addr & 0x00ff);
    cmd[4] = (uint8_t)(reg_num >> 8);
    cmd[5] = (uint8_t)(reg_num & 0x00ff);
    crc_code = crc_16(cmd, sizeof(cmd) - 2);
    cmd[6] = (uint8_t)(crc_code & 0x00FF);
    cmd[7] = (uint8_t)(crc_code >> 8);

    uint16_t id = 1;
    do
    {
        clear_rxbuffer(slave[id].serial);
        serial_send(slave[id].serial,cmd, sizeof(cmd));
    } while(++id <= MB_MASTER_TOTAL_SLAVE_NUM);

    rt_thread_mdelay(timeout);
    id = 1;
    do
    {
        rx_length = serial_recv(slave[id].serial, rx_buffer, sizeof(rx_buffer),0);
        if (rx_length < 2)
        {
            log_develop(MB_INFO,"length error\n");
            res |= (1<<(id-1));
            continue;
        }

        if(rx_buffer[0] !=  slave_addr) {
            log_develop(MB_INFO,"addr error\n");
            res |= (1<<(id-1));
            continue;
        }
        crc_code = ((uint16_t)rx_buffer[rx_length - 1] << 8) | rx_buffer[rx_length - 2];
        if (crc_code != crc_16(rx_buffer, rx_length - 2)) {
            log_develop(MB_INFO,"crc error\n");
            res |= (1<<(id-1));
            continue;
        }
        if (rx_buffer[2] != 2 * reg_num) {
            res |= (1<<(id-1));
            continue;
        }
        for (uint8_t i = 0, j = 0; j < reg_num; i += 2, j += 1) {
            user_reg_hold_buf_2[id-1][reg_addr + j] = ((uint16_t)rx_buffer[i + 3] << 8) | (uint16_t)rx_buffer[i + 4];
            log_develop(MB_INFO,"ID:%d ",id-1);
            log_develop(MB_INFO, "%04x ", user_reg_hold_buf_2[j]);
        }
        log_develop(MB_INFO,"\n");
    } while(++id <= MB_MASTER_TOTAL_SLAVE_NUM);

    return  res;
}



/**
  * @brief  向多个串口上的多个设备，同时写入同一个寄存器地址，但每个设备写入的值可以不同。
	*	@param	slave_addr: 	Modbus 从机地址
	*	@param	reg_addr: 		要写入的寄存器地址
	*	@param	data[]:			每个设备对应写入的数据
  * @param  timeout:			等待回复时间
  * @retval 
  */
rt_err_t mb_write_holding_register_2(uint8_t slave_addr, uint16_t reg_addr, uint16_t data[], uint32_t timeout)
{
    uint16_t crc_code;
    uint8_t cmd[] = {0x01, 0x06, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
    rt_err_t res = RT_EOK;
    cmd[0] = slave_addr;
    cmd[2] = (uint8_t)(reg_addr >> 8);
    cmd[3] = (uint8_t)(reg_addr & 0x00FF);
 
    uint16_t id = 1;
    do
    {
				cmd[4] = (uint8_t)(data[id-1] >> 8);
				cmd[5] = (uint8_t)(data[id-1] & 0x00FF);
				crc_code = crc_16(cmd, sizeof(cmd) - 2);
				cmd[6] = (uint8_t)(crc_code & 0x00FF);
				cmd[7] = (uint8_t)(crc_code >> 8);
        clear_rxbuffer(slave[id].serial);
        serial_send(slave[id].serial,cmd, sizeof(cmd));
    } while(++id <= MB_MASTER_TOTAL_SLAVE_NUM);
    rt_thread_mdelay(timeout);

    uint8_t rx_buffer[8];
    uint16_t rx_length;

    id = 1;
    do
    {
        rx_length = serial_recv(slave[id].serial, rx_buffer, sizeof(rx_buffer),0);
        if (rx_length != sizeof(rx_buffer))
        {
            log_develop(MB_INFO,"length error\n");
            res |= (1<<(id-1));
            continue;
        }
        if(rx_buffer[0] !=  slave_addr) {
            log_develop(MB_INFO,"addr error\n");
            res |= (1<<(id-1));
            continue;
        }
        crc_code = ((uint16_t)rx_buffer[rx_length - 1] << 8) | rx_buffer[rx_length - 2];
        if (crc_code != crc_16(rx_buffer, rx_length - 2)) {
            log_develop(MB_INFO,"crc error\n");
            res |= (1<<(id-1));
            continue;
        }
    } while(++id <= MB_MASTER_TOTAL_SLAVE_NUM);

    return res;
}


/**
  * @brief  通过指定串口，向某个 Modbus 从机写入一个保持寄存器（06H 功能码），并等待从机确认。
	*	@param	serial_nu：		使用哪个串口
	*	@param	slave_addr: 	Modbus 从机地址
	*	@param	reg_addr: 		要写入的寄存器地址
	*	@param	data:					写入的数据
  * @param  timeout:			等待回复时间
  * @retval 
  */
rt_err_t mb_write_holding_register(enum serial_num serial_nu,uint8_t slave_addr, uint16_t reg_addr, uint16_t data, uint32_t timeout)
{
    uint16_t crc_code;
    uint8_t cmd[] = {0x01, 0x06, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
    rt_err_t res = RT_EOK;
    cmd[0] = slave_addr;
    cmd[2] = (uint8_t)(reg_addr >> 8);
    cmd[3] = (uint8_t)(reg_addr & 0x00FF);
    cmd[4] = (uint8_t)(data >> 8);
    cmd[5] = (uint8_t)(data & 0x00FF);
    crc_code = crc_16(cmd, sizeof(cmd) - 2);
    cmd[6] = (uint8_t)(crc_code & 0x00FF);
    cmd[7] = (uint8_t)(crc_code >> 8);
    log_develop(MB_INFO, "cmd: ");
    for (size_t i = 0; i < 8; i++) {
        log_develop(MB_INFO, "%02x ", cmd[i]);
    }
    log_develop(MB_INFO,"\n");
    // rs485_tx_mode();
    // rt_thread_mdelay(1);
    serial_send(serial_nu,cmd, sizeof(cmd));

    uint8_t rx_buffer[8];
    uint16_t rx_length;

    //  rs485_rx_mode();
    rx_length = serial_recv(serial_nu, rx_buffer, sizeof(rx_buffer),timeout);

    if (rx_length != sizeof(rx_buffer))
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
    return RT_EOK;

cmd_fail:
    return res;
}



/**
  * @brief  向指定从机，从某个起始寄存器地址开始，一次连续写入 8 个寄存器（共16字节数据）。
	*	@param	serial_nu：		使用哪个串口
	*	@param	slave_addr: 	Modbus 从机地址
	*	@param	reg_addr: 		要写入的寄存器地址
	*	@param	data[]:				要写入的8个寄存器值
  * @param  timeout:			等待回复时间
  * @retval 
  */
rt_err_t mb_write_8_holding_register(enum serial_num serial_nu,uint8_t slave_addr, uint16_t reg_addr, uint16_t data[], uint32_t timeout)
{
    uint16_t crc_code;
    uint8_t cmd[25] = {0x01, 0x10, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
    rt_err_t res = RT_EOK;
    cmd[0] = slave_addr;
    cmd[1] =  0x10;
    cmd[2] = (uint8_t)(reg_addr >> 8);
    cmd[3] = (uint8_t)(reg_addr & 0x00FF);
    cmd[4] =	0x00;
    cmd[5] = 	0x08;
    cmd[6] = 	0x10;
    cmd[7] = (uint8_t)(data[0] >> 8);
    cmd[8] = (uint8_t)(data[0] & 0x00FF);
    cmd[9] = (uint8_t)(data[1] >> 8);
    cmd[10] = (uint8_t)(data[1] & 0x00FF);
    cmd[11] = (uint8_t)(data[2] >> 8);
    cmd[12] = (uint8_t)(data[2] & 0x00FF);
    cmd[13] = (uint8_t)(data[3] >> 8);
    cmd[14] = (uint8_t)(data[3] & 0x00FF);
    cmd[15] = (uint8_t)(data[4] >> 8);
    cmd[16] = (uint8_t)(data[4] & 0x00FF);
    cmd[17] = (uint8_t)(data[5] >> 8);
    cmd[18] = (uint8_t)(data[5] & 0x00FF);
    cmd[19] = (uint8_t)(data[6] >> 8);
    cmd[20] = (uint8_t)(data[6] & 0x00FF);
    cmd[21] = (uint8_t)(data[7] >> 8);
    cmd[22] = (uint8_t)(data[7] & 0x00FF);

    crc_code = crc_16(cmd, sizeof(cmd) - 2);
    cmd[23] = (uint8_t)(crc_code & 0x00FF);
    cmd[24] = (uint8_t)(crc_code >> 8);
    log_develop(MB_INFO, "cmd: ");
    for (size_t i = 0; i < 8; i++) {
        log_develop(MB_INFO, "%02x ", cmd[i]);
    }
    log_develop(MB_INFO,"\n");
    // rs485_tx_mode();
    // rt_thread_mdelay(1);
    serial_send(serial_nu,cmd, sizeof(cmd));

    uint8_t rx_buffer[8];
    uint16_t rx_length;

    //  rs485_rx_mode();
    rx_length = serial_recv(serial_nu, rx_buffer, sizeof(rx_buffer),timeout);

    if (rx_length != sizeof(rx_buffer))
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
    return RT_EOK;

cmd_fail:
    return res;
}



//rt_err_t mb_reread_holding_register(enum serial_num serial_nu,uint8_t slave_addr, uint16_t reg_addr, uint16_t reg_num, uint32_t timeout)
//{
//		rt_err_t result = RT_EOK;
//    uint16_t fail_cnt = 0;
//    do
//    {
//        result = mb_parallel_read_holding_register(serial_nu, slave_addr,  reg_addr, reg_num, timeout);
//        if(result != RT_EOK)
//        {
//			rt_thread_mdelay(50);
//            fail_cnt ++;
//        }
//        else
//        {
//            break;
//        }
//    } while(fail_cnt < 1);

//    if(fail_cnt >= 1)
//    {
//        result = RT_ERROR;
//        goto cmd_fail;
//    }

//    return RT_EOK;

//cmd_fail:
//    return result;
//}





rt_err_t mb_rewrite_holding_register(enum serial_num serial_nu,uint8_t slave_addr, uint16_t reg_addr, uint16_t data, uint32_t timeout)
{
    rt_err_t result = RT_EOK;
    uint16_t fail_cnt = 0;
    do
    {
        result = mb_write_holding_register(serial_nu,slave_addr,reg_addr,data,timeout);
        if(result != RT_EOK)
        {
            rt_thread_mdelay(50);
            fail_cnt ++;
        }
        else
        {
            break;
        }
    } while(fail_cnt < 1);

    if(fail_cnt >= 1)
    {
        result = RT_ERROR;
        goto cmd_fail;
    }

    return RT_EOK;

cmd_fail:
    return result;
}


rt_err_t mb_parallel_read(uint16_t reg_addr,uint16_t rx_buf[])
{



}



