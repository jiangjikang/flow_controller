#include "modbus.h"
#include "crc16.h"
#include "rtthread.h"
#include "debug.h"


void clear_rxbuffer(enum serial serial_num);

uint16_t user_reg_hold_buf_2[MB_MASTER_TOTAL_SLAVE_NUM][M_REG_HOLDING_NREGS];
uint16_t user_reg_hold_buf[M_REG_HOLDING_NREGS];


/**
 * @brief  读取指定从站保持寄存器（Modbus 0x03）
 *
 * 通过指定串口向从站发送读取命令，接收并校验响应数据，
 * 成功后保存到 user_reg_hold_buf[]。
 *
 * @param[in] serial_num  串口号
 * @param[in] slave_addr  从站地址
 * @param[in] reg_addr    起始寄存器地址
 * @param[in] reg_num     读取寄存器数量
 * @param[in] timeout     接收超时时间，单位 ms
 *
 * @return rt_err_t
 * @retval RT_EOK    成功
 * @retval RT_ERROR  失败
 *
 * @note
 * 校验内容：长度、地址、CRC、字节数。
 */
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
		clear_rxbuffer(serial_num);
    serial_send(serial_num,cmd, sizeof(cmd));
		
		
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
			log_develop(MB_INFO,"ID:%d ",serial_num);
      log_develop(MB_INFO, "%04x ", user_reg_hold_buf[j]);
    }
    log_develop(MB_INFO,"\n");

    return RT_EOK;

cmd_fail:
    return  res;
}




/**
 * @brief  并行读取多个从站保持寄存器（Modbus 0x03）
 *
 * 向所有从站同时发送读取命令，等待指定时间后依次接收响应，
 * 校验数据后保存到 user_reg_hold_buf_2[][]。
 *
 * @param[in] slave_addr  从站地址
 * @param[in] reg_addr    起始寄存器地址
 * @param[in] reg_num     读取寄存器数量
 * @param[in] timeout     等待响应时间，单位 ms
 *
 * @return rt_err_t
 * @retval 0      全部成功
 * @retval other  位掩码错误码，bit0表示1号从站失败，bit1表示2号从站失败
 *
 * @note
 * 校验内容：长度、地址、CRC、字节数。
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
		enum serial number = SERIAL_3;
    do
    {
        clear_rxbuffer(number);
        serial_send(number,cmd, sizeof(cmd));
				number = (enum serial)SERIAL_3 + 1;
    } while(++id <= MB_MASTER_TOTAL_SLAVE_NUM);

    rt_thread_mdelay(timeout);
    id = 1;
    do
    {
        rx_length = serial_recv(number, rx_buffer, sizeof(rx_buffer),0);
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
 * @brief  并行写入多个从站单个保持寄存器（Modbus 0x06）
 *
 * 向各从站发送写单寄存器命令，每个从站写入同一寄存器地址，
 * 写入数据由 data[] 提供。发送完成后等待指定时间，再依次读取响应并校验。
 *
 * @param[in] slave_addr  从站地址
 * @param[in] reg_addr    目标保持寄存器地址
 * @param[in] data[]      各从站写入数据数组，data[0]对应1号从站
 * @param[in] timeout     等待响应时间，单位 ms
 *
 * @return rt_err_t
 * @retval 0      全部从站写入成功
 * @retval other  位掩码错误码，bit0表示1号从站失败，bit1表示2号从站失败
 *
 * @note
 * 仅校验响应长度、地址及 CRC。
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
	enum serial number = SERIAL_3;
	do
	{
		
		cmd[4] = (uint8_t)(data[id-1] >> 8);
		cmd[5] = (uint8_t)(data[id-1] & 0x00FF);
		crc_code = crc_16(cmd, sizeof(cmd) - 2);
		cmd[6] = (uint8_t)(crc_code & 0x00FF);
		cmd[7] = (uint8_t)(crc_code >> 8);
		clear_rxbuffer(number);
		serial_send(number,cmd, sizeof(cmd));
		number = (enum serial)SERIAL_3 + 1;
	} while(++id <= MB_MASTER_TOTAL_SLAVE_NUM);
	rt_thread_mdelay(timeout);
	
	uint8_t rx_buffer[8];
	uint16_t rx_length;
	
	id = 1;
	do
    {
        rx_length = serial_recv(number, rx_buffer, sizeof(rx_buffer), 0);
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
 * @brief  写入指定从站单个保持寄存器（Modbus 0x06）
 *
 * 通过指定串口向从站发送写单寄存器命令，接收并校验响应帧。
 *
 * @param[in] serial_num   串口号
 * @param[in] slave_addr  从站地址
 * @param[in] reg_addr    寄存器地址
 * @param[in] data        写入数据
 * @param[in] timeout     接收超时时间，单位 ms
 *
 * @return rt_err_t
 * @retval RT_EOK    写入成功
 * @retval RT_ERROR  写入失败
 *
 * @note
 * 校验内容：响应长度、地址、CRC。
 */
rt_err_t mb_write_holding_register(enum serial serial_num,uint8_t slave_addr, uint16_t reg_addr, uint16_t data, uint32_t timeout)
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
    serial_send(serial_num,cmd, sizeof(cmd));

    uint8_t rx_buffer[8];
    uint8_t rx_length;

    //  rs485_rx_mode();
    uint16_t total = 0;

	while(total < sizeof(rx_buffer))
	{
			total += serial_recv(serial_num,
													 &rx_buffer[total],
													 sizeof(rx_buffer) - total,
													 timeout);
	}

	if(total != sizeof(rx_buffer))
	{
			log_develop(MB_INFO,"length error total=%d\n", total);
	}

//    if (rx_length != sizeof(rx_buffer))
//    {
//        log_develop(MB_INFO,"length error\n");
//        res = RT_ERROR;
//        goto cmd_fail;
//    }
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
 * @brief  写入指定从站连续8个保持寄存器（Modbus 0x10）
 *
 * 通过指定串口向从站发送写多个寄存器命令，
 * 连续写入8个保持寄存器，并校验响应帧。
 *
 * @param[in] serial_num  串口号
 * @param[in] slave_addr  从站地址
 * @param[in] reg_addr    起始寄存器地址
 * @param[in] data[]      写入数据数组，共8个寄存器数据
 * @param[in] timeout     接收超时时间，单位 ms
 *
 * @return rt_err_t
 * @retval RT_EOK    写入成功
 * @retval RT_ERROR  写入失败
 *
 * @note
 * 校验内容：响应长度、地址、CRC。
 */
rt_err_t mb_write_8_holding_register(enum serial serial_num,uint8_t slave_addr, uint16_t reg_addr, uint16_t data[], uint32_t timeout)
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
		
    serial_send(serial_num,cmd, sizeof(cmd));

    uint8_t rx_buffer[8];
    uint16_t rx_length;
		
    rx_length = serial_recv(serial_num, rx_buffer, sizeof(rx_buffer),timeout);

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





/**
 * @brief  重试写入单个保持寄存器
 *
 * 调用 mb_write_holding_register() 写入寄存器，
 * 若失败则延时后重试，达到最大次数仍失败则返回错误。
 *
 * @param[in] serial_num  串口号
 * @param[in] slave_addr  从站地址
 * @param[in] reg_addr    寄存器地址
 * @param[in] data        写入数据
 * @param[in] timeout     单次通信超时时间，单位 ms
 *
 * @return rt_err_t
 * @retval RT_EOK    写入成功
 * @retval RT_ERROR  重试后仍失败
 *
 * @note
 * 当前最多重试1次，失败后延时50ms。
 */
rt_err_t mb_rewrite_holding_register(enum serial serial_num,uint8_t slave_addr, uint16_t reg_addr, uint16_t data, uint32_t timeout)
{
    rt_err_t result = RT_EOK;
    uint16_t fail_cnt = 0;
    do
    {
        result = mb_write_holding_register(serial_num,slave_addr,reg_addr,data,timeout);
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









