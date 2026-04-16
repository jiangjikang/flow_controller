#include "modbus.h"
#include "crc.h"

#define MODBUS_MAX_REG  110

rt_uint16_t modbus_reg_buf[MODBUS_MAX_REG];
rt_uint16_t modbus_reg_len = 0;

volatile rt_uint8_t modbus_data_ready = 0;

// 读保持寄存器(0x03)
void modbus_read_holding_registers(rt_uint8_t slave,
                                   rt_uint16_t addr,
                                   rt_uint16_t num)
{
    rt_uint8_t tx_buf[8];

    tx_buf[0] = slave;
    tx_buf[1] = 0x03;
    tx_buf[2] = addr >> 8;
    tx_buf[3] = addr & 0xFF;
    tx_buf[4] = num >> 8;
    tx_buf[5] = num & 0xFF;

    rt_uint16_t crc = modbus_crc16(tx_buf, 6);

    tx_buf[6] = crc & 0xFF;        // CRC低字节
    tx_buf[7] = crc >> 8;          // CRC高字节

    uart_send(tx_buf, 8);
		
}


// 写单个寄存器(0x06)  
void modbus_write_single_register(rt_uint8_t slave,
                                  rt_uint16_t addr,
                                  rt_uint16_t value)
{
    rt_uint8_t tx_buf[8];

    tx_buf[0] = slave;
    tx_buf[1] = 0x06;
    tx_buf[2] = addr >> 8;
    tx_buf[3] = addr & 0xFF;
    tx_buf[4] = value >> 8;
    tx_buf[5] = value & 0xFF;

    rt_uint16_t crc = modbus_crc16(tx_buf, 6);

    tx_buf[6] = crc & 0xFF;
    tx_buf[7] = crc >> 8;

    uart_send(tx_buf, 8);
		
	 
}
MSH_CMD_EXPORT(modbus_write_single_register, start);

// 写多个寄存器(0x10)
void modbus_write_multiple_registers(rt_uint8_t slave,
                                     rt_uint16_t addr,
                                     rt_uint16_t num,
                                     rt_uint16_t *data)
{
    rt_uint8_t tx_buf[256];   //根据实际情况调整
    rt_uint16_t i;

    tx_buf[0] = slave;
    tx_buf[1] = 0x10;
    tx_buf[2] = addr >> 8;
    tx_buf[3] = addr & 0xFF;
    tx_buf[4] = num >> 8;
    tx_buf[5] = num & 0xFF;
    tx_buf[6] = num * 2;   // 字节数

    /* 填充数据 */
    for (i = 0; i < num; i++)
    {
        tx_buf[7 + i * 2] = data[i] >> 8;
        tx_buf[8 + i * 2] = data[i] & 0xFF;
    }

    rt_uint16_t len = 7 + num * 2;

    rt_uint16_t crc = modbus_crc16(tx_buf, len);

    tx_buf[len]     = crc & 0xFF;
    tx_buf[len + 1] = crc >> 8;

    uart_send(tx_buf, len + 2);
}








// 数据解析
int modbus_parse(rt_uint8_t *rx_buf, rt_uint16_t len)
{
    if (len < 5)
        return MODBUS_ERROR;

    /* ================= CRC 校验 ================= */
    rt_uint16_t crc_calc = modbus_crc16(rx_buf, len - 2);
    rt_uint16_t crc_recv = rx_buf[len - 2] | (rx_buf[len - 1] << 8);

    if (crc_calc != crc_recv)
    {
        return MODBUS_CRC_ERROR;
    }

    /* ================= 基本字段 ================= */
    rt_uint8_t slave_addr = rx_buf[0];
    rt_uint8_t func_code  = rx_buf[1];

    /* ================= 异常响应 ================= */
    if (func_code & 0x80)
    {
        return MODBUS_EXCEPT;
    }

    /* ================= 正常解析 ================= */
    switch (func_code)
    {
        /* ===== 0x03 读保持寄存器 ===== */
        case 0x03:
        {
            rt_uint8_t byte_count = rx_buf[2];
            rt_uint16_t reg_num = byte_count / 2;

            if (reg_num > MODBUS_MAX_REG)
                reg_num = MODBUS_MAX_REG;

            for (int i = 0; i < reg_num; i++)
            {
                modbus_reg_buf[i] =
                    (rx_buf[3 + i * 2] << 8) |
                     rx_buf[4 + i * 2];
            }

            modbus_reg_len = reg_num;   // 更新有效长度
        }
        break;

        /* ===== 0x06 写单寄存器 ===== */
        case 0x06:
        {
            rt_uint16_t addr  = (rx_buf[2] << 8) | rx_buf[3];
            rt_uint16_t value = (rx_buf[4] << 8) | rx_buf[5];

            if (addr < MODBUS_MAX_REG)
            {
                modbus_reg_buf[addr] = value;
            }
        }
        break;

        /* ===== 0x10 写多个寄存器 ===== */
        case 0x10:
        {
            // 一般只确认成功即可
        }
        break;

        default:
            return MODBUS_ERROR;
    }

    return MODBUS_OK;
}






