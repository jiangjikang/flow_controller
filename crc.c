#include "crc.h"


rt_uint16_t modbus_crc16(rt_uint8_t *buf, rt_uint16_t len)
{
    rt_uint16_t crc = 0xFFFF;

    for (rt_uint16_t pos = 0; pos < len; pos++)
    {
        crc ^= (rt_uint16_t)buf[pos];  // 与当前字节异或

        for (rt_uint8_t i = 0; i < 8; i++)
        {
            if (crc & 0x0001)
            {
                crc >>= 1;
                crc ^= 0xA001;
            }
            else
            {
                crc >>= 1;
            }
        }
    }

    return crc;
}