#include "calibration.h"
#include <rtdevice.h>
#include <board.h>
#include "debug.h"
#include "uart.h"
#include "modbus.h"
#include "crc16.h"
#include "analog.h"
#include "user_table.h"
#include "env.h"
#include "pwm.h"


#define FLOW_START_ADDR  51			// 
#define FREQ_START_ADDR  63
#define AN_FLOW_START_ADDR  (75)  	// 流量寄存器地址偏移
#define AN_ZKB_START_ADDR  (81)		// 占空比寄存器地址偏移

#define REG_AN_TEST_POINT_ADDR 87	// 模拟量测试点寄存器地址
#define REG_CAL_ENABLE_ADDR 50		// 标定使能寄存器地址

#define REG_FLOW_ADDR 0
#define REG_FREQ_ADDR 46       // 46   1
#define REG_TEMP_ADDR 2

#define REG_OUT_TYPR_ADDR   10
#define REG_RANGE_L_ADDR   11
#define REG_RANGE_H_ADDR   12
#define REG_AN_RANGE_L_ADDR  13
#define REG_AN_RANGE_H_ADDR  14

#define CAL_POINT_NUM_MAX 12
#define AN_CAL_POINT_NUM_MAX 6


#define SERIAL_NUM  8
#define SERIAL_NUM_START_ADDR  10


rt_mutex_t dynamic_mutex = RT_NULL;

static rt_uint32_t rx_length = 0;
static rt_uint32_t tx_length = 0;
static uint8_t rx_buffer[256];
static uint8_t tx_buffer[256];
static uint16_t ret_value[MB_MASTER_TOTAL_SLAVE_NUM+10];
static uint16_t command_param[MB_MASTER_TOTAL_SLAVE_NUM+10];



static int16_t abs_int16(int16_t a)
{
    if(a >= 0)
        return a;
    else
        return -a;
}

static int16_t sign(int16_t i)
{
    if(i%2 == 0)
        return -1;
    else
        return 1;
}

rt_err_t sensor_analog_calib(enum analog_type type,uint16_t slaveid,uint16_t point,uint16_t std_flow,uint16_t setval)
{
    uint16_t left = 40;
    uint16_t right = 800;
    uint16_t mid = 0;
    uint16_t id = slaveid;
    rt_err_t result = RT_EOK;
    uint16_t analog = 0;
    uint16_t fail_cnt = 0;

    uint16_t diff_thred = 4;

    if(type == CURRENT_TYPE)
    {
        diff_thred = 25;
    }
         
    result = mb_rewrite_holding_register(slave[id].serial,SENSOR_ID,AN_FLOW_START_ADDR + point-1,std_flow, MB_TIMEOUT);
    if(result != RT_EOK)
    {
        goto cmd_fail;
    }

    result = mb_rewrite_holding_register(slave[id].serial,SENSOR_ID,REG_AN_TEST_POINT_ADDR,std_flow, MB_TIMEOUT);
    if(result != RT_EOK)
    {
        goto cmd_fail;
    }

    while(left  < right) {

        mid = (left + right)/2;

        result = mb_rewrite_holding_register(slave[id].serial,SENSOR_ID,AN_ZKB_START_ADDR + point - 1,mid, MB_TIMEOUT);
        if(result != RT_EOK)
        {
            result = RT_ERROR;
            goto cmd_fail;
        }
        rt_thread_mdelay(50);
        analog = adc_read_voltage(slave[id].channel);
        analog = analog_calib(type,analog);
        log_develop(AN_INFO,"analog=%d\n",analog);

        if(analog >= setval + diff_thred) {
            right = mid-1;
        }
        else if(analog <= setval - diff_thred) {
            left = mid+1;
        }
        else {
            break;
        }
    }
    if(right <= left)
    {

        if(abs_int16((int16_t)(analog - setval)) > 500)
        {
            result = RT_ERROR;
            goto cmd_fail;
        }
        else
        {
            uint16_t diff_min = 0xffff;
            uint16_t best_zkb = 0;
            uint16_t diff;
            for(int16_t zkb = mid,i = 1,k=0; i < 11; k++,i += k/2,zkb = mid + i*sign(k))
            {
                if(zkb < 0)
                    continue;

                result = mb_rewrite_holding_register(slave[id].serial,SENSOR_ID,AN_ZKB_START_ADDR + point - 1,zkb, MB_TIMEOUT);
                if(result != RT_EOK)
                {
                    result = RT_ERROR;
                    goto cmd_fail;
                }
                rt_thread_mdelay(50);
                analog = adc_read_voltage(slave[id].channel);
                analog = analog_calib(type,analog);
                log_develop(AN_INFO,"analog=%d\n",analog);

                diff = abs_int16((int16_t)(analog - setval));

                if(diff < diff_min) {
                    diff_min = diff;
                    best_zkb = zkb;
                }

                if( diff <= diff_thred)
                {
                    break;
                }

                if(i >= 10)
                {
                    mb_rewrite_holding_register(slave[id].serial,SENSOR_ID,AN_ZKB_START_ADDR + point - 1,best_zkb, MB_TIMEOUT);
//					rt_thread_mdelay(50);
//					analog = adc_read_voltage(type,slave[id].channel);
//					log_develop(AN_INFO,"analog=%d\n",analog);
                    result = RT_ERROR;
                    goto cmd_fail;
                }

            }
        }
    }
    return RT_EOK;

cmd_fail:
    return result;
}




void read_handler(uint16_t cmd_type,uint16_t reg_addr)
{
    uint16_t crc_code;
    rt_err_t result;
    tx_length = 0;
    tx_buffer[tx_length] = BOARD_ID;
    tx_buffer[tx_length+1] = 0x03;
    tx_buffer[tx_length+2] = 0;		// 长度待确定
    tx_length += 3;

    tx_buffer[tx_length] = cmd_type>>8;
    tx_buffer[tx_length+1] = cmd_type&0xff;
    tx_length += 2;

    result = mb_parallel_read_holding_register(SENSOR_ID,reg_addr,1, MB_TIMEOUT);
    uint16_t id = 1;
    do
    {
        if(result & (1<<(id-1)))
        {
            ret_value[id-1] = UINT16_MAX;
            log_develop(CAL_INFO,"read error for cmd %04x,and sensor id is %d\n",cmd_type,id);
        }
        else
        {
            ret_value[id-1] = user_reg_hold_buf_2[id-1][reg_addr];
        }
    } while(++id <= MB_MASTER_TOTAL_SLAVE_NUM);

    uint16_t i = 0;
    do
    {
        tx_buffer[tx_length] = ret_value[i]>>8;
        tx_buffer[tx_length+1] = ret_value[i]&0xff;
        tx_length += 2;
    }
    while(++i < MB_MASTER_TOTAL_SLAVE_NUM);

    tx_buffer[2] = tx_length - 3;

    crc_code = crc_16(tx_buffer, tx_length);
    tx_buffer[tx_length] = (uint8_t)(crc_code & 0x00FF);
    tx_buffer[tx_length+1] = (uint8_t)(crc_code >> 8);
    tx_length += 2;

    log_develop(MB_INFO, "tx:%04x ", ret_value[0]);
    log_develop(MB_INFO, "%04x ", ret_value[1]);
    log_develop(MB_INFO, "%04x ", ret_value[2]);
    log_develop(MB_INFO, "\n");
    serial_send(SERIAL_1,&tx_buffer[0],tx_length);
}


rt_err_t read_sensor_flow_rs485(uint16_t flow_arr[],uint16_t len)
{
    rt_err_t result;
    result = mb_parallel_read_holding_register(SENSOR_ID,REG_FLOW_ADDR,1, MB_TIMEOUT);
    uint16_t id = 1;
    do
    {
        if(result & (1<<(id-1)))
        {
            flow_arr[id-1] = UINT16_MAX;
            log_develop(CAL_INFO,"read error for cmd %04x,and sensor id is %d\n",id);
        }
        else
        {
            flow_arr[id-1] = user_reg_hold_buf_2[id-1][REG_FLOW_ADDR];
        }
    } while(++id <= MB_MASTER_TOTAL_SLAVE_NUM);


    return RT_EOK;
}

rt_err_t read_sensor_flow_an(uint16_t output_channel,uint16_t flow_arr[],uint16_t len)
{
    uint16_t meas_range_l = sysEnv.getEnvVar(MEAS_RANGE_L);
    uint16_t meas_range_h = sysEnv.getEnvVar(MEAS_RANGE_H);

    uint16_t an_range_l;
    uint16_t an_range_h;

    if(output_channel == 1 )
    {
        an_range_l = sysEnv.getEnvVar(CH1_AN_RANGE_L);
        an_range_h = sysEnv.getEnvVar(CH1_AN_RANGE_H);
    }
    else if(output_channel == 2)
    {
        an_range_l = sysEnv.getEnvVar(CH2_AN_RANGE_L);
        an_range_h = sysEnv.getEnvVar(CH2_AN_RANGE_H);
    }

    uint16_t volt;
    int16_t flow;

    uint16_t id = 1;
    do
    {
        volt = get_analog_value_calib(slave[id].channel);
        log_develop(CAL_INFO,"analog=%d\n",volt);
        if(volt < 10)
        {
            log_develop(CAL_INFO,"read flow analog failed\n");
            flow = UINT16_MAX;
        }
        else
        {
            flow = (int32_t)(volt - an_range_l)*(meas_range_h - meas_range_l)/(an_range_h - an_range_l) + meas_range_l;
            if(flow < 0)
            {
                flow = 0;
            }
            log_develop(CAL_INFO,"flow=%d\n",flow);
        }
        flow_arr[id-1] = flow;

    } while(++id <= MB_MASTER_TOTAL_SLAVE_NUM);


    return RT_EOK;
}


rt_err_t read_sensor_flow_freq(uint16_t output_channel,uint16_t flow_arr[],uint16_t len)
{
    uint16_t meas_range_l = sysEnv.getEnvVar(MEAS_RANGE_L);
    uint16_t meas_range_h = sysEnv.getEnvVar(MEAS_RANGE_H);
    uint16_t an_range_l;
    uint16_t an_range_h;

    if(output_channel == 1 )
    {
        an_range_l = sysEnv.getEnvVar(CH1_AN_RANGE_L);
        an_range_h = sysEnv.getEnvVar(CH1_AN_RANGE_H);
    }
    else if(output_channel == 2)
    {
        an_range_l = sysEnv.getEnvVar(CH2_AN_RANGE_L);
        an_range_h = sysEnv.getEnvVar(CH2_AN_RANGE_H);
    }

    uint16_t pwm_cnt[MB_MASTER_TOTAL_SLAVE_NUM];

    if(get_pwm_count(pwm_cnt) == RT_EOK)
    {
        uint16_t id = 1;
        do
        {
            int16_t flow;
            flow = (float)(pwm_cnt[id-1] - an_range_l)*(meas_range_h - meas_range_l)/(an_range_h - an_range_l);
            if(flow < 0)
            {
                flow = 0;
            }
            flow_arr[id-1] = flow;
        } while(++id <= MB_MASTER_TOTAL_SLAVE_NUM);
    }
    return RT_EOK;
}


rt_err_t read_sensor_flow_module(uint16_t output_channel,uint8_t an_type,uint16_t flow_arr[],uint16_t len)
{
    rt_err_t result;
    uint16_t voltage[MB_MASTER_TOTAL_SLAVE_NUM];
    int16_t flow;
    uint16_t meas_range_l = sysEnv.getEnvVar(MEAS_RANGE_L);
    uint16_t meas_range_h = sysEnv.getEnvVar(MEAS_RANGE_H);

    uint16_t an_range_l;
    uint16_t an_range_h;
    if(output_channel == 1 )
    {
        an_range_l = sysEnv.getEnvVar(CH1_AN_RANGE_L);
        an_range_h = sysEnv.getEnvVar(CH1_AN_RANGE_H);
    }
    else if(output_channel == 2)
    {
        an_range_l = sysEnv.getEnvVar(CH2_AN_RANGE_L);
        an_range_h = sysEnv.getEnvVar(CH2_AN_RANGE_H);
    }

    uint8_t addr = 1;
    if(an_type == VOLT_TYPE)
    {
        addr = AN_VOLT_MODULE_ID;
    }
    else if(an_type == CURRENT_TYPE)
    {
        addr = AN_CURRENT_MODULE_ID;
    }

    result = mb_read_holding_register(SERIAL_5,addr,0,ANLONG_CHANNEL_NUM,MB_TIMEOUT);
    if(result != RT_EOK)
    {
        rt_memset(&flow_arr[0],0xff,ANLONG_CHANNEL_NUM*sizeof(uint16_t));
        log_develop(CAL_INFO,"read analog_module error\n");
        return RT_ERROR;
    }
    else
    {
        rt_memcpy(&voltage[0],&user_reg_hold_buf[0],ANLONG_CHANNEL_NUM*sizeof(uint16_t));
    }

    uint16_t id = 1;
    do
    {
        uint8_t decimal_num = (voltage[id-1]/10000)%10;
        voltage[id-1] %= 10000;
        if(decimal_num == 2)
        {
            voltage[id-1] *= 10;
        }
        if(an_type == CURRENT_TYPE)
        {
            voltage[id-1] = (uint32_t)1000*voltage[id-1]/249;
        }
        flow = (int32_t)(voltage[id-1] - an_range_l)*(meas_range_h - meas_range_l)/(an_range_h - an_range_l) + meas_range_l;
        if(flow < 0)
        {
            flow = 0;
        }
        log_develop(CAL_INFO,"flow=%d\n",flow);
        flow_arr[id-1] = flow;
    } while(++id <= MB_MASTER_TOTAL_SLAVE_NUM);
    return RT_EOK;
}

void read_sensor_flow_handler(uint16_t cmd_type,uint16_t output1_type,uint16_t output2_type)
{
    uint16_t crc_code;
    tx_length = 0;
    tx_buffer[tx_length] = BOARD_ID;
    tx_buffer[tx_length+1] = 0x03;
    tx_buffer[tx_length+2] = 0;    // 长度待确定
    tx_length += 3;

    tx_buffer[tx_length] = cmd_type>>8;
    tx_buffer[tx_length+1] = cmd_type&0xff;
    tx_length += 2;

    uint16_t ch1_an_range_h = sysEnv.getEnvVar(CH1_AN_RANGE_H);

    if(output1_type == VOLT_TYPE && ch1_an_range_h < 5000)
    {
        read_sensor_flow_an(1,&ret_value[0],MB_MASTER_TOTAL_SLAVE_NUM);
    }
    else if(output1_type == VOLT_TYPE && ch1_an_range_h >= 5000)
    {
        read_sensor_flow_module(1,output1_type,&ret_value[0],MB_MASTER_TOTAL_SLAVE_NUM);
    }
    else if(output1_type == CURRENT_TYPE)
    {
        read_sensor_flow_module(1,output1_type,&ret_value[0],MB_MASTER_TOTAL_SLAVE_NUM);
    }
    else if(output1_type == PWM_TYPE)
    {
        read_sensor_flow_freq(1,&ret_value[0],MB_MASTER_TOTAL_SLAVE_NUM);
    }
    else if(output1_type == UART_TYPE  || output1_type == RS485_TYPE)
    {
        read_sensor_flow_rs485(&ret_value[0],MB_MASTER_TOTAL_SLAVE_NUM);
    }
    else
    {
        rt_memset(&ret_value[0],0xff,MB_MASTER_TOTAL_SLAVE_NUM*2);
    }

    uint16_t i = 0;
    do
    {
        tx_buffer[tx_length] = ret_value[i]>>8;
        tx_buffer[tx_length+1] = ret_value[i]&0xff;
        tx_length += 2;
    }
    while(++i < MB_MASTER_TOTAL_SLAVE_NUM);

    uint16_t ch2_an_range_h = sysEnv.getEnvVar(CH2_AN_RANGE_H);

    if(output2_type == VOLT_TYPE && ch2_an_range_h < 5000)
    {
        read_sensor_flow_an(2,&ret_value[0],MB_MASTER_TOTAL_SLAVE_NUM);
    }
    else if(output2_type == VOLT_TYPE && ch2_an_range_h >= 5000)
    {
        read_sensor_flow_module(2,output2_type,&ret_value[0],MB_MASTER_TOTAL_SLAVE_NUM);
    }
    else if(output2_type == CURRENT_TYPE)
    {
        read_sensor_flow_module(2,output2_type,&ret_value[0],MB_MASTER_TOTAL_SLAVE_NUM);
    }
    else if(output2_type == PWM_TYPE)
    {
        read_sensor_flow_freq(2,&ret_value[0],MB_MASTER_TOTAL_SLAVE_NUM);
    }
    else if(output2_type == UART_TYPE  || output2_type == RS485_TYPE)
    {
        read_sensor_flow_rs485(&ret_value[0],MB_MASTER_TOTAL_SLAVE_NUM);
    }
    else
    {
        rt_memset(&ret_value[0],0xff,MB_MASTER_TOTAL_SLAVE_NUM*2);
    }

    i = 0;
    do
    {
        tx_buffer[tx_length] = ret_value[i]>>8;
        tx_buffer[tx_length+1] = ret_value[i]&0xff;
        tx_length += 2;
    }
    while(++i < MB_MASTER_TOTAL_SLAVE_NUM);

    tx_buffer[2] = tx_length - 3;

    crc_code = crc_16(tx_buffer, tx_length);
    tx_buffer[tx_length] = (uint8_t)(crc_code & 0x00FF);
    tx_buffer[tx_length+1] = (uint8_t)(crc_code >> 8);
    tx_length += 2;

    serial_send(SERIAL_1,&tx_buffer[0],tx_length);
}


void read_sensor_single_flow_handler(uint16_t cmd_type,uint16_t output_channel,uint16_t output_type)
{
    uint16_t crc_code;
    tx_length = 0;
    tx_buffer[tx_length] = BOARD_ID;
    tx_buffer[tx_length+1] = 0x03;
    tx_buffer[tx_length+2] = 0;    // 长度待确定
    tx_length += 3;

    tx_buffer[tx_length] = cmd_type>>8;
    tx_buffer[tx_length+1] = cmd_type&0xff;
    tx_length += 2;

    uint16_t an_range_h;

    if(output_channel == 1 )
    {
        an_range_h = sysEnv.getEnvVar(CH1_AN_RANGE_H);
    }
    else if(output_channel == 2)
    {
        an_range_h = sysEnv.getEnvVar(CH2_AN_RANGE_H);
    }


    if(output_type == VOLT_TYPE && an_range_h < 5000)
    {
        read_sensor_flow_an(output_channel,&ret_value[0],MB_MASTER_TOTAL_SLAVE_NUM);
    }
    else if(output_type == VOLT_TYPE && an_range_h >= 5000)
    {
        read_sensor_flow_module(output_channel,output_type,&ret_value[0],MB_MASTER_TOTAL_SLAVE_NUM);
    }
    else if(output_type == CURRENT_TYPE)
    {
        read_sensor_flow_module(output_channel,output_type,&ret_value[0],MB_MASTER_TOTAL_SLAVE_NUM);
    }
    else if(output_type == PWM_TYPE)
    {
        read_sensor_flow_freq(output_channel,&ret_value[0],MB_MASTER_TOTAL_SLAVE_NUM);
    }
    else if(output_type == UART_TYPE  || output_type == RS485_TYPE)
    {
        read_sensor_flow_rs485(&ret_value[0],MB_MASTER_TOTAL_SLAVE_NUM);
    }
    else
    {
        rt_memset(&ret_value[0],0xff,MB_MASTER_TOTAL_SLAVE_NUM*2);
    }

    uint16_t i = 0;
    do
    {
        tx_buffer[tx_length] = ret_value[i]>>8;
        tx_buffer[tx_length+1] = ret_value[i]&0xff;
        tx_length += 2;
    }
    while(++i < MB_MASTER_TOTAL_SLAVE_NUM);

    tx_buffer[2] = tx_length - 3;

    crc_code = crc_16(tx_buffer, tx_length);
    tx_buffer[tx_length] = (uint8_t)(crc_code & 0x00FF);
    tx_buffer[tx_length+1] = (uint8_t)(crc_code >> 8);
    tx_length += 2;

    serial_send(SERIAL_1,&tx_buffer[0],tx_length);
}


void write_handler(uint16_t reg_addr,uint16_t value)
{
    rt_err_t result;
    uint16_t data[MB_MASTER_TOTAL_SLAVE_NUM];

    uint16_t id = 1;
    do
    {
        data[id-1] = value;
    } while(++id <= MB_MASTER_TOTAL_SLAVE_NUM);

    result = mb_write_holding_register_2(SENSOR_ID,reg_addr,data, MB_TIMEOUT);

    id = 1;
    do
    {
        if(result & (1<(id-1)))
        {
            log_develop(CAL_INFO,"write error for sensor id %d\n",id);
        }
    } while(++id <= MB_MASTER_TOTAL_SLAVE_NUM);

}


void write_handler_2(uint16_t reg_addr,const uint16_t buf[])
{
    rt_err_t result;
    uint16_t data[MB_MASTER_TOTAL_SLAVE_NUM];

    uint16_t id = 1;
    do
    {
        data[id-1] = buf[id-1];
    } while(++id <= MB_MASTER_TOTAL_SLAVE_NUM);

    result = mb_write_holding_register_2(SENSOR_ID,reg_addr,data, MB_TIMEOUT);

    id = 1;
    do
    {
        if(result & (1<(id-1)))
        {
            log_develop(CAL_INFO,"write error for sensor id %d\n",id);
        }
    } while(++id <= MB_MASTER_TOTAL_SLAVE_NUM);
}


void write_handler_3(uint16_t reg_addr,uint16_t buf[][8])
{
    rt_err_t result;
    uint16_t id = 1;
    do
    {
        result = mb_write_8_holding_register(slave[id].serial,SENSOR_ID, reg_addr, buf[id-1], 2*MB_TIMEOUT);
        if(result != RT_EOK)
        {
            log_develop(CAL_INFO,"write error for sensor id %d\n",id);
        }
        else
        {
            __nop();
        }
    } while(++id <= MB_MASTER_TOTAL_SLAVE_NUM);

}


void sensor_analog_cal_handler(uint16_t cmd_type)
{
    rt_err_t result;
    uint16_t crc_code;
    uint16_t an_type = sysEnv.getEnvVar(CH1_OUTPUT_TYPE);
    // uint16_t an_type = VOLT;
    tx_length = 0;
    tx_buffer[tx_length] = BOARD_ID;
    tx_buffer[tx_length+1] = 0x06;
    tx_buffer[tx_length+2] = 0x02;
    tx_length += 3;
    uint16_t point = (rx_buffer[5]<<8) + rx_buffer[6];

    if(point < 1 || point > AN_CAL_POINT_NUM_MAX)
    {
        log_develop(CAL_INFO,"analog cal point error\n");
        return;
    }
    uint16_t std_flow = (rx_buffer[7]<<8) + rx_buffer[8];
    uint16_t setval = (rx_buffer[9]<<8) + rx_buffer[10];

    uint16_t id = 1;
    do
    {
        // rt_thread_mdelay(150);
        result = sensor_analog_calib(an_type,id,point,std_flow,setval);
        if(result != RT_EOK)
        {
            log_develop(CAL_INFO,"analog cal error for senser %d\n",id);
        }
        else
        {
            log_develop(CAL_INFO,"analog cal success for senser %d\n",id);
        }
    } while(++id <= MB_MASTER_TOTAL_SLAVE_NUM);

    tx_buffer[3] = cmd_type>>8;
    tx_buffer[4] = cmd_type&0xff;
    tx_length += 2;

    crc_code = crc_16(tx_buffer, tx_length);
    tx_buffer[5] = (uint8_t)(crc_code & 0x00FF);
    tx_buffer[6] = (uint8_t)(crc_code >> 8);
    tx_length += 2;

    serial_send(SERIAL_1,&tx_buffer[0],tx_length);
}


void normal_reply(uint16_t cmd_type)
{
    uint16_t crc_code;
    tx_length = 0;

    tx_buffer[tx_length] = BOARD_ID;
    tx_buffer[tx_length+1] = 0x06;
    tx_buffer[tx_length+2] = 0x02;
    tx_length += 3;
    tx_buffer[tx_length] = cmd_type>>8;
    tx_buffer[tx_length+1] = cmd_type&0xff;
    tx_length += 2;
    crc_code = crc_16(tx_buffer, tx_length);
    tx_buffer[tx_length] = (uint8_t)(crc_code & 0x00FF);
    tx_buffer[tx_length+1] = (uint8_t)(crc_code >> 8);
    tx_length += 2;
    serial_send(SERIAL_1,&tx_buffer[0],tx_length);
}


void read_reply(uint16_t cmd_type,int16_t buf[],uint16_t len)
{
    uint16_t crc_code;
    tx_length = 0;

    tx_buffer[tx_length] = BOARD_ID;
    tx_buffer[tx_length+1] = 0x03;
    tx_buffer[tx_length+2] = 0x02;
    tx_length += 3;
    tx_buffer[tx_length] = cmd_type>>8;
    tx_buffer[tx_length+1] = cmd_type&0xff;
    tx_length += 2;

    for(uint16_t i = 0; i<len; i++)
    {
        tx_buffer[tx_length] = buf[i]>>8;
        tx_buffer[tx_length+1] = buf[i]&0xff;
        tx_length += 2;
    }

    tx_buffer[2] = tx_length - 3;

    crc_code = crc_16(tx_buffer, tx_length);
    tx_buffer[tx_length] = (uint8_t)(crc_code & 0x00FF);
    tx_buffer[tx_length+1] = (uint8_t)(crc_code >> 8);
    tx_length += 2;
    serial_send(SERIAL_1,&tx_buffer[0],tx_length);
}

void read_alarm_status(uint16_t cmd_type)
{
    uint16_t crc_code;
    tx_length = 0;
    tx_buffer[tx_length] = BOARD_ID;
    tx_buffer[tx_length+1] = 0x03;
    tx_buffer[tx_length+2] = 0;    // 长度待确定
    tx_length += 3;

    tx_buffer[tx_length] = cmd_type>>8;
    tx_buffer[tx_length+1] = cmd_type&0xff;
    tx_length += 2;

    rt_memset(ret_value,0,sizeof(ret_value));

    if((0 != rt_pin_read(PULSE_CAP_1_PIN_NUM)))
    {
        ret_value[0] = 1;
    }
    else
    {
        ret_value[0] = (0 != rt_pin_read(PULSE_CAP_4_PIN_NUM))?2:0;
    }

    if((0 != rt_pin_read(PULSE_CAP_2_PIN_NUM)))
    {
        ret_value[1] = 1;
    }
    else
    {
        ret_value[1] = (0 != rt_pin_read(PULSE_CAP_5_PIN_NUM))?2:0;
    }

    if((0 != rt_pin_read(PULSE_CAP_3_PIN_NUM)))
    {
        ret_value[2] = 1;
    }
    else
    {
        ret_value[2] = (0 != rt_pin_read(PULSE_CAP_6_PIN_NUM))?2:0;
    }

    uint16_t i = 0;
    do
    {
        tx_buffer[tx_length] = ret_value[i]>>8;
        tx_buffer[tx_length+1] = ret_value[i]&0xff;
        tx_length += 2;
    }
    while(++i < MB_MASTER_TOTAL_SLAVE_NUM);

    tx_buffer[2] = tx_length - 3;
    crc_code = crc_16(tx_buffer, tx_length);
    tx_buffer[tx_length] = (uint8_t)(crc_code & 0x00FF);
    tx_buffer[tx_length+1] = (uint8_t)(crc_code >> 8);
    tx_length += 2;
    serial_send(SERIAL_1,&tx_buffer[0],tx_length);
}


void cal_task_entry(void *param)
{
    uint16_t cmd_type;
    uint16_t crc_code;
    dynamic_mutex = rt_mutex_create("dmutex", RT_IPC_FLAG_PRIO);
    while(1)
    {
        rx_length = serial_recv(SERIAL_1, rx_buffer, sizeof(rx_buffer),RT_WAITING_FOREVER);
        if (rx_length < 5 || rx_buffer[2] != rx_length - 5) {
            log_develop(CAL_INFO,"msg lenght error\n");
            continue;
        }
        if(rx_buffer[0] !=  BOARD_ID) {
            log_develop(CAL_INFO,"msg addr error\n");
            continue;
        }
        crc_code = ((uint16_t)rx_buffer[rx_length - 1] << 8) | rx_buffer[rx_length - 2];

        if (crc_code != crc_16(rx_buffer, rx_length - 2)) {
            log_develop(CAL_INFO,"msg crc error\n");
            continue;
        }

        cmd_type = (rx_buffer[3]<<8) + rx_buffer[4];
        switch(rx_buffer[1])
        {
        case 0x03:
            if(cmd_type == 0x1000)
            {

            }
            else if(cmd_type == 0x1100)
            {
                uint16_t value[MB_MASTER_TOTAL_SLAVE_NUM];
                uint16_t id = 1;
                do
                {
                    value[id-1] = get_analog_value_uncalib(slave[id].channel);
                } while(++id <= MB_MASTER_TOTAL_SLAVE_NUM);
                read_reply(cmd_type,value,3);
            }
            else if(cmd_type == 0x1101)
            {
                uint16_t value[MB_MASTER_TOTAL_SLAVE_NUM];
                uint16_t id = 1;
                do
                {
                    value[id-1] = get_analog_value_calib(slave[id].channel);
                } while(++id <= MB_MASTER_TOTAL_SLAVE_NUM);
                read_reply(cmd_type,value,3);
            }
            else if(cmd_type == 0x1001) 	// 读通道1流量
            {
                uint16_t output1_type = sysEnv.getEnvVar(CH1_OUTPUT_TYPE);
                read_sensor_single_flow_handler(cmd_type,1,output1_type);
            }
            else if(cmd_type == 0x1002) 	// 读通道2流量
            {
                uint16_t output2_type = sysEnv.getEnvVar(CH2_OUTPUT_TYPE);
                read_sensor_single_flow_handler(cmd_type,2,output2_type);
            }
            else if(cmd_type == 0x1010) 	// 读通道1、2流量
            {
                uint16_t output1_type = sysEnv.getEnvVar(CH1_OUTPUT_TYPE);
                uint16_t output2_type = sysEnv.getEnvVar(CH2_OUTPUT_TYPE);
                read_sensor_flow_handler(cmd_type,output1_type,output2_type);
            }
            else if(cmd_type == 0x1003) 	// 读频率
            {
                read_handler(cmd_type,REG_FREQ_ADDR);
            }
            else if(cmd_type == 0x1004)   //读温度
            {
                read_handler(cmd_type,REG_TEMP_ADDR);
            }
            else if(cmd_type == 0x1005)   //读标定点流量
            {
                command_param[0] = (rx_buffer[5]<<8) + rx_buffer[6];
                if(command_param[0] < 1 || command_param[0] > CAL_POINT_NUM_MAX - 1)    ///
                {
                    int16_t data[3] = {-1,-1,-1};
                    read_reply(cmd_type,data,3);
                    continue;
                }
                uint16_t reg_addr;
                if(command_param[0] == 1)
                {
                    reg_addr = FLOW_START_ADDR + command_param[0] - 1;
                }
                else if(command_param[0] == 11)
                {
                    reg_addr = FLOW_START_ADDR + 1;
                }
                else
                {
                    reg_addr = FLOW_START_ADDR + command_param[0];  // 跳过第二个标定点，把第二个标定点另做它用。
                }
                read_handler(cmd_type,reg_addr);
            }

            else if(cmd_type == 0x1006)   											// 读标定点频率
            {
                command_param[0] = (rx_buffer[5]<<8) + rx_buffer[6];
                if(command_param[0] < 1 || command_param[0] > CAL_POINT_NUM_MAX -1)  ///
                {
                    int16_t data[3] = {-1,-1,-1};
                    read_reply(cmd_type,data,3);
                    continue;
                }
                uint16_t reg_addr;
                if(command_param[0] == 1)
                {
                    reg_addr = FREQ_START_ADDR + command_param[0] - 1;
                }
                else if(command_param[0] == 11)
                {
                    reg_addr = FREQ_START_ADDR + 1;
                }
                else
                {
                    reg_addr = FREQ_START_ADDR + command_param[0];  // 跳过第二个标定点，把第二个标定点另做它用。
                }
                read_handler(cmd_type,reg_addr );
            }
            else if(cmd_type == 0x1007)   	// 读模拟量校准点流量
            {
                command_param[0] = (rx_buffer[5]<<8) + rx_buffer[6];

                if(command_param[0] < 1 || command_param[0]>AN_CAL_POINT_NUM_MAX)
                {
                    continue;
                }

                read_handler(cmd_type,AN_FLOW_START_ADDR + command_param[0] - 1 );
            }
            else if(cmd_type == 0x1008)   	// 读模拟量校准点占空比
            {
                command_param[0] = (rx_buffer[5]<<8) + rx_buffer[6];
                if(command_param[0] < 1 || command_param[0] > AN_CAL_POINT_NUM_MAX)
                {
                    continue;
                }
                read_handler(cmd_type,AN_ZKB_START_ADDR + command_param[0] - 1 );
            }

            else if(cmd_type == 0x1009)   	// 读传感器通道1输出类型
            {
                int16_t output_type = sysEnv.getEnvVar(CH1_OUTPUT_TYPE);
                read_reply(cmd_type,&output_type,1);
            }
            else if(cmd_type == 0x100E)   	// 读传感器通道2输出类型
            {
                int16_t output_type = sysEnv.getEnvVar(CH2_OUTPUT_TYPE);
                read_reply(cmd_type,&output_type,1);
            }
            else if(cmd_type == 0x1011)   	// 读传感器通道1、2输出类型
            {
                int16_t output_type[2];
                output_type[0] = sysEnv.getEnvVar(CH1_OUTPUT_TYPE);
                output_type[1] = sysEnv.getEnvVar(CH2_OUTPUT_TYPE);
                read_reply(cmd_type,output_type,2);
            }
            else if(cmd_type == 0x100A)   	// 读传感器量程
            {
                int16_t meas_range[2];
                meas_range[0] = sysEnv.getEnvVar(MEAS_RANGE_L);
                meas_range[1] = sysEnv.getEnvVar(MEAS_RANGE_H);
                read_reply(cmd_type,meas_range,2);
            }
            else if(cmd_type == 0x100B)   	// 读通道1模拟量上下限
            {
                int16_t an_range[2];
                an_range[0] = sysEnv.getEnvVar(CH1_AN_RANGE_L);
                an_range[1] = sysEnv.getEnvVar(CH1_AN_RANGE_H);
                read_reply(cmd_type,an_range,2);
            }
            else if(cmd_type == 0x100F)   	// 读通道2模拟量上下限
            {
                int16_t an_range[2];
                an_range[0] = sysEnv.getEnvVar(CH2_AN_RANGE_L);
                an_range[1] = sysEnv.getEnvVar(CH2_AN_RANGE_H);
                read_reply(cmd_type,an_range,2);
            }
            else if(cmd_type == 0x100C)   	// 读报警状态
            {
                uint16_t out_type = sysEnv.getEnvVar(CH1_OUTPUT_TYPE);
                if( out_type != NPN_TYPE && out_type != PNP_TYPE)
                {
                    out_type = sysEnv.getEnvVar(CH2_OUTPUT_TYPE);
                    if(out_type != NPN_TYPE && out_type != PNP_TYPE)
                    {
                        int16_t data[3] = {0,0,0};
                        read_reply(cmd_type,data,3);
                        continue;
                    }
                }
                read_alarm_status(cmd_type);
            }
            else if(cmd_type == 0x100D)    // 读序列号
            {
                uint16_t serial_num[MB_MASTER_TOTAL_SLAVE_NUM][SERIAL_NUM];
                uint32_t result;
                result = mb_parallel_read_holding_register(SENSOR_ID,SERIAL_NUM_START_ADDR,SERIAL_NUM,2*MB_TIMEOUT);
                uint16_t id = 1;
                do
                {
                    if(result & (1<<(id-1)))
                    {
                        rt_memset(serial_num[id-1],0xff,SERIAL_NUM*sizeof(uint16_t));
                        log_develop(CAL_INFO,"read error for cmd %04x,and sensor id is %d\n",cmd_type,id);
                    }
                    else
                    {
                        rt_memcpy(serial_num[id-1],&user_reg_hold_buf_2[id-1][SERIAL_NUM_START_ADDR],SERIAL_NUM*sizeof(uint16_t));
                    }
                } while(++id <= MB_MASTER_TOTAL_SLAVE_NUM);

                int16_t tmp[SERIAL_NUM * MB_MASTER_TOTAL_SLAVE_NUM];
                rt_memcpy(tmp,serial_num[0],SERIAL_NUM*sizeof(uint16_t));
                rt_memcpy(tmp + SERIAL_NUM,serial_num[1],SERIAL_NUM*sizeof(uint16_t));
                rt_memcpy(tmp + 2*SERIAL_NUM,serial_num[2],SERIAL_NUM*sizeof(uint16_t));
                read_reply(cmd_type,tmp,SERIAL_NUM * MB_MASTER_TOTAL_SLAVE_NUM);
            }
            break;

        case 0x06:

            if(cmd_type==0x2000) //标定使能
            {
                write_handler(REG_CAL_ENABLE_ADDR,1234);
                normal_reply(cmd_type);

            }
            else if(cmd_type==0x2001)  						// 写标定点流量
            {
                command_param[0] = (rx_buffer[5]<<8) + rx_buffer[6];
                command_param[1] = (rx_buffer[7]<<8) + rx_buffer[8];

                if(command_param[0] < 2 || command_param[0] > CAL_POINT_NUM_MAX-1)
                {
                    continue;
                }
                uint16_t reg_addr;
                if(command_param[0] == 1)
                {
                    reg_addr = FLOW_START_ADDR + command_param[0] - 1;
                }
                else if(command_param[0] == 11)
                {
                    reg_addr = FLOW_START_ADDR + 1;
                }
                else
                {
                    reg_addr = FLOW_START_ADDR + command_param[0];  // 跳过第二个标定点，把第二个标定点另做它用。
                }
                //  write_handler(reg_addr,command_param[1]);
                normal_reply(cmd_type);

            }
            else if(cmd_type==0x2002) 							// 写标定点频率
            {
                command_param[0] = (rx_buffer[5]<<8) + rx_buffer[6];  // 第几个标定点
                if(command_param[0] < 2 || command_param[0] > CAL_POINT_NUM_MAX-1)
                {
                    continue;
                }
                uint16_t id = 1;
                do
                {
                    command_param[id] = (rx_buffer[5+2*id]<<8) + rx_buffer[6+2*id];
                } while(++id <= MB_MASTER_TOTAL_SLAVE_NUM);

                uint16_t reg_addr;
                if(command_param[0] == 1)
                {
                    reg_addr = FREQ_START_ADDR + command_param[0] - 1;
                }
                else if(command_param[0] == 11)
                {
                    reg_addr = FREQ_START_ADDR + 1;
                }
                else
                {
                    reg_addr = FREQ_START_ADDR + command_param[0];  // 跳过第二个标定点，把第二个标定点另做它用。
                }
                write_handler_2(reg_addr,&command_param[1]);
                normal_reply(cmd_type);
            }
            else if(cmd_type==0x2003)  	// 使能模拟量校准
            {
                write_handler(REG_CAL_ENABLE_ADDR,1235);
                normal_reply(cmd_type);
            }
            else if(cmd_type==0x2004)		// 模拟量校准
            {
                rt_mutex_take(dynamic_mutex, RT_WAITING_FOREVER);
                sensor_analog_cal_handler(cmd_type);
                rt_mutex_release(dynamic_mutex);
            }
            else if(cmd_type==0x2005) 	// 保存标定
            {
                uint16_t cal_point_flow[MB_MASTER_TOTAL_SLAVE_NUM][CAL_POINT_NUM_MAX];
                uint16_t cal_point_freq[MB_MASTER_TOTAL_SLAVE_NUM][CAL_POINT_NUM_MAX];

                uint32_t result;
                result = mb_parallel_read_holding_register(SENSOR_ID,FLOW_START_ADDR,CAL_POINT_NUM_MAX, 2*MB_TIMEOUT);
                uint16_t id = 1;
                do
                {
                    if(result & (1<<(id-1)))
                    {
                        rt_memset(cal_point_flow[id-1],0xff,CAL_POINT_NUM_MAX*sizeof(uint16_t));
                        log_develop(CAL_INFO,"read error for cmd %04x,and sensor id is %d\n",cmd_type,id);
                    }
                    else
                    {
                        rt_memcpy(cal_point_flow[id-1],&user_reg_hold_buf_2[id-1][FLOW_START_ADDR],CAL_POINT_NUM_MAX*sizeof(uint16_t));
                        cal_point_flow[id-1][1] = cal_point_flow[id-1][2] * 4 / 5;

                        cal_point_flow[id-1][11] = cal_point_flow[id-1][10] + cal_point_flow[id-1][3] - cal_point_flow[id-1][2] ;

                        if(cal_point_flow[id-1][11] < cal_point_flow[id-1][10])
                            cal_point_flow[id-1][11] = cal_point_flow[id-1][10] + 10;
                    }

                } while(++id <= MB_MASTER_TOTAL_SLAVE_NUM);

                rt_thread_mdelay(50);
                result = mb_parallel_read_holding_register(SENSOR_ID,FREQ_START_ADDR,CAL_POINT_NUM_MAX, 2*MB_TIMEOUT);

                id = 1;
                do
                {

                    if(result & (1<<(id-1)))
                    {
                        rt_memset(cal_point_freq[id-1],0xff,CAL_POINT_NUM_MAX*sizeof(uint16_t));
                        log_develop(CAL_INFO,"read error for cmd %04x,and sensor id is %d\n",cmd_type,id);
                    }
                    else
                    {
                        rt_memcpy(cal_point_freq[id-1],&user_reg_hold_buf_2[id-1][FREQ_START_ADDR],CAL_POINT_NUM_MAX*sizeof(uint16_t));
                        cal_point_freq[id-1][1] = cal_point_freq[id-1][2]*3/4;
                        cal_point_freq[id-1][11] = cal_point_freq[id-1][10] + 3*(cal_point_freq[id-1][3] - cal_point_freq[id-1][2]) ;
                        if(cal_point_freq[id-1][11] < cal_point_freq[id-1][10])
                            cal_point_freq[id-1][11] = cal_point_freq[id-1][10] + 100;
                    }

                } while(++id <= MB_MASTER_TOTAL_SLAVE_NUM);

                uint16_t tmp[MB_MASTER_TOTAL_SLAVE_NUM];
                rt_thread_mdelay(50);
                for(uint16_t i = 0; i<MB_MASTER_TOTAL_SLAVE_NUM; i++)
                {
                    tmp[i] =  cal_point_flow[i][1];
                }
                //  write_handler_2(FLOW_START_ADDR + 1,tmp);
                rt_thread_mdelay(50);
                for(uint16_t i = 0; i<MB_MASTER_TOTAL_SLAVE_NUM; i++)
                {
                    tmp[i] =  cal_point_flow[i][11];
                }
                write_handler_2(FLOW_START_ADDR + 11,tmp);
                rt_thread_mdelay(50);
                for(uint16_t i = 0; i<MB_MASTER_TOTAL_SLAVE_NUM; i++)
                {
                    tmp[i] =  cal_point_freq[i][1];
                }
                //  write_handler_2(FREQ_START_ADDR + 1,tmp);
                rt_thread_mdelay(50);
                for(uint16_t i = 0; i<MB_MASTER_TOTAL_SLAVE_NUM; i++)
                {
                    tmp[i] =  cal_point_freq[i][11];
                }
                write_handler_2(FREQ_START_ADDR + 11,tmp);
                rt_thread_mdelay(50);

                write_handler(REG_CAL_ENABLE_ADDR,0);
                normal_reply(cmd_type);
            }
            else if(cmd_type==0x2006)	// 设置通道1输出类型
            {
                command_param[0] = (rx_buffer[5]<<8) + rx_buffer[6];  // 参数1，输出类型
                sysEnv.setEnvVar(CH1_OUTPUT_TYPE,command_param[0]);
                sysEnv.save();
                normal_reply(cmd_type);
            }
            else if(cmd_type==0x200B)	// 设置通道2输出类型
            {
                command_param[0] = (rx_buffer[5]<<8) + rx_buffer[6];  // 参数1，输出类型
                sysEnv.setEnvVar(CH2_OUTPUT_TYPE,command_param[0]);
                sysEnv.save();
                normal_reply(cmd_type);
            }
            else if(cmd_type==0x2007)	// 设置量程
            {
                command_param[0] = (rx_buffer[5]<<8) + rx_buffer[6];  // 参数1，量程下限
                command_param[1] = (rx_buffer[7]<<8) + rx_buffer[8];  // 参数2，量程上限
                sysEnv.setEnvVar(MEAS_RANGE_L,command_param[0]);
                sysEnv.setEnvVar(MEAS_RANGE_H,command_param[1]);
                sysEnv.save();
                normal_reply(cmd_type);
            }
            else if(cmd_type==0x2008) 	// 设置通道1模拟量输出范围
            {
                command_param[0] = (rx_buffer[5]<<8) + rx_buffer[6];  // 参数1，模拟量输出下限
                command_param[1] = (rx_buffer[7]<<8) + rx_buffer[8];  // 参数2，模拟量输出上限
                sysEnv.setEnvVar(CH1_AN_RANGE_L,command_param[0]);
                sysEnv.setEnvVar(CH1_AN_RANGE_H,command_param[1]);
                sysEnv.save();
                normal_reply(cmd_type);
                HAL_NVIC_SystemReset();
            }
            else if(cmd_type==0x200C) 	// 设置通道2模拟量输出范围
            {
                command_param[0] = (rx_buffer[5]<<8) + rx_buffer[6];  // 参数1，模拟量输出下限
                command_param[1] = (rx_buffer[7]<<8) + rx_buffer[8];  // 参数2，模拟量输出上限
                sysEnv.setEnvVar(CH2_AN_RANGE_L,command_param[0]);
                sysEnv.setEnvVar(CH2_AN_RANGE_H,command_param[1]);
                sysEnv.save();
                normal_reply(cmd_type);
                HAL_NVIC_SystemReset();
            }
            else if(cmd_type==0x2009)   // 保存模拟量校准值
            {
                write_handler(REG_CAL_ENABLE_ADDR,0);
                normal_reply(cmd_type);
            }
            else if(cmd_type==0x200A)  // 写序列号
            {
                uint16_t serial_num[3][8];
                serial_num[0][0] = (rx_buffer[5]<<8) + rx_buffer[6];
                serial_num[0][1] = (rx_buffer[7]<<8) + rx_buffer[8];
                serial_num[0][2] = (rx_buffer[9]<<8) + rx_buffer[10];
                serial_num[0][3] = (rx_buffer[11]<<8) + rx_buffer[12];
                serial_num[0][4] = (rx_buffer[13]<<8) + rx_buffer[14];
                serial_num[0][5] = (rx_buffer[15]<<8) + rx_buffer[16];
                serial_num[0][6] = (rx_buffer[17]<<8) + rx_buffer[18];
                serial_num[0][7] = (rx_buffer[19]<<8) + rx_buffer[20];

                serial_num[1][0] = (rx_buffer[21]<<8) + rx_buffer[22];
                serial_num[1][1] = (rx_buffer[23]<<8) + rx_buffer[24];
                serial_num[1][2] = (rx_buffer[25]<<8) + rx_buffer[26];
                serial_num[1][3] = (rx_buffer[27]<<8) + rx_buffer[28];
                serial_num[1][4] = (rx_buffer[29]<<8) + rx_buffer[30];
                serial_num[1][5] = (rx_buffer[31]<<8) + rx_buffer[32];
                serial_num[1][6] = (rx_buffer[33]<<8) + rx_buffer[34];
                serial_num[1][7] = (rx_buffer[35]<<8) + rx_buffer[36];

                serial_num[2][0] = (rx_buffer[37]<<8) + rx_buffer[38];
                serial_num[2][1] = (rx_buffer[39]<<8) + rx_buffer[40];
                serial_num[2][2] = (rx_buffer[41]<<8) + rx_buffer[42];
                serial_num[2][3] = (rx_buffer[43]<<8) + rx_buffer[44];
                serial_num[2][4] = (rx_buffer[45]<<8) + rx_buffer[46];
                serial_num[2][5] = (rx_buffer[47]<<8) + rx_buffer[48];
                serial_num[2][6] = (rx_buffer[49]<<8) + rx_buffer[50];
                serial_num[2][7] = (rx_buffer[51]<<8) + rx_buffer[52];
                write_handler_3(SERIAL_NUM_START_ADDR,serial_num);
                normal_reply(cmd_type);
            }

            else if(cmd_type==0x2010)  // 写标定点流量 分别写
            {
                command_param[0] = (rx_buffer[5]<<8) + rx_buffer[6];  // 第几个标定点
                if(command_param[0] < 1 || command_param[0] > CAL_POINT_NUM_MAX-1)
                    continue;
                uint16_t id = 1;
                do
                {
                    command_param[id] = (rx_buffer[5+2*id]<<8) + rx_buffer[6+2*id];
                } while(++id <= MB_MASTER_TOTAL_SLAVE_NUM);

                uint16_t reg_addr;
                if(command_param[0] == 1)
                {
                    reg_addr = FLOW_START_ADDR + command_param[0] - 1;
                }
                else if(command_param[0] == 11)
                {
                    reg_addr = FLOW_START_ADDR + 1;
                }
                else
                {
                    reg_addr = FLOW_START_ADDR + command_param[0];  // 跳过第二个标定点，把第二个标定点另做它用。
                }
                write_handler_2(reg_addr,&command_param[1]);
                normal_reply(cmd_type);
            }
            else if(cmd_type==0x2011)  // 写模拟量校准点流量
            {
                command_param[0] = (rx_buffer[5]<<8) + rx_buffer[6];  // 第几个标定点
                if(command_param[0] < 1 || command_param[0] > AN_CAL_POINT_NUM_MAX)
                {
                    continue;
                }
                uint16_t id = 1;
                do
                {
                    command_param[id] = (rx_buffer[5+2*id]<<8) + rx_buffer[6+2*id];
                } while(++id <= MB_MASTER_TOTAL_SLAVE_NUM);

                uint16_t reg_addr =  AN_FLOW_START_ADDR + command_param[0] - 1;
                write_handler_2(reg_addr,&command_param[1]);
                normal_reply(cmd_type);
            }
            else if(cmd_type==0x2012)  // 写校准点PWM占空比
            {
                command_param[0] = (rx_buffer[5]<<8) + rx_buffer[6];  // 第几个标定点
                if(command_param[0] < 1 || command_param[0] > AN_CAL_POINT_NUM_MAX)
                {
                    continue;
                }
                uint16_t id = 1;
                do
                {
                    command_param[id] = (rx_buffer[5+2*id]<<8) + rx_buffer[6+2*id];
                } while(++id <= MB_MASTER_TOTAL_SLAVE_NUM);

                uint16_t reg_addr =  AN_ZKB_START_ADDR + command_param[0] - 1;

                write_handler_2(reg_addr,&command_param[1]);
                normal_reply(cmd_type);
            }
            else if(cmd_type==0x2100) 								
            {
                command_param[0] = (rx_buffer[5]<<8) + rx_buffer[6];  
                if(command_param[0] > 1)
                {
                    continue;
                }
                uint16_t id = 1;
                do
                {
                    command_param[id] = (rx_buffer[5+2*id]<<8) + rx_buffer[6+2*id];
                } while(++id <=  3);   	// 校准点数3个

                if(command_param[0] == 0)    //电压
                {
                    sysEnv.setEnvVar(VOLT_CAL_POINT_X1,command_param[1]);
                    sysEnv.setEnvVar(VOLT_CAL_POINT_X2,command_param[2]);
                    sysEnv.setEnvVar(VOLT_CAL_POINT_X3,command_param[3]);
                }
                else if(command_param[0] == 1)   //电流
                {
                    sysEnv.setEnvVar(CURRENT_CAL_POINT_X1,command_param[1]);
                    sysEnv.setEnvVar(CURRENT_CAL_POINT_X2,command_param[2]);
                    sysEnv.setEnvVar(CURRENT_CAL_POINT_X3,command_param[3]);
                }
                sysEnv.save();
                normal_reply(cmd_type);
                HAL_NVIC_SystemReset();
            }
            break;

        default:

            break;

        }
    }
}


static int cal_task_init(void)
{
    rt_err_t ret = RT_EOK;
    static rt_thread_t cal_thread = RT_NULL;

    cal_thread = rt_thread_create("cal_thread",cal_task_entry, RT_NULL,2048,5, 20);

    if (cal_thread != RT_NULL)
        rt_thread_startup(cal_thread);

    return ret;
}
INIT_APP_EXPORT(cal_task_init);








