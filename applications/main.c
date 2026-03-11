/*
 * Copyright (c) 2006-2018, RT-Thread Development Team
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Change Logs:
 * Date           Author       Notes
 * 2019-03-05     whj4674672   first version
 */

#include <rtthread.h>
#include <rtdevice.h>
#include <board.h>
#include <drv_spi.h>
#include "my_dac.h"
#include "filter.h"

//-----------------------------------------------------------------
// 宏定义
//-----------------------------------------------------------------
// 命令寄存器
#define NO_OP				0x0000	// 继续操作
#define STDBY				0x8200	// 进入待机状态
#define PWR_DN				0x8300	// 设备断电
#define RST					0x8500	// 复位
#define AUTO_RST			0xA000	// 重启后启动自动模式
#define MAN_CH_0			0xC000	// 选择通道0输入
#define MAN_CH_1			0xC400	// 选择通道1输入
#define MAN_CH_2			0xC800	// 选择通道2输入
#define MAN_CH_3			0xCC00	// 选择通道3输入
#define MAN_CH_4			0xD000	// 选择通道4输入
#define MAN_CH_5			0xD400	// 选择通道5输入
#define MAN_CH_6			0xD800	// 选择通道6输入
#define MAN_CH_7			0xDC00	// 选择通道7输入
#define MAN_AUX				0xE000	// 选择通道AUX输入

// 程序寄存器
#define AUTO_SEQ_EN						0x01	// 自动扫描排序控制寄存器
#define CH_PWR_DN			 			0x02	// 通道掉电寄存器
#define FEATURE_SELECT 					0x03	// 器件特性选择控制寄存器

#define CH0_INPUT_RANGE 				0x05	// 通道0输入范围选择寄存器
#define CH1_INPUT_RANGE 				0x06	// 通道1输入范围选择寄存器
#define CH2_INPUT_RANGE 				0x07	// 通道2输入范围选择寄存器
#define CH3_INPUT_RANGE 				0x08	// 通道3输入范围选择寄存器
#define CH4_INPUT_RANGE 				0x09	// 通道4输入范围选择寄存器
#define CH5_INPUT_RANGE 				0x0A	// 通道5输入范围选择寄存器
#define CH6_INPUT_RANGE 				0x0B	// 通道6输入范围选择寄存器
#define CH7_INPUT_RANGE 				0x0C	// 通道7输入范围选择寄存器
#define CH0_HYSTERESIS 					0x15	//
#define CH0_HIGH_THRESHOLD_MSB 	0x16	//
#define CH0_HIGH_THRESHOLD_LSB 	0x17	//
#define CH0_LOW_THRESHOLD_MSB 	0x18	//
#define CH0_LOW_THRESHOLD_LSB 	0x19	//
#define CH7_HYSTERESIS 					0x38	//
#define CH7_HIGH_THRESHOLD_MSB 	0x39	//
#define CH7_HIGH_THRESHOLD_LSB 	0x3A	//
#define CH7_LOW_THRESHOLD_MSB 	0x3B	//
#define CH7_LOW_THRESHOLD_LSB 	0x3C	//
#define COMMAND_READ_BACK 			0x3F	// 命令回读寄存器，只读

// 输入范围（VREF = 4.096V）
#define VREF_B_25							0x00	// 通道输入范围±2.5*VREF
#define VREF_B_125							0x01	// 通道输入范围±1.25*VREF
#define VREF_B_0625							0x02	// 通道输入范围±0.625*VREF
#define VREF_U_25							0x05	// 通道输入范围2.5*VREF
#define VREF_U_125							0x06	// 通道输入范围1.25*VREF



/* defined the LED0 pin: PB1 */
#define LED0_PIN    GET_PIN(A, 0)

#define ADS8688_DAISY_PIN 		GET_PIN(E, 3) 		/* 菊花链模式 */
#define ADS8688_CS_PIN 		GET_PIN(E, 4) 			/* CS片选引脚 */
#define ADS8688_RST_PIN		GET_PIN(C, 13)

static struct rt_spi_device spi_dev_ads8688; 		/* SPI设备ads8688对象 */
static struct stm32_hw_spi_cs  spi_cs;  			/* SPI设备CS片选引脚 */

struct median_filter adc_median_filter; 


void ads8688_write_command(uint16_t comm)
{
	rt_size_t len;
	uint8_t send_buf[2];
	
	send_buf[0] = (uint8_t)(comm >> 8);
	send_buf[1] = (uint8_t)(comm &0x00FF);	
	len = rt_spi_send(&spi_dev_ads8688,send_buf,2);
	__NOP();

}

void ads8688_write_program(uint8_t addr,uint8_t data)
{
	rt_size_t len;
	uint8_t send_buf[2] = {0,0};
	
	send_buf[0] = (addr << 1) | 0x01;
	send_buf[1] = data;
	
	len = rt_spi_send(&spi_dev_ads8688,send_buf,2);
	__NOP();
}

//void ADS8688_Write_Program(uint8_t addr, uint8_t data)
//{
//	uint8_t wr_data[2] = {0x00, 0x00};
//	
//	wr_data[0] = (addr << 1) | 0x01;
//	wr_data[1] = data;
//	
//	CS_L;
//	HAL_SPI_Transmit(&SPI_Handler, wr_data, 2, 0xFFFF);
//	CS_H;
//}

rt_err_t ads8688_get_man_ch_data(uint16_t ch, uint16_t *data)
{
	rt_err_t err;
	uint8_t recv_buf[2]={0,0};
	uint8_t send_buf[2]={0,0};
	
	ads8688_write_command(ch);
	rt_thread_mdelay(1);
	
	err = rt_spi_send_then_recv(&spi_dev_ads8688,send_buf,2,recv_buf,2);
	
	if(err == RT_EOK)
	{
		*data = ((uint16_t)recv_buf[0] << 8) | recv_buf[1];
	}
	
	return err;
}



static int rt_hw_ads8688_config(void)
{
    rt_err_t res;

    spi_cs.GPIO_Pin = GPIO_PIN_4;
	spi_cs.GPIOx = GPIOE;
	
    rt_pin_mode(ADS8688_CS_PIN, PIN_MODE_OUTPUT);    		/* 设置片选管脚模式为输出 */
	rt_pin_mode(ADS8688_RST_PIN, PIN_MODE_OUTPUT);  
	rt_pin_mode(ADS8688_DAISY_PIN, PIN_MODE_OUTPUT);   
	
    res = rt_spi_bus_attach_device(&spi_dev_ads8688, "spi_dev_ads8688" , "spi4", (void*)&spi_cs);
    if (res != RT_EOK)
    {
        return res;
    }
	
	struct rt_spi_configuration cfg;
    cfg.data_width = 8;
    cfg.mode = RT_SPI_MASTER | RT_SPI_MODE_0 | RT_SPI_MSB;
    cfg.max_hz = 2 * 1000 * 1000;

    rt_spi_configure(&spi_dev_ads8688, &cfg);
	
	rt_pin_write(ADS8688_RST_PIN, PIN_LOW);
	rt_thread_mdelay(1);
	rt_pin_write(ADS8688_RST_PIN, PIN_HIGH);
	rt_pin_write(ADS8688_DAISY_PIN, PIN_LOW);
	
	ads8688_write_command(RST);
	
	rt_thread_mdelay(1);
	
	ads8688_write_program(CH0_INPUT_RANGE, VREF_B_25);
//	ads8688_write_program(CH1_INPUT_RANGE, VREF_B_25);	
//	ads8688_write_program(CH2_INPUT_RANGE, VREF_B_25);	
//	ads8688_write_program(CH3_INPUT_RANGE, VREF_B_25);	
//	ads8688_write_program(CH4_INPUT_RANGE, VREF_B_25);	
//	ads8688_write_program(CH5_INPUT_RANGE, VREF_B_25);	
//	ads8688_write_program(CH6_INPUT_RANGE, VREF_B_25);	
//	ads8688_write_program(CH7_INPUT_RANGE, VREF_B_25);
	
	ads8688_write_program(CH_PWR_DN, 0x00);				// 8个通道退出低功耗
	//ads8688_write_program(AUTO_SEQ_EN, 0xFF);			// 8个通道自动扫描排序
	ads8688_write_command(MAN_CH_0);
	
}



float convert_current_to_flow(float current)
{
	float flow = 0;
	
	flow = (current - 4.0f)*(3000.0f - 0.0f)/(20.0f - 4.0f);
	
	return flow;
}


void set_dac_output_voltage(float vlot_set_value_mV)
{
	uint16_t dac_value = vlot_set_value_mV * 4096 / (3300 *3.1428f);
	
	HAL_DAC_SetValue(&DAC1_Handler,DAC_CHANNEL_1,DAC_ALIGN_12B_R,dac_value);		// 设置DAC值
	HAL_DAC_SetValue(&DAC1_Handler,DAC_CHANNEL_2,DAC_ALIGN_12B_R,dac_value);		// 设置DAC值
}

static int isspace(int x)
{
    if (x == ' ' || x == '\t' || x == '\n' || x == '\f' || x == '\b' || x == '\r')
        return 1;
    else
        return 0;
}

static int isdigit(int x)
{
    if (x <= '9' && x >= '0')
        return 1;
    else
        return 0;

}

static int atoi(const char *nptr)
{
    int c;              /* current char */
    int total;         /* current total */
    int sign;           /* if '-', then negative, otherwise positive */

    /* skip whitespace */
    while (isspace((int)(unsigned char)*nptr))
        ++nptr;

    c = (int)(unsigned char) * nptr++;
    sign = c;           /* save sign indication */
    if (c == '-' || c == '+')
        c = (int)(unsigned char) * nptr++;  /* skip sign */

    total = 0;

    while (isdigit(c)) {
        total = 10 * total + (c - '0');     /* accumulate digit */
        c = (int)(unsigned char) * nptr++;  /* get next char */
    }

    if (sign == '-')
        return -total;
    else
        return total;   /* return result, negated if necessary */
}


float g_set_flow_val = 0;
float g_k_factor = 1.0f;

void setvar(int argc, char **argv)
{
    if (argc < 3) {
        rt_kprintf("Please input'setenv <name> <value>'\n");
        return;
    }
    int32_t value = atoi(argv[2]);
    if (!rt_strcmp(argv[1], "flow")) 
	{
        g_set_flow_val =  value;
    }
	else if (!rt_strcmp(argv[1], "kfactor")) 
	{
        g_k_factor =  (float)value/1000;
    }
	else 
	{ 
        rt_kprintf("Please input'setenv <name> <value>'\n");
    }
}
MSH_CMD_EXPORT(setvar, set var);


void stop(void)
{
	g_set_flow_val = 0;
    set_dac_output_voltage(0);
	rt_kprintf("stop stop");
}
MSH_CMD_EXPORT(stop,stop);


int main(void)
{
	uint16_t adc_data_tmp = 0;
	uint16_t adc_data = 0;

	uint8_t dis_buf[40];
	float volt_mV;
	float current_mA;
	float flow_rate = 0;
	
	rt_err_t err;

	float vlot_set_value_mV = 0;	
	static uint16_t count = 0;
	
	int32_t adc_value_unfiltered = 0;
	
	DAC1_Init();		// 初始化DAC1
	 /* set LED0 pin mode to output */	
    rt_pin_mode(LED0_PIN, PIN_MODE_OUTPUT);	
	rt_hw_ads8688_config();
	median_filter_init(&adc_median_filter, 10);
	
    while (1)
    {
		err = ads8688_get_man_ch_data(MAN_CH_0,&adc_data_tmp);
		if(err == RT_EOK)
		{
			adc_data = median_filter(&adc_median_filter, adc_data_tmp);
		}
		else
		{
			rt_kprintf("adc read error!\r\n");
		}
		
		volt_mV = ((float)adc_data-32767)*20480.0/65536;
		current_mA = volt_mV / 499;
		
		flow_rate = convert_current_to_flow(current_mA);
		
		flow_rate *= g_k_factor;
		
		if(flow_rate < 0)
		{
			flow_rate = 0;
		}
	
		if(count ++ >= 20)
		{
			count = 0;
			sprintf ((char *)dis_buf,"CH0: %10.4lfmV  D: %04X", volt_mV , (uint16_t)adc_data);
			rt_kprintf("%s\r\n", (char *)dis_buf);
			rt_kprintf("flow = %d\r\n", (int32_t)flow_rate);
		}
		
		if(flow_rate < g_set_flow_val - 500)
		{
			vlot_set_value_mV += 50;
		}
		else if(flow_rate < g_set_flow_val-100)
		{
			vlot_set_value_mV += 5;
		}
		else if(flow_rate < g_set_flow_val-20)
		{
			vlot_set_value_mV += 2;
		}
		else if(flow_rate < g_set_flow_val - 2)
		{
			vlot_set_value_mV += 0.1;
		}
		
		else if(flow_rate > g_set_flow_val + 500)
		{	
			vlot_set_value_mV -= 50;
		}
		else if(flow_rate > g_set_flow_val + 100)
		{	
			vlot_set_value_mV -= 5;
		}
		else if(flow_rate > g_set_flow_val + 20)
		{	
			vlot_set_value_mV -= 2;
		}
		else if(flow_rate > g_set_flow_val + 2)
		{	
			vlot_set_value_mV -= 0.1;
		}
		else if(g_set_flow_val == 0)
		{
			vlot_set_value_mV = 0;
		}
			
		
		if(vlot_set_value_mV < 0)
		{
			vlot_set_value_mV = 0;
		}
		else if(vlot_set_value_mV > 7000)
		{
			vlot_set_value_mV = 7000;
		}
		
		set_dac_output_voltage(vlot_set_value_mV);

		rt_thread_mdelay(100);
    }
    return RT_EOK;
}
