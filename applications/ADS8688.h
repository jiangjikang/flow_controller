
#ifndef __ADS8688_H
#define __ADS8688_H


#include "stm32h7xx_hal.h"
#include <rtdevice.h>
#include <drv_spi.h>
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

extern struct median_filter adc_median_filter; 

extern float g_set_flow_val;
extern float g_k_factor;

/* defined the LED0 pin: PB1 */
#define LED0_PIN    GET_PIN(A, 0)

#define ADS8688_DAISY_PIN 		GET_PIN(E, 3) 		/* 菊花链模式 */
#define ADS8688_CS_PIN 		GET_PIN(E, 4) 			/* CS片选引脚 */
#define ADS8688_RST_PIN		GET_PIN(C, 13)





void ads8688_write_command(uint16_t comm);
void ads8688_write_program(uint8_t addr,uint8_t data);
rt_err_t ads8688_get_man_ch_data(uint16_t ch, uint16_t *data);
int rt_hw_ads8688_config(void);
float convert_current_to_flow(float current);



#endif
