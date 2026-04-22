#include "analog.h"
#include <rtdevice.h>
#include <board.h>
#include "env.h"
#include "modbus.h"
#include "user_table.h"

#include "filter.h"

#define SW_EN_PIN    	GET_PIN(B, 13)
#define SW_A_PIN    	GET_PIN(B, 14)
#define SW_B_PIN    	GET_PIN(B, 15)

#define ADC_DEV_NAME        "adc1"
#define ADC_DEV_CHANNEL     10
#define REFER_VOLTAGE       3300
#define CONVERT_BITS        (1 << 12)


extern rt_mutex_t dynamic_mutex;

rt_adc_device_t adc_dev;

static rt_timer_t timer1 = RT_NULL;
static rt_sem_t timeout_sem = RT_NULL;

static int32_t res1 = 0;
static int32_t res2 = 0;
static int32_t res31 = 0;
static int32_t res32 = 0;
static int32_t res33 = 0;
static int32_t res34 = 0;
static int32_t vlot_k1 = 0;
static int32_t vlot_k2 = 0;
static int32_t vlot_k3 = 0;
static int32_t vlot_k4 = 0;
static int32_t current_k1 = 0;
static int32_t current_k2 = 0;
static int32_t current_k3 = 0;
static int32_t current_k4 = 0;

static uint16_t analog_value[MB_MASTER_TOTAL_SLAVE_NUM];
uint16_t analog_value_uncalib[MB_MASTER_TOTAL_SLAVE_NUM];
struct sliding_average_filter an_filter[MB_MASTER_TOTAL_SLAVE_NUM];


#define  ADC_BUF_LEN 100

uint32_t adc_buf[ADC_BUF_LEN];


uint16_t adc_read_voltage(uint8_t channel)
{
    uint32_t value = 0;
    static uint8_t last_channel = 0;
	
    rt_pin_write(SW_EN_PIN, PIN_LOW);  // enable
	
	
    switch(channel)
    {
    case 1:
        rt_pin_write(SW_B_PIN, PIN_LOW);
        rt_pin_write(SW_A_PIN, PIN_LOW);
        break;
    case 2:
        rt_pin_write(SW_B_PIN, PIN_LOW);
        rt_pin_write(SW_A_PIN, PIN_HIGH);
        break;
    case 3:
        rt_pin_write(SW_B_PIN, PIN_HIGH);
        rt_pin_write(SW_A_PIN, PIN_LOW);
        break;
    case 4:
        rt_pin_write(SW_B_PIN, PIN_HIGH);
        rt_pin_write(SW_A_PIN, PIN_HIGH);
        break;
	default:
		break;
    }
    rt_adc_enable(adc_dev, ADC_DEV_CHANNEL);
    rt_thread_mdelay(70);

    if(channel != last_channel)
    {
        rt_thread_mdelay(100);
    }

    last_channel = channel;

    for(uint16_t i = 0; i < ADC_BUF_LEN; i++)
    {
        adc_buf[i] = rt_adc_read(adc_dev, ADC_DEV_CHANNEL);
    }

    value = 0;

    for(uint16_t i = 0; i < ADC_BUF_LEN ; i++) {
        value += adc_buf[i];
    }
    value /= 100;
    value = (uint64_t)value * REFER_VOLTAGE / CONVERT_BITS;

    rt_adc_disable(adc_dev, ADC_DEV_CHANNEL);
   rt_pin_write(SW_EN_PIN, PIN_HIGH);  			// disable

    return value;
}


uint16_t analog_calib(enum analog_type type,uint16_t value)
{
	int32_t output_value = 0;
	int32_t cal_point_x[3];
	int32_t cal_point_y[3];
	if(type == VOLT_TYPE)
    {
		cal_point_x[0] = sysEnv.getEnvVar(VOLT_CAL_POINT_X1);
		cal_point_x[1] = sysEnv.getEnvVar(VOLT_CAL_POINT_X2);
		cal_point_x[2] = sysEnv.getEnvVar(VOLT_CAL_POINT_X3);	
		cal_point_y[0] = sysEnv.getEnvVar(VOLT_CAL_POINT_Y1);
		cal_point_y[1] = sysEnv.getEnvVar(VOLT_CAL_POINT_Y2);
		cal_point_y[2] = sysEnv.getEnvVar(VOLT_CAL_POINT_Y3);
	}
	else if(type == CURRENT_TYPE)
	{
		cal_point_x[0] = sysEnv.getEnvVar(CURRENT_CAL_POINT_X1);
		cal_point_x[1] = sysEnv.getEnvVar(CURRENT_CAL_POINT_X2);
		cal_point_x[2] = sysEnv.getEnvVar(CURRENT_CAL_POINT_X3);	
		cal_point_y[0] = sysEnv.getEnvVar(CURRENT_CAL_POINT_Y1);
		cal_point_y[1] = sysEnv.getEnvVar(CURRENT_CAL_POINT_Y2);
		cal_point_y[2] = sysEnv.getEnvVar(CURRENT_CAL_POINT_Y3);
	}
	if(value < cal_point_x[1])
	{
		output_value = (int32_t)(value - cal_point_x[0] ) 	\
               *(cal_point_y[1] - cal_point_y[0])			\
               /(cal_point_x[1] -  cal_point_x[0])       	\
               + cal_point_y[0];
	}
	else if(value >= cal_point_x[1])
	{
		output_value = (int32_t)(value - cal_point_x[1] )   \
               *(cal_point_y[2] - cal_point_y[1])           \
               /(cal_point_x[2] -  cal_point_x[1])        	\
               + cal_point_y[1];
	}
	
	if(output_value < 0)
	{
		output_value = 0;
	}
	if(output_value > RT_UINT16_MAX)
	{
		output_value = RT_UINT16_MAX;
	}
	
	return output_value;
}



/* 定时器 1 超时函数 */
static void timeout1(void *parameter)
{
    rt_sem_release(timeout_sem);
}


void analog_thread_task_entry(void *param)
{
    rt_err_t result;
	uint16_t voltage = 0;

    sliding_average_filter_init(&an_filter[0],3);
    sliding_average_filter_init(&an_filter[1],3);
    sliding_average_filter_init(&an_filter[2],3);
    timeout_sem = rt_sem_create("timeout_sem", 0, RT_IPC_FLAG_PRIO);
    if (timeout_sem == RT_NULL)
    {
        rt_kprintf("create timeout_sem semaphore failed.\n");
    }

    timer1 = rt_timer_create("timer1", timeout1,
                             RT_NULL, 1000,
                             RT_TIMER_FLAG_PERIODIC);

    /* 启动定时器 1 */
    if (timer1 != RT_NULL)
        rt_timer_start(timer1);

    while(1)
    {
        result = rt_sem_take(timeout_sem, RT_WAITING_FOREVER);
        if (result != RT_EOK)
        {
            rt_kprintf("take timeout_sem failed.\n");
        }
        uint16_t output_type = sysEnv.getEnvVar(CH1_OUTPUT_TYPE);

        if(output_type!=VOLT_TYPE && output_type!=CURRENT_TYPE)
		{
			output_type = sysEnv.getEnvVar(CH2_OUTPUT_TYPE);
			if(output_type != VOLT_TYPE && output_type!=CURRENT_TYPE)
			{
				continue;
			}
		}
		
        uint16_t id = 1;
        do
        {
            rt_mutex_take(dynamic_mutex, RT_WAITING_FOREVER);
            voltage = adc_read_voltage(slave[id].channel);
			analog_value_uncalib[id-1] = voltage;
		    analog_value[id-1] = analog_calib(output_type,voltage);
            rt_mutex_release(dynamic_mutex);
            analog_value[id-1]= sliding_average_filter(&an_filter[id-1], analog_value[id-1]);
        } while(++id <= MB_MASTER_TOTAL_SLAVE_NUM);
    }
}


uint16_t get_analog_value_calib(uint8_t channel)
{
    return analog_value[channel-1];
}


uint16_t get_analog_value_uncalib(uint8_t channel)
{
    return analog_value_uncalib[channel-1];
}



static int analog_collector_init(void)
{

    rt_uint32_t value, vol;
    rt_err_t ret = RT_EOK;

    res1 = sysEnv.getEnvVar(RES1);
    res2 = sysEnv.getEnvVar(RES2);
    res31 = sysEnv.getEnvVar(RES31);
    res32 = sysEnv.getEnvVar(RES32);
    res33 = sysEnv.getEnvVar(RES33);
    res34 = sysEnv.getEnvVar(RES34);
    vlot_k1 = sysEnv.getEnvVar(VOLTK1);
    vlot_k2 = sysEnv.getEnvVar(VOLTK2);
    vlot_k3 = sysEnv.getEnvVar(VOLTK3);
    vlot_k4 = sysEnv.getEnvVar(VOLTK4);

    current_k1 = sysEnv.getEnvVar(CURRENTK1);
    current_k2 = sysEnv.getEnvVar(CURRENTK2);
    current_k3 = sysEnv.getEnvVar(CURRENTK3);
    current_k4 = sysEnv.getEnvVar(CURRENTK4);

    rt_pin_mode(SW_EN_PIN, PIN_MODE_OUTPUT);
    rt_pin_mode(SW_A_PIN, PIN_MODE_OUTPUT);
    rt_pin_mode(SW_B_PIN, PIN_MODE_OUTPUT);

    rt_pin_write(SW_EN_PIN, PIN_LOW);

    rt_pin_write(SW_B_PIN, PIN_LOW);
    rt_pin_write(SW_A_PIN, PIN_LOW);

    adc_dev = (rt_adc_device_t)rt_device_find(ADC_DEV_NAME);
    if (adc_dev == RT_NULL)
    {
        rt_kprintf("can't find %s device!\n", ADC_DEV_NAME);
        return RT_ERROR;
    }

    return ret;
}

//INIT_ENV_EXPORT(analog_collector_init);



static int analog_thread_init(void)
{
    rt_thread_t analog_thread = RT_NULL;

    analog_thread = rt_thread_create("analog_thread",analog_thread_task_entry, RT_NULL,1024,6, 20);

    if (analog_thread != RT_NULL)
        rt_thread_startup(analog_thread);
}
INIT_APP_EXPORT(analog_thread_init);


