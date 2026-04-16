#include "pwm.h"
#include <rtdevice.h>
#include <board.h>
#include "modbus.h"
#include "filter.h"
#include "env.h"


#define PWM_CHNNEL_NUM  MB_MASTER_TOTAL_SLAVE_NUM
#define HWTIMER_DEV_NAME   "timer3"     /* 定时器名称 */
rt_device_t hwtimer_dev = RT_NULL;   	/* 定时器设备句柄 */


static rt_sem_t timeout_sem = RT_NULL;
static uint16_t  pulse_cnt[PWM_CHNNEL_NUM] = {0,0,0};
static uint16_t  pulse_cnt_copy[PWM_CHNNEL_NUM] = {0,0,0};

struct sliding_average_filter pwm_filter[PWM_CHNNEL_NUM];


void hwtimer_start(void);
int pulse_cap_init(void);

void pulse_cap_1_callback(void *args)
{
    pulse_cnt[0]++;

}
void pulse_cap_2_callback(void *args)
{
    pulse_cnt[1]++;

}
void pulse_cap_3_callback(void *args)
{
    pulse_cnt[2]++;
}


/* 定时器超时回调函数 */
static rt_err_t timeout_cb(rt_device_t dev, rt_size_t size)
{
    uint16_t an_range_h = sysEnv.getEnvVar(CH1_AN_RANGE_H);
    for(uint16_t i = 0; i<PWM_CHNNEL_NUM; i++)
    {
        pulse_cnt_copy[i] = pulse_cnt[i];
        if(pulse_cnt_copy[i] > an_range_h + 100)
        {
            pulse_cnt_copy[i] = an_range_h + 100;
        }
        pulse_cnt_copy[i] = sliding_average_filter(&pwm_filter[i], pulse_cnt_copy[i]);
    }
    rt_sem_release(timeout_sem);
    return 0;
}

void hwtimer_start(void)
{
    rt_hwtimerval_t timeout_s;      /* 定时器超时值 */
    /* 设置定时器超时值为5s并启动定时器 */
    timeout_s.sec = 1;      /* 秒 */
    timeout_s.usec = 0;     /* 微秒 */

    if (rt_device_write(hwtimer_dev, 0, &timeout_s, sizeof(timeout_s)) != sizeof(timeout_s))
    {
        rt_kprintf("set timeout value failed\n");
    }

}

void pwm_task_entry(void *param)
{
    rt_err_t result;

    sliding_average_filter_init(&pwm_filter[0],3);
    sliding_average_filter_init(&pwm_filter[1],3);
    sliding_average_filter_init(&pwm_filter[2],3);

    pulse_cap_init();
    while(1)
    {
        for(uint16_t i = 0; i < PWM_CHNNEL_NUM; i++)
        {
            pulse_cnt[i] = 0;
        }

        hwtimer_start();
		
        result = rt_sem_take(timeout_sem, RT_WAITING_FOREVER);
        if (result != RT_EOK)
        {
            rt_kprintf("take timeout_sem failed.\n");
        }

//		rt_kprintf("pwmflow1 = %d\n",pulse_cnt_copy[0]);
//		rt_kprintf("pwmflow2 = %d\n",pulse_cnt_copy[1]);
//		rt_kprintf("pwmflow3 = %d\n",pulse_cnt_copy[2]);

    }
}


rt_err_t get_pwm_count(uint16_t *buf)
{
    for(uint16_t i = 0; i < PWM_CHNNEL_NUM; i++)
    {
        buf[i] = pulse_cnt_copy[i];
    }
    return RT_EOK;
}


int pulse_cap_init(void)
{
    rt_pin_mode(PULSE_CAP_1_PIN_NUM, PIN_MODE_INPUT_PULLUP);
    rt_pin_attach_irq(PULSE_CAP_1_PIN_NUM, PIN_IRQ_MODE_FALLING, pulse_cap_1_callback, RT_NULL);
    rt_pin_irq_enable(PULSE_CAP_1_PIN_NUM, PIN_IRQ_ENABLE);

    rt_pin_mode(PULSE_CAP_2_PIN_NUM, PIN_MODE_INPUT_PULLUP);
    rt_pin_attach_irq(PULSE_CAP_2_PIN_NUM, PIN_IRQ_MODE_FALLING, pulse_cap_2_callback, RT_NULL);
    rt_pin_irq_enable(PULSE_CAP_2_PIN_NUM, PIN_IRQ_ENABLE);

    rt_pin_mode(PULSE_CAP_3_PIN_NUM, PIN_MODE_INPUT_PULLUP);
    rt_pin_attach_irq(PULSE_CAP_3_PIN_NUM, PIN_IRQ_MODE_FALLING, pulse_cap_3_callback, RT_NULL);
    rt_pin_irq_enable(PULSE_CAP_3_PIN_NUM, PIN_IRQ_ENABLE);


    timeout_sem = rt_sem_create("timeout_sem", 0, RT_IPC_FLAG_PRIO);
    if (timeout_sem == RT_NULL)
    {
        rt_kprintf("create timeout_sem semaphore failed.\n");
        return -1;
    }

    rt_err_t ret = RT_EOK;

    rt_hwtimer_mode_t mode;         /* 定时器模式 */
    rt_uint32_t freq = 10000;               /* 计数频率 */

    /* 查找定时器设备 */
    hwtimer_dev = rt_device_find(HWTIMER_DEV_NAME);
    if (hwtimer_dev == RT_NULL)
    {
        rt_kprintf("hwtimer sample run failed! can't find %s device!\n", HWTIMER_DEV_NAME);
        return RT_ERROR;
    }

    /* 以读写方式打开设备 */
    ret = rt_device_open(hwtimer_dev, RT_DEVICE_OFLAG_RDWR);
    if (ret != RT_EOK)
    {
        rt_kprintf("open %s device failed!\n", HWTIMER_DEV_NAME);
        return ret;
    }

    /* 设置超时回调函数 */
    rt_device_set_rx_indicate(hwtimer_dev, timeout_cb);

    /* 设置计数频率(若未设置该项，默认为1Mhz 或 支持的最小计数频率) */
    rt_device_control(hwtimer_dev, HWTIMER_CTRL_FREQ_SET, &freq);
    /* 设置模式为周期性定时器（若未设置，默认是HWTIMER_MODE_ONESHOT）*/
    mode = HWTIMER_MODE_ONESHOT;
    ret = rt_device_control(hwtimer_dev, HWTIMER_CTRL_MODE_SET, &mode);
    if (ret != RT_EOK)
    {
        rt_kprintf("set mode failed! ret is :%d\n", ret);
        return ret;
    }
}


static int pwm_thread_init(void)
{
    static rt_thread_t pwm_thread = RT_NULL;

    pwm_thread = rt_thread_create("pwm_thread",pwm_task_entry, RT_NULL,1024,6, 20);

    if (pwm_thread != RT_NULL)
        rt_thread_startup(pwm_thread);

}
INIT_APP_EXPORT(pwm_thread_init);

