#include "debug.h"
#include "rtthread.h"
#include "env.h"

uint32_t log_level = AN_INFO | CAL_INFO  ;
static int atoi(const char *nptr);


extern void write_handler(uint16_t reg_addr,uint16_t value);

long printenv(void)
{
    int32_t output_type = sysEnv.getEnvVar(CH1_OUTPUT_TYPE);
    int32_t meas_range_l = sysEnv.getEnvVar(MEAS_RANGE_L);
    int32_t meas_range_h = sysEnv.getEnvVar(MEAS_RANGE_H);
    int32_t an_range_l = sysEnv.getEnvVar(CH1_AN_RANGE_L);
    int32_t an_range_h = sysEnv.getEnvVar(CH1_AN_RANGE_H);
		int32_t res1 = sysEnv.getEnvVar(RES1);
	int32_t res2 = sysEnv.getEnvVar(RES2);
	int32_t res31 = sysEnv.getEnvVar(RES31);
	int32_t res32 = sysEnv.getEnvVar(RES32);
	int32_t res33 = sysEnv.getEnvVar(RES33);
	int32_t res34 = sysEnv.getEnvVar(RES34);
	int32_t volt_k1 =  sysEnv.getEnvVar(VOLTK1);
	int32_t volt_k2 =  sysEnv.getEnvVar(VOLTK2);
	int32_t volt_k3 =  sysEnv.getEnvVar(VOLTK3);
	int32_t volt_k4 =  sysEnv.getEnvVar(VOLTK4);
	int32_t current_k1 = sysEnv.getEnvVar(CURRENTK1);
	int32_t current_k2 = sysEnv.getEnvVar(CURRENTK2);
	int32_t current_k3 = sysEnv.getEnvVar(CURRENTK3);
	int32_t current_k4 = sysEnv.getEnvVar(CURRENTK4);
	
    rt_kprintf("output_type = %d\n", output_type);
    rt_kprintf("meas_range_l = %d\n", meas_range_l);
    rt_kprintf("meas_range_h = %d\n",meas_range_h);
    rt_kprintf("an_range_l = %d\n", an_range_l);
    rt_kprintf("an_range_h = %d\n", an_range_h);	
	rt_kprintf("res1 = %d\n", res1);
	rt_kprintf("res2 = %d\n", res2);
	rt_kprintf("res31 = %d\n", res31);
	rt_kprintf("res32 = %d\n", res32);
	rt_kprintf("res33 = %d\n", res33);
	rt_kprintf("res34 = %d\n", res34);
	rt_kprintf("volt_k1 = %d\n", volt_k1);	
	rt_kprintf("volt_k2 = %d\n", volt_k2);	
	rt_kprintf("volt_k3 = %d\n", volt_k3);	
	rt_kprintf("volt_k4 = %d\n", volt_k4);	
	rt_kprintf("current_k1 = %d\n", current_k1);
	rt_kprintf("current_k2 = %d\n", current_k2);
	rt_kprintf("current_k3 = %d\n", current_k3);
	rt_kprintf("current_k4 = %d\n", current_k4);
	
    return RT_EOK;
}
MSH_CMD_EXPORT(printenv, print env);



void setenv(int argc, char **argv)
{
    if (argc < 3) {
        rt_kprintf("Please input'setenv <name> <value>'\n");
        return;
    }
    int32_t value = atoi(argv[2]);
    if (!rt_strcmp(argv[1], "output")) {
        sysEnv.setEnvVar(CH1_OUTPUT_TYPE, value);
    } else if (!rt_strcmp(argv[1], "mrangel")) {
        sysEnv.setEnvVar(MEAS_RANGE_L, value);
    } else if (!rt_strcmp(argv[1], "mrangeh")) {
        sysEnv.setEnvVar(MEAS_RANGE_H, value);
    }else if(!rt_strcmp(argv[1], "anrangel")){
        sysEnv.setEnvVar(CH1_AN_RANGE_L, value);
    }else if (!rt_strcmp(argv[1], "anrangeh")) {
        sysEnv.setEnvVar(CH1_AN_RANGE_H, value);
    }else if (!rt_strcmp(argv[1], "res1")) {
        sysEnv.setEnvVar(RES1, value);
    }else if (!rt_strcmp(argv[1], "res2")) {
        sysEnv.setEnvVar(RES2, value);
    }else if (!rt_strcmp(argv[1], "res31")) {
        sysEnv.setEnvVar(RES31, value);
    }else if (!rt_strcmp(argv[1], "res32")) {
        sysEnv.setEnvVar(RES32, value);
    }else if (!rt_strcmp(argv[1], "res33")) {
        sysEnv.setEnvVar(RES33, value);
    }else if (!rt_strcmp(argv[1], "res34")) {
        sysEnv.setEnvVar(RES34, value);
    }else if (!rt_strcmp(argv[1], "voltk1")) {
        sysEnv.setEnvVar(VOLTK1, value);
    }else if (!rt_strcmp(argv[1], "voltk2")) {
        sysEnv.setEnvVar(VOLTK2, value);
    }else if (!rt_strcmp(argv[1], "voltk3")) {
        sysEnv.setEnvVar(VOLTK3, value);
    }else if (!rt_strcmp(argv[1], "voltk4")) {
        sysEnv.setEnvVar(VOLTK4, value);
    }else if (!rt_strcmp(argv[1], "currentk1")) {
        sysEnv.setEnvVar(CURRENTK1, value);
    }else if (!rt_strcmp(argv[1], "currentk2")) {
        sysEnv.setEnvVar(CURRENTK2, value);
    }else if (!rt_strcmp(argv[1], "currentk3")) {
        sysEnv.setEnvVar(CURRENTK3, value);
    }else if (!rt_strcmp(argv[1], "currentk4")) {
        sysEnv.setEnvVar(CURRENTK4, value);
    }else if (!rt_strcmp(argv[1], "antest")) {
        write_handler(50,1235);
		write_handler(87,value);
    }else { 
        rt_kprintf("Please input'setenv <name> <value>'\n");
    }
}
MSH_CMD_EXPORT(setenv, set env);


void saveenv(void)
{
    sysEnv.save();
    rt_kprintf("saving env ok\n");
}
MSH_CMD_EXPORT(saveenv, save env);


long restart(void)
{
    NVIC_SystemReset();
    return RT_EOK;
}
MSH_CMD_EXPORT(restart,system restart);



	



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





