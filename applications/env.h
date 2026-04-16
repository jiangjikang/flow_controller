#ifndef __ENV_H
#define __ENV_H


#include "rtthread.h"
#include "stm32h7xx_hal.h"

#define ADDR_FLASH_PAGE_128 		((uint32_t)0x803F800)
#define FLASH_USER_START_ADDR 	ADDR_FLASH_PAGE_128
#define FLASH_USER_END_ADDR 	ADDR_FLASH_PAGE_128 + FLASH_PAGE_SIZE

#define PAGE_SIZE 	FLASH_PAGE_SIZE		//页大小   - 2K

typedef enum SysEnvVar_EN
{
    PowerOnSign = 0,
	CH1_OUTPUT_TYPE,
	CH2_OUTPUT_TYPE,
	MEAS_RANGE_L,
	MEAS_RANGE_H,
	CH1_AN_RANGE_L,
	CH2_AN_RANGE_L,
	CH1_AN_RANGE_H,
	CH2_AN_RANGE_H,
	RES1,
	RES2,
	RES31,
	RES32,
	RES33,
	RES34,
	VOLTK1,
	VOLTK2,
	VOLTK3,
	VOLTK4,
	CURRENTK1,
	CURRENTK2,
	CURRENTK3,
	CURRENTK4,
	VOLT_CAL_POINT_X1,
	VOLT_CAL_POINT_X2,
	VOLT_CAL_POINT_X3,
	VOLT_CAL_POINT_Y1,
	VOLT_CAL_POINT_Y2,
	VOLT_CAL_POINT_Y3,
	
	CURRENT_CAL_POINT_X1,
	CURRENT_CAL_POINT_X2,
	CURRENT_CAL_POINT_X3,
	CURRENT_CAL_POINT_Y1,
	CURRENT_CAL_POINT_Y2,
	CURRENT_CAL_POINT_Y3,
	
} SysEnvVar;

/* default system environment variables set */
#pragma pack(1)
typedef struct SysEnv_ST
{
    struct VarSet
    {
        int32_t powerOnSign; // 首次上电标志
		int32_t ch1_output_type;
		int32_t ch2_output_type;
        int32_t meas_range_l;
		int32_t meas_range_h;
		int32_t ch1_an_range_l;
		int32_t ch2_an_range_l;
		int32_t ch1_an_range_h;
		int32_t ch2_an_range_h;
		int32_t res1;
		int32_t res2;
		int32_t res31;
		int32_t res32;
		int32_t res33;
		int32_t res34;
		int32_t voltk1;
		int32_t voltk2;
		int32_t voltk3;
		int32_t voltk4;
		int32_t currentk1;
		int32_t currentk2;
		int32_t currentk3;
		int32_t currentk4;
		int32_t volt_cal_point_x1;
		int32_t volt_cal_point_x2;
		int32_t volt_cal_point_x3;
		int32_t volt_cal_point_y1;
		int32_t volt_cal_point_y2;
		int32_t volt_cal_point_y3;
		
		int32_t current_cal_point_x1;
		int32_t current_cal_point_x2;
		int32_t current_cal_point_x3;
		int32_t current_cal_point_y1;
		int32_t current_cal_point_y2;
		int32_t current_cal_point_y3;
    } varSet;

    rt_bool_t envCacheChanged;

    void (*setEnvVar)(SysEnvVar, int32_t);
    int32_t (*getEnvVar)(SysEnvVar);
    void (*save)(void);

} SysEnv;
#pragma pack()

extern SysEnv sysEnv;
extern int32_t writeFlash(uint32_t addr, uint8_t *buf, size_t size);
extern void readFlash(uint32_t addr, int32_t *buf, size_t size);




#endif

