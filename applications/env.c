#include "env.h"
#include "rtthread.h"


SysEnv sysEnv = {	
    .varSet.powerOnSign = 0x1234,     /* 增、删、修改环境变量，需要修改该值 */  // 0x02
    .varSet.ch1_output_type = 1,
	.varSet.ch2_output_type = 1,
    .varSet.meas_range_l = 0,
    .varSet.meas_range_h = 800,
    .varSet.ch1_an_range_l = 300,
	.varSet.ch2_an_range_l = 300,
    .varSet.ch1_an_range_h = 3500,
	.varSet.ch2_an_range_h = 3500,
	.varSet.res1 = 20000, //单位 Ω
	.varSet.res2 = 30000,//单位 Ω
	.varSet.res31 = 2460,//单位 0.1Ω  //1326
	.varSet.res32 = 2455, //单位 0.1Ω  // 1329
	.varSet.res33 = 2455,//单位 0.1Ω  //1329
	.varSet.res34 = 2500,//单位 0.1Ω  //1327
	.varSet.voltk1 = 1234,
	.varSet.voltk2 = 1234,
	.varSet.voltk3 = 1234,
	.varSet.voltk4 = 1234,
	.varSet.currentk1 = 130, 
	.varSet.currentk2 = 110, 
	.varSet.currentk3 = 110, 
	.varSet.currentk4 = 185,
	
	.varSet.volt_cal_point_x1 = 246,    // 00 f6
	.varSet.volt_cal_point_x2 = 1641,   // 06 69
	.varSet.volt_cal_point_x3 = 2876,   // 0b 3c
	
	.varSet.volt_cal_point_y1 = 300, 
	.varSet.volt_cal_point_y2 = 2000, 
	.varSet.volt_cal_point_y3 = 3500, 
	
	.varSet.current_cal_point_x1 = 1000, 
	.varSet.current_cal_point_x2 = 2000, 
	.varSet.current_cal_point_x3 = 3000, 
	.varSet.current_cal_point_y1 = 1000, 
	.varSet.current_cal_point_y2 = 2000, 
	.varSet.current_cal_point_y3 = 3000, 
};


int32_t writeFlash(uint32_t addr, uint8_t *buf, size_t size)
{
    HAL_StatusTypeDef status;
    uint32_t sector;
    uint32_t sector_error = 0;

    FLASH_EraseInitTypeDef EraseInitStruct;

    if (addr < 0x08020000)
        sector = FLASH_SECTOR_0;
    else if (addr < 0x08040000)	
        sector = FLASH_SECTOR_1;
    else
        sector = FLASH_SECTOR_2;   

    HAL_FLASH_Unlock();

    __HAL_FLASH_CLEAR_FLAG(FLASH_FLAG_ALL_ERRORS_BANK1);
		__HAL_FLASH_CLEAR_FLAG(FLASH_FLAG_ALL_ERRORS_BANK2);

    EraseInitStruct.TypeErase    = FLASH_TYPEERASE_SECTORS;
    EraseInitStruct.Sector       = sector;
    EraseInitStruct.NbSectors    = 1;
    EraseInitStruct.VoltageRange = FLASH_VOLTAGE_RANGE_3;

    status = HAL_FLASHEx_Erase(&EraseInitStruct, &sector_error);
    if (status != HAL_OK)
    {
        HAL_FLASH_Lock();
        return RT_ERROR;
    }
    uint32_t addr_copy = addr;
    uint32_t i = 0;

    for (i = 0; i < size; i += 32)
    {
        status = HAL_FLASH_Program(FLASH_TYPEPROGRAM_FLASHWORD,
                                   addr_copy,
                                   (uint32_t)(buf + i));

        if (status != HAL_OK)
        {
            HAL_FLASH_Lock();
            return RT_ERROR;
        }

        addr_copy += 32;
    }

    HAL_FLASH_Lock();

    for (i = 0; i < size; i++)
    {
        if (*(uint8_t *)(addr + i) != buf[i])
        {
            return RT_ERROR;
        }
    }

    return RT_EOK;
}


void readFlash(uint32_t addr, int32_t *buf, size_t size)
{
    if (size % 4 != 0) {
        return;
    }

    for (; size > 0; size -= 4, addr += 4, buf++) {
        *buf = * (volatile int32_t *) addr;
    }
}


static void setEnvVar(SysEnvVar var, int32_t val)
{
    rt_enter_critical();
    int32_t *varSet = (int32_t *)(& (sysEnv.varSet));
    if (varSet[var] != val) {
        varSet[var] = val;
        sysEnv.envCacheChanged = RT_TRUE;
    }
    rt_exit_critical();

}

static int32_t getEnvVar(SysEnvVar var)
{
    rt_enter_critical();
    int32_t *varSet = (int32_t *)(& (sysEnv.varSet));
    int32_t val = varSet[var];
    rt_exit_critical();

    return val;
}



static void save(void)
{
    rt_base_t level;
    if (sysEnv.envCacheChanged == RT_TRUE) {
        rt_enter_critical();
        writeFlash(FLASH_USER_START_ADDR,
           (uint8_t *)(&(sysEnv.varSet)),
           sizeof(sysEnv.varSet));
        rt_exit_critical();
        sysEnv.envCacheChanged = RT_FALSE;
    }
}


int sysEnvInit(void)
{
    sysEnv.envCacheChanged = RT_FALSE;
    sysEnv.getEnvVar = getEnvVar;
    sysEnv.save = save;
    sysEnv.setEnvVar = setEnvVar;

    int32_t powerOnSign = 0;
    readFlash(FLASH_USER_START_ADDR, &powerOnSign, sizeof(powerOnSign));

    if (powerOnSign != sysEnv.varSet.powerOnSign) {
        sysEnv.envCacheChanged = RT_TRUE;
        save();
    } else {
        readFlash(FLASH_USER_START_ADDR,
                  (int32_t *)(& (sysEnv.varSet)),
                  sizeof(sysEnv.varSet));
    }
    return 0;
}
INIT_PREV_EXPORT(sysEnvInit);

