#ifndef BSP_I2C_HAL_ADAPTER_H
#define BSP_I2C_HAL_ADAPTER_H

#include "bsp_i2c.h"
#include "stm32g0xx_hal.h"

#ifdef __cplusplus
extern "C" {
#endif

#ifdef HAL_I2C_MODULE_ENABLED
typedef I2C_HandleTypeDef BspI2cHalHandle;
#else
typedef void BspI2cHalHandle;
#endif

typedef enum
{
    BSP_I2C_HAL_MODE_BLOCKING = 0,
    BSP_I2C_HAL_MODE_DMA,
    BSP_I2C_HAL_MODE_IT
} BspI2cHalMode;

void BspI2cHalAdapter_MakeConfig(BspI2cConfig *config,
                                 BspI2cHalHandle *hi2c,
                                 BspI2cHalMode mode,
                                 uint32_t default_timeout_ms);

#ifdef __cplusplus
}
#endif

#endif
