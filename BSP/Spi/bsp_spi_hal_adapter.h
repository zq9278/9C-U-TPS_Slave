#ifndef BSP_SPI_HAL_ADAPTER_H
#define BSP_SPI_HAL_ADAPTER_H

#include "bsp_spi.h"
#include "stm32g0xx_hal.h"

#ifdef __cplusplus
extern "C" {
#endif

#ifdef HAL_SPI_MODULE_ENABLED
typedef SPI_HandleTypeDef BspSpiHalHandle;
#else
typedef void BspSpiHalHandle;
#endif

typedef enum
{
    BSP_SPI_HAL_MODE_BLOCKING = 0,
    BSP_SPI_HAL_MODE_DMA,
    BSP_SPI_HAL_MODE_IT
} BspSpiHalMode;

void BspSpiHalAdapter_MakeConfig(BspSpiConfig *config,
                                 BspSpiHalHandle *hspi,
                                 BspSpiHalMode mode,
                                 BspSpiChipSelectCallback chip_select,
                                 void *chip_select_context,
                                 uint8_t auto_chip_select,
                                 uint32_t default_timeout_ms);

#ifdef __cplusplus
}
#endif

#endif
