#ifndef BSP_CAN_HAL_ADAPTER_H
#define BSP_CAN_HAL_ADAPTER_H

#include "bsp_can.h"
#include "stm32g0xx_hal.h"

#ifdef __cplusplus
extern "C" {
#endif

#ifdef HAL_CAN_MODULE_ENABLED
typedef CAN_HandleTypeDef BspCanHalHandle;
#else
typedef void BspCanHalHandle;
#endif

typedef enum
{
    BSP_CAN_HAL_MODE_POLLING = 0,
    BSP_CAN_HAL_MODE_IT
} BspCanHalMode;

void BspCanHalAdapter_MakeConfig(BspCanConfig *config,
                                 BspCanHalHandle *hcan,
                                 BspCanHalMode mode,
                                 uint32_t default_timeout_ms);

uint8_t BspCanHalAdapter_ReadRxFifo0(BspCanHalHandle *hcan, BspCanFrame *frame);
uint8_t BspCanHalAdapter_ReadRxFifo1(BspCanHalHandle *hcan, BspCanFrame *frame);
uint32_t BspCanHalAdapter_GetError(BspCanHalHandle *hcan);

#ifdef __cplusplus
}
#endif

#endif
