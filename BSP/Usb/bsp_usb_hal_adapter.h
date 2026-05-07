#ifndef BSP_USB_HAL_ADAPTER_H
#define BSP_USB_HAL_ADAPTER_H

#include "bsp_usb.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef uint8_t (*BspUsbHalStartFn)(void *context);
typedef uint8_t (*BspUsbHalSendFn)(void *context, uint8_t endpoint, const uint8_t *data, uint16_t length);
typedef uint8_t (*BspUsbHalRearmRxFn)(void *context, uint8_t endpoint);

typedef struct
{
    BspUsbHalStartFn start;
    BspUsbHalSendFn send_blocking;
    BspUsbHalSendFn send_dma;
    BspUsbHalSendFn send_it;
    BspUsbHalRearmRxFn rearm_rx;
    void *usb_context;
} BspUsbHalAdapter;

typedef enum
{
    BSP_USB_HAL_MODE_BLOCKING = 0,
    BSP_USB_HAL_MODE_DMA,
    BSP_USB_HAL_MODE_IT
} BspUsbHalMode;

void BspUsbHalAdapter_MakeConfig(BspUsbConfig *config,
                                 BspUsbHalAdapter *adapter,
                                 BspUsbHalMode mode,
                                 uint8_t auto_rearm_rx);

#ifdef __cplusplus
}
#endif

#endif
