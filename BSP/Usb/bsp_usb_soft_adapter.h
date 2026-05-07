#ifndef BSP_USB_SOFT_ADAPTER_H
#define BSP_USB_SOFT_ADAPTER_H

#include "bsp_usb.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef struct
{
    BspUsb *loopback_target;
} BspUsbSoftAdapter;

void BspUsbSoftAdapter_MakeConfig(BspUsbConfig *config,
                                  BspUsbSoftAdapter *adapter,
                                  BspUsb *loopback_target,
                                  uint8_t auto_rearm_rx);

#ifdef __cplusplus
}
#endif

#endif
