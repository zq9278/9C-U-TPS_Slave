#include "bsp_usb_hal_adapter.h"

#include <stddef.h>

static uint8_t UsbHal_Start(void *context)
{
    BspUsbHalAdapter *adapter = (BspUsbHalAdapter *)context;

    if ((adapter == NULL) || (adapter->start == NULL))
    {
        return 0U;
    }

    return adapter->start(adapter->usb_context);
}

static uint8_t UsbHal_SendBlocking(void *context, uint8_t endpoint, const uint8_t *data, uint16_t length)
{
    BspUsbHalAdapter *adapter = (BspUsbHalAdapter *)context;

    if ((adapter == NULL) || (adapter->send_blocking == NULL))
    {
        return 0U;
    }

    return adapter->send_blocking(adapter->usb_context, endpoint, data, length);
}

static uint8_t UsbHal_SendDma(void *context, uint8_t endpoint, const uint8_t *data, uint16_t length)
{
    BspUsbHalAdapter *adapter = (BspUsbHalAdapter *)context;

    if ((adapter == NULL) || (adapter->send_dma == NULL))
    {
        return 0U;
    }

    return adapter->send_dma(adapter->usb_context, endpoint, data, length);
}

static uint8_t UsbHal_SendIt(void *context, uint8_t endpoint, const uint8_t *data, uint16_t length)
{
    BspUsbHalAdapter *adapter = (BspUsbHalAdapter *)context;

    if ((adapter == NULL) || (adapter->send_it == NULL))
    {
        return 0U;
    }

    return adapter->send_it(adapter->usb_context, endpoint, data, length);
}

static uint8_t UsbHal_RearmRx(void *context, uint8_t endpoint)
{
    BspUsbHalAdapter *adapter = (BspUsbHalAdapter *)context;

    if ((adapter == NULL) || (adapter->rearm_rx == NULL))
    {
        return 0U;
    }

    return adapter->rearm_rx(adapter->usb_context, endpoint);
}

void BspUsbHalAdapter_MakeConfig(BspUsbConfig *config,
                                 BspUsbHalAdapter *adapter,
                                 BspUsbHalMode mode,
                                 uint8_t auto_rearm_rx)
{
    if (config == NULL)
    {
        return;
    }

    config->ops.start = UsbHal_Start;
    config->ops.rearm_rx = UsbHal_RearmRx;
    config->low_level_context = adapter;
    config->auto_rearm_rx = auto_rearm_rx;

    switch (mode)
    {
    case BSP_USB_HAL_MODE_DMA:
        config->ops.send_packet = UsbHal_SendDma;
        break;
    case BSP_USB_HAL_MODE_IT:
        config->ops.send_packet = UsbHal_SendIt;
        break;
    case BSP_USB_HAL_MODE_BLOCKING:
    default:
        config->ops.send_packet = UsbHal_SendBlocking;
        break;
    }
}
