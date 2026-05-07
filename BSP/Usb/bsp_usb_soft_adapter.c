#include "bsp_usb_soft_adapter.h"

#include <stddef.h>

static uint8_t SoftStart(void *context)
{
    (void)context;
    return 1U;
}

static uint8_t SoftSend(void *context, uint8_t endpoint, const uint8_t *data, uint16_t length)
{
    BspUsbSoftAdapter *adapter = (BspUsbSoftAdapter *)context;

    if ((adapter == NULL) || (adapter->loopback_target == NULL))
    {
        return 0U;
    }

    BspUsb_OnPacketReceivedFromIsr(adapter->loopback_target, endpoint, data, length);
    return 1U;
}

static uint8_t SoftRearmRx(void *context, uint8_t endpoint)
{
    (void)context;
    (void)endpoint;
    return 1U;
}

void BspUsbSoftAdapter_MakeConfig(BspUsbConfig *config,
                                  BspUsbSoftAdapter *adapter,
                                  BspUsb *loopback_target,
                                  uint8_t auto_rearm_rx)
{
    if (config == NULL)
    {
        return;
    }

    if (adapter != NULL)
    {
        adapter->loopback_target = loopback_target;
    }

    config->ops.start = SoftStart;
    config->ops.send_packet = SoftSend;
    config->ops.rearm_rx = SoftRearmRx;
    config->low_level_context = adapter;
    config->auto_rearm_rx = auto_rearm_rx;
}
