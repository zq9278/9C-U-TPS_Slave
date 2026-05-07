#include "bsp_can_soft_adapter.h"

#include <stddef.h>

static uint8_t SoftStart(void *context)
{
    (void)context;
    return 1U;
}

static uint8_t SoftSend(void *context, const BspCanFrame *frame, uint32_t timeout_ms)
{
    BspCanSoftAdapter *adapter = (BspCanSoftAdapter *)context;
    (void)timeout_ms;

    if ((adapter == NULL) || (adapter->loopback_target == NULL) || (frame == NULL))
    {
        return 0U;
    }

    BspCan_OnRxFrameFromIsr(adapter->loopback_target, frame);
    return 1U;
}

static uint8_t SoftRecover(void *context)
{
    (void)context;
    return 1U;
}

void BspCanSoftAdapter_MakeConfig(BspCanConfig *config,
                                  BspCanSoftAdapter *adapter,
                                  BspCan *loopback_target,
                                  uint32_t default_timeout_ms)
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
    config->ops.send = SoftSend;
    config->ops.recover = SoftRecover;
    config->low_level_context = adapter;
    config->default_timeout_ms = default_timeout_ms;
    config->auto_recover_on_error = 0U;
}
