#ifndef BSP_CAN_SOFT_ADAPTER_H
#define BSP_CAN_SOFT_ADAPTER_H

#include "bsp_can.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef struct
{
    BspCan *loopback_target;
} BspCanSoftAdapter;

void BspCanSoftAdapter_MakeConfig(BspCanConfig *config,
                                  BspCanSoftAdapter *adapter,
                                  BspCan *loopback_target,
                                  uint32_t default_timeout_ms);

#ifdef __cplusplus
}
#endif

#endif
