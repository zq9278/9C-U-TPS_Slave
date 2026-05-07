#ifndef BSP_I2C_SOFT_ADAPTER_H
#define BSP_I2C_SOFT_ADAPTER_H

#include "bsp_i2c.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef void (*BspI2cSoftWriteLine)(void *context, uint8_t level);
typedef uint8_t (*BspI2cSoftReadLine)(void *context);
typedef void (*BspI2cSoftDelay)(void *context);

typedef struct
{
    BspI2cSoftWriteLine write_scl;
    BspI2cSoftWriteLine write_sda;
    BspI2cSoftReadLine read_sda;
    BspI2cSoftDelay delay;
    void *line_context;
} BspI2cSoftAdapter;

void BspI2cSoftAdapter_MakeConfig(BspI2cConfig *config,
                                  BspI2cSoftAdapter *adapter,
                                  uint32_t default_timeout_ms);

#ifdef __cplusplus
}
#endif

#endif
