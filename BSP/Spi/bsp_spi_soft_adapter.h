#ifndef BSP_SPI_SOFT_ADAPTER_H
#define BSP_SPI_SOFT_ADAPTER_H

#include "bsp_spi.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef void (*BspSpiSoftWriteLine)(void *context, uint8_t level);
typedef uint8_t (*BspSpiSoftReadLine)(void *context);
typedef void (*BspSpiSoftDelay)(void *context);

typedef struct
{
    BspSpiSoftWriteLine write_sck;
    BspSpiSoftWriteLine write_mosi;
    BspSpiSoftReadLine read_miso;
    BspSpiSoftDelay delay;
    void *line_context;
    uint8_t cpol;
    uint8_t cpha;
} BspSpiSoftAdapter;

void BspSpiSoftAdapter_MakeConfig(BspSpiConfig *config,
                                  BspSpiSoftAdapter *adapter,
                                  BspSpiChipSelectCallback chip_select,
                                  void *chip_select_context,
                                  uint8_t auto_chip_select,
                                  uint32_t default_timeout_ms);

#ifdef __cplusplus
}
#endif

#endif
