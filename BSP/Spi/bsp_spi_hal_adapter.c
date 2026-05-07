#include "bsp_spi_hal_adapter.h"

#include <stddef.h>
#include <string.h>

static uint8_t SpiHal_TransmitBlocking(void *context,
                                       const uint8_t *data,
                                       uint16_t size,
                                       uint32_t timeout_ms)
{
#ifdef HAL_SPI_MODULE_ENABLED
    SPI_HandleTypeDef *hspi = (SPI_HandleTypeDef *)context;
    return (uint8_t)((hspi != NULL) &&
                     (HAL_SPI_Transmit(hspi, (uint8_t *)data, size, timeout_ms) == HAL_OK));
#else
    (void)context; (void)data; (void)size; (void)timeout_ms;
    return 0U;
#endif
}

static uint8_t SpiHal_ReceiveBlocking(void *context,
                                      uint8_t *data,
                                      uint16_t size,
                                      uint32_t timeout_ms)
{
#ifdef HAL_SPI_MODULE_ENABLED
    SPI_HandleTypeDef *hspi = (SPI_HandleTypeDef *)context;
    return (uint8_t)((hspi != NULL) &&
                     (HAL_SPI_Receive(hspi, data, size, timeout_ms) == HAL_OK));
#else
    (void)context; (void)data; (void)size; (void)timeout_ms;
    return 0U;
#endif
}

static uint8_t SpiHal_TransmitReceiveBlocking(void *context,
                                              const uint8_t *tx_data,
                                              uint8_t *rx_data,
                                              uint16_t size,
                                              uint32_t timeout_ms)
{
#ifdef HAL_SPI_MODULE_ENABLED
    SPI_HandleTypeDef *hspi = (SPI_HandleTypeDef *)context;
    return (uint8_t)((hspi != NULL) &&
                     (HAL_SPI_TransmitReceive(hspi, (uint8_t *)tx_data, rx_data, size, timeout_ms) == HAL_OK));
#else
    (void)context; (void)tx_data; (void)rx_data; (void)size; (void)timeout_ms;
    return 0U;
#endif
}

static uint8_t SpiHal_TransmitDma(void *context,
                                  const uint8_t *data,
                                  uint16_t size,
                                  uint32_t timeout_ms)
{
#ifdef HAL_SPI_MODULE_ENABLED
    SPI_HandleTypeDef *hspi = (SPI_HandleTypeDef *)context;
    (void)timeout_ms;
    return (uint8_t)((hspi != NULL) && (HAL_SPI_Transmit_DMA(hspi, (uint8_t *)data, size) == HAL_OK));
#else
    (void)context; (void)data; (void)size; (void)timeout_ms;
    return 0U;
#endif
}

static uint8_t SpiHal_ReceiveDma(void *context,
                                 uint8_t *data,
                                 uint16_t size,
                                 uint32_t timeout_ms)
{
#ifdef HAL_SPI_MODULE_ENABLED
    SPI_HandleTypeDef *hspi = (SPI_HandleTypeDef *)context;
    (void)timeout_ms;
    return (uint8_t)((hspi != NULL) && (HAL_SPI_Receive_DMA(hspi, data, size) == HAL_OK));
#else
    (void)context; (void)data; (void)size; (void)timeout_ms;
    return 0U;
#endif
}

static uint8_t SpiHal_TransmitReceiveDma(void *context,
                                         const uint8_t *tx_data,
                                         uint8_t *rx_data,
                                         uint16_t size,
                                         uint32_t timeout_ms)
{
#ifdef HAL_SPI_MODULE_ENABLED
    SPI_HandleTypeDef *hspi = (SPI_HandleTypeDef *)context;
    (void)timeout_ms;
    return (uint8_t)((hspi != NULL) &&
                     (HAL_SPI_TransmitReceive_DMA(hspi, (uint8_t *)tx_data, rx_data, size) == HAL_OK));
#else
    (void)context; (void)tx_data; (void)rx_data; (void)size; (void)timeout_ms;
    return 0U;
#endif
}

static uint8_t SpiHal_TransmitIt(void *context,
                                 const uint8_t *data,
                                 uint16_t size,
                                 uint32_t timeout_ms)
{
#ifdef HAL_SPI_MODULE_ENABLED
    SPI_HandleTypeDef *hspi = (SPI_HandleTypeDef *)context;
    (void)timeout_ms;
    return (uint8_t)((hspi != NULL) && (HAL_SPI_Transmit_IT(hspi, (uint8_t *)data, size) == HAL_OK));
#else
    (void)context; (void)data; (void)size; (void)timeout_ms;
    return 0U;
#endif
}

static uint8_t SpiHal_ReceiveIt(void *context,
                                uint8_t *data,
                                uint16_t size,
                                uint32_t timeout_ms)
{
#ifdef HAL_SPI_MODULE_ENABLED
    SPI_HandleTypeDef *hspi = (SPI_HandleTypeDef *)context;
    (void)timeout_ms;
    return (uint8_t)((hspi != NULL) && (HAL_SPI_Receive_IT(hspi, data, size) == HAL_OK));
#else
    (void)context; (void)data; (void)size; (void)timeout_ms;
    return 0U;
#endif
}

static uint8_t SpiHal_TransmitReceiveIt(void *context,
                                        const uint8_t *tx_data,
                                        uint8_t *rx_data,
                                        uint16_t size,
                                        uint32_t timeout_ms)
{
#ifdef HAL_SPI_MODULE_ENABLED
    SPI_HandleTypeDef *hspi = (SPI_HandleTypeDef *)context;
    (void)timeout_ms;
    return (uint8_t)((hspi != NULL) &&
                     (HAL_SPI_TransmitReceive_IT(hspi, (uint8_t *)tx_data, rx_data, size) == HAL_OK));
#else
    (void)context; (void)tx_data; (void)rx_data; (void)size; (void)timeout_ms;
    return 0U;
#endif
}

void BspSpiHalAdapter_MakeConfig(BspSpiConfig *config,
                                 BspSpiHalHandle *hspi,
                                 BspSpiHalMode mode,
                                 BspSpiChipSelectCallback chip_select,
                                 void *chip_select_context,
                                 uint8_t auto_chip_select,
                                 uint32_t default_timeout_ms)
{
    if (config == NULL)
    {
        return;
    }

    (void)memset(config, 0, sizeof(*config));
    config->low_level_context = hspi;
    config->chip_select_context = chip_select_context;
    config->ops.chip_select = chip_select;
    config->auto_chip_select = auto_chip_select;
    config->default_timeout_ms = default_timeout_ms;

    switch (mode)
    {
    case BSP_SPI_HAL_MODE_DMA:
        config->ops.transmit = SpiHal_TransmitDma;
        config->ops.receive = SpiHal_ReceiveDma;
        config->ops.transmit_receive = SpiHal_TransmitReceiveDma;
        break;
    case BSP_SPI_HAL_MODE_IT:
        config->ops.transmit = SpiHal_TransmitIt;
        config->ops.receive = SpiHal_ReceiveIt;
        config->ops.transmit_receive = SpiHal_TransmitReceiveIt;
        break;
    case BSP_SPI_HAL_MODE_BLOCKING:
    default:
        config->ops.transmit = SpiHal_TransmitBlocking;
        config->ops.receive = SpiHal_ReceiveBlocking;
        config->ops.transmit_receive = SpiHal_TransmitReceiveBlocking;
        break;
    }
}
