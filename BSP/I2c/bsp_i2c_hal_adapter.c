#include "bsp_i2c_hal_adapter.h"

#include <stddef.h>
#include <string.h>

#ifdef HAL_I2C_MODULE_ENABLED
static uint16_t BspI2cHal_DevAddr(uint8_t dev_addr)
{
    return (uint16_t)((uint16_t)dev_addr << 1U);
}
#endif

static uint8_t BspI2cHal_ReadBlocking(void *context,
                                      uint8_t dev_addr,
                                      uint8_t reg_addr,
                                      uint8_t *data,
                                      uint16_t size,
                                      uint32_t timeout_ms)
{
#ifdef HAL_I2C_MODULE_ENABLED
    I2C_HandleTypeDef *hi2c = (I2C_HandleTypeDef *)context;
    return (uint8_t)((hi2c != NULL) &&
                     (HAL_I2C_Mem_Read(hi2c,
                                       BspI2cHal_DevAddr(dev_addr),
                                       reg_addr,
                                       I2C_MEMADD_SIZE_8BIT,
                                       data,
                                       size,
                                       timeout_ms) == HAL_OK));
#else
    (void)context; (void)dev_addr; (void)reg_addr; (void)data; (void)size; (void)timeout_ms;
    return 0U;
#endif
}

static uint8_t BspI2cHal_WriteBlocking(void *context,
                                       uint8_t dev_addr,
                                       uint8_t reg_addr,
                                       const uint8_t *data,
                                       uint16_t size,
                                       uint32_t timeout_ms)
{
#ifdef HAL_I2C_MODULE_ENABLED
    I2C_HandleTypeDef *hi2c = (I2C_HandleTypeDef *)context;
    return (uint8_t)((hi2c != NULL) &&
                     (HAL_I2C_Mem_Write(hi2c,
                                        BspI2cHal_DevAddr(dev_addr),
                                        reg_addr,
                                        I2C_MEMADD_SIZE_8BIT,
                                        (uint8_t *)data,
                                        size,
                                        timeout_ms) == HAL_OK));
#else
    (void)context; (void)dev_addr; (void)reg_addr; (void)data; (void)size; (void)timeout_ms;
    return 0U;
#endif
}

static uint8_t BspI2cHal_MasterTransmitBlocking(void *context,
                                                uint8_t dev_addr,
                                                const uint8_t *data,
                                                uint16_t size,
                                                uint32_t timeout_ms)
{
#ifdef HAL_I2C_MODULE_ENABLED
    I2C_HandleTypeDef *hi2c = (I2C_HandleTypeDef *)context;
    return (uint8_t)((hi2c != NULL) &&
                     (HAL_I2C_Master_Transmit(hi2c,
                                              BspI2cHal_DevAddr(dev_addr),
                                              (uint8_t *)data,
                                              size,
                                              timeout_ms) == HAL_OK));
#else
    (void)context; (void)dev_addr; (void)data; (void)size; (void)timeout_ms;
    return 0U;
#endif
}

static uint8_t BspI2cHal_MasterReceiveBlocking(void *context,
                                               uint8_t dev_addr,
                                               uint8_t *data,
                                               uint16_t size,
                                               uint32_t timeout_ms)
{
#ifdef HAL_I2C_MODULE_ENABLED
    I2C_HandleTypeDef *hi2c = (I2C_HandleTypeDef *)context;
    return (uint8_t)((hi2c != NULL) &&
                     (HAL_I2C_Master_Receive(hi2c,
                                             BspI2cHal_DevAddr(dev_addr),
                                             data,
                                             size,
                                             timeout_ms) == HAL_OK));
#else
    (void)context; (void)dev_addr; (void)data; (void)size; (void)timeout_ms;
    return 0U;
#endif
}

static void BspI2cHal_Recover(void *context)
{
#ifdef HAL_I2C_MODULE_ENABLED
    I2C_HandleTypeDef *hi2c = (I2C_HandleTypeDef *)context;

    if (hi2c == NULL)
    {
        return;
    }

    __HAL_I2C_DISABLE_IT(hi2c, I2C_IT_ERRI);
    __HAL_I2C_DISABLE(hi2c);
    __HAL_I2C_CLEAR_FLAG(hi2c, I2C_FLAG_BERR | I2C_FLAG_ARLO | I2C_FLAG_OVR | I2C_FLAG_TIMEOUT);

    (void)HAL_I2C_DeInit(hi2c);
    (void)HAL_I2C_Init(hi2c);
    (void)HAL_I2CEx_ConfigAnalogFilter(hi2c, I2C_ANALOGFILTER_ENABLE);
    (void)HAL_I2CEx_ConfigDigitalFilter(hi2c, 0U);
    __HAL_I2C_ENABLE_IT(hi2c, I2C_IT_ERRI);
#else
    (void)context;
#endif
}

static uint8_t BspI2cHal_ReadDma(void *context,
                                 uint8_t dev_addr,
                                 uint8_t reg_addr,
                                 uint8_t *data,
                                 uint16_t size,
                                 uint32_t timeout_ms)
{
#ifdef HAL_I2C_MODULE_ENABLED
    I2C_HandleTypeDef *hi2c = (I2C_HandleTypeDef *)context;
    (void)timeout_ms;
    return (uint8_t)((hi2c != NULL) &&
                     (HAL_I2C_Mem_Read_DMA(hi2c,
                                           BspI2cHal_DevAddr(dev_addr),
                                           reg_addr,
                                           I2C_MEMADD_SIZE_8BIT,
                                           data,
                                           size) == HAL_OK));
#else
    (void)context; (void)dev_addr; (void)reg_addr; (void)data; (void)size; (void)timeout_ms;
    return 0U;
#endif
}

static uint8_t BspI2cHal_WriteDma(void *context,
                                  uint8_t dev_addr,
                                  uint8_t reg_addr,
                                  const uint8_t *data,
                                  uint16_t size,
                                  uint32_t timeout_ms)
{
#ifdef HAL_I2C_MODULE_ENABLED
    I2C_HandleTypeDef *hi2c = (I2C_HandleTypeDef *)context;
    (void)timeout_ms;
    return (uint8_t)((hi2c != NULL) &&
                     (HAL_I2C_Mem_Write_DMA(hi2c,
                                            BspI2cHal_DevAddr(dev_addr),
                                            reg_addr,
                                            I2C_MEMADD_SIZE_8BIT,
                                            (uint8_t *)data,
                                            size) == HAL_OK));
#else
    (void)context; (void)dev_addr; (void)reg_addr; (void)data; (void)size; (void)timeout_ms;
    return 0U;
#endif
}

static uint8_t BspI2cHal_ReadIt(void *context,
                                uint8_t dev_addr,
                                uint8_t reg_addr,
                                uint8_t *data,
                                uint16_t size,
                                uint32_t timeout_ms)
{
#ifdef HAL_I2C_MODULE_ENABLED
    I2C_HandleTypeDef *hi2c = (I2C_HandleTypeDef *)context;
    (void)timeout_ms;
    return (uint8_t)((hi2c != NULL) &&
                     (HAL_I2C_Mem_Read_IT(hi2c,
                                          BspI2cHal_DevAddr(dev_addr),
                                          reg_addr,
                                          I2C_MEMADD_SIZE_8BIT,
                                          data,
                                          size) == HAL_OK));
#else
    (void)context; (void)dev_addr; (void)reg_addr; (void)data; (void)size; (void)timeout_ms;
    return 0U;
#endif
}

static uint8_t BspI2cHal_WriteIt(void *context,
                                 uint8_t dev_addr,
                                 uint8_t reg_addr,
                                 const uint8_t *data,
                                 uint16_t size,
                                 uint32_t timeout_ms)
{
#ifdef HAL_I2C_MODULE_ENABLED
    I2C_HandleTypeDef *hi2c = (I2C_HandleTypeDef *)context;
    (void)timeout_ms;
    return (uint8_t)((hi2c != NULL) &&
                     (HAL_I2C_Mem_Write_IT(hi2c,
                                           BspI2cHal_DevAddr(dev_addr),
                                           reg_addr,
                                           I2C_MEMADD_SIZE_8BIT,
                                           (uint8_t *)data,
                                           size) == HAL_OK));
#else
    (void)context; (void)dev_addr; (void)reg_addr; (void)data; (void)size; (void)timeout_ms;
    return 0U;
#endif
}

void BspI2cHalAdapter_MakeConfig(BspI2cConfig *config,
                                 BspI2cHalHandle *hi2c,
                                 BspI2cHalMode mode,
                                 uint32_t default_timeout_ms)
{
    if (config == NULL)
    {
        return;
    }

    (void)memset(config, 0, sizeof(*config));
    config->low_level_context = hi2c;
    config->default_timeout_ms = default_timeout_ms;
    config->ops.master_transmit = BspI2cHal_MasterTransmitBlocking;
    config->ops.master_receive = BspI2cHal_MasterReceiveBlocking;
    config->ops.recover = BspI2cHal_Recover;

    switch (mode)
    {
    case BSP_I2C_HAL_MODE_DMA:
        config->ops.read_mem = BspI2cHal_ReadDma;
        config->ops.write_mem = BspI2cHal_WriteDma;
        break;
    case BSP_I2C_HAL_MODE_IT:
        config->ops.read_mem = BspI2cHal_ReadIt;
        config->ops.write_mem = BspI2cHal_WriteIt;
        break;
    case BSP_I2C_HAL_MODE_BLOCKING:
    default:
        config->ops.read_mem = BspI2cHal_ReadBlocking;
        config->ops.write_mem = BspI2cHal_WriteBlocking;
        break;
    }
}
