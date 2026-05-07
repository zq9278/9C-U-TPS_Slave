#ifndef BSP_SPI_H
#define BSP_SPI_H

#include <stddef.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

enum
{
    /**
     * @brief 默认 SPI 传输超时时间，单位毫秒。
     */
    BSP_SPI_DEFAULT_TIMEOUT_MS = 1000U
};

/**
 * @brief SPI 片选控制回调。
 *
 * 很多 SPI 设备都需要在每次传输前后手动拉低 / 拉高片选，
 * 所以把它作为可选回调放进总线对象里，便于以后复用不同设备。
 */
typedef void (*BspSpiChipSelectCallback)(void *context, uint8_t selected);

/**
 * @brief SPI 底层发送回调。
 */
typedef uint8_t (*BspSpiTransmitCallback)(void *context,
                                          const uint8_t *data,
                                          uint16_t size,
                                          uint32_t timeout_ms);

/**
 * @brief SPI 底层接收回调。
 */
typedef uint8_t (*BspSpiReceiveCallback)(void *context,
                                         uint8_t *data,
                                         uint16_t size,
                                         uint32_t timeout_ms);

/**
 * @brief SPI 底层全双工收发回调。
 */
typedef uint8_t (*BspSpiTransmitReceiveCallback)(void *context,
                                                 const uint8_t *tx_data,
                                                 uint8_t *rx_data,
                                                 uint16_t size,
                                                 uint32_t timeout_ms);

/**
 * @brief SPI 底层适配操作集合。
 *
 * 纯 C 下通过函数指针模拟“对象方法”，让同一套上层代码能复用到：
 * - SPI1
 * - SPI2
 * - 软件 SPI
 * - 带专用片选时序的特殊设备
 */
typedef struct
{
    BspSpiTransmitCallback transmit;
    BspSpiReceiveCallback receive;
    BspSpiTransmitReceiveCallback transmit_receive;
    BspSpiChipSelectCallback chip_select;
} BspSpiOps;

/**
 * @brief SPI 总线初始化配置。
 */
typedef struct
{
    BspSpiOps ops;
    void *low_level_context;
    void *chip_select_context;
    uint32_t default_timeout_ms;
    uint8_t auto_chip_select;
} BspSpiConfig;

/**
 * @brief SPI 总线对象。
 *
 * 一条 SPI 外设或一套软件 SPI 对应一个实例，
 * 上层通过传递 `BspSpi *` 来复用不同总线，效果上接近 C++ 的对象实例。
 */
typedef struct
{
    BspSpiConfig config;
} BspSpi;

void BspSpi_Init(BspSpi *bus, const BspSpiConfig *config);
void BspSpi_SetDefaultTimeout(BspSpi *bus, uint32_t timeout_ms);
void BspSpi_Select(BspSpi *bus);
void BspSpi_Deselect(BspSpi *bus);

uint8_t BspSpi_Transmit(BspSpi *bus,
                        const uint8_t *data,
                        uint16_t size,
                        uint32_t timeout_ms);
uint8_t BspSpi_Receive(BspSpi *bus,
                       uint8_t *data,
                       uint16_t size,
                       uint32_t timeout_ms);
uint8_t BspSpi_TransmitReceive(BspSpi *bus,
                               const uint8_t *tx_data,
                               uint8_t *rx_data,
                               uint16_t size,
                               uint32_t timeout_ms);

#ifdef __cplusplus
}
#endif

#endif
