#ifndef BSP_I2C_H
#define BSP_I2C_H

#include <stddef.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

enum
{
    /**
     * @brief 默认 I2C 读写超时时间，单位毫秒。
     */
    BSP_I2C_DEFAULT_TIMEOUT_MS = 1000U
};

/**
 * @brief 底层原始读函数类型。
 *
 * 这层抽象的意义是：BSP/I2c 不直接绑定 HAL，也不依赖某个具体 I2C 外设句柄。
 * 以后你可以给 I2C1、I2C2、软件 I2C、甚至某些桥接总线都接上同一套上层寄存器访问逻辑。
 */
typedef uint8_t (*BspI2cReadMemCallback)(void *context,
                                         uint8_t dev_addr,
                                         uint8_t reg_addr,
                                         uint8_t *data,
                                         uint16_t size,
                                         uint32_t timeout_ms);

/**
 * @brief 底层原始写函数类型。
 */
typedef uint8_t (*BspI2cWriteMemCallback)(void *context,
                                          uint8_t dev_addr,
                                          uint8_t reg_addr,
                                          const uint8_t *data,
                                          uint16_t size,
                                          uint32_t timeout_ms);
typedef uint8_t (*BspI2cMasterTransmitCallback)(void *context,
                                                uint8_t dev_addr,
                                                const uint8_t *data,
                                                uint16_t size,
                                                uint32_t timeout_ms);
typedef uint8_t (*BspI2cMasterReceiveCallback)(void *context,
                                               uint8_t dev_addr,
                                               uint8_t *data,
                                               uint16_t size,
                                               uint32_t timeout_ms);
typedef void (*BspI2cRecoverCallback)(void *context);

/**
 * @brief I2C 底层适配操作集合。
 *
 * 上层寄存器读写函数只依赖这里的两个回调：
 * 1. 读一段寄存器数据；
 * 2. 写一段寄存器数据。
 */
typedef struct
{
    BspI2cReadMemCallback read_mem;
    BspI2cWriteMemCallback write_mem;
    BspI2cMasterTransmitCallback master_transmit;
    BspI2cMasterReceiveCallback master_receive;
    BspI2cRecoverCallback recover;
} BspI2cOps;

/**
 * @brief I2C 总线初始化配置。
 *
 * `low_level_context` 可以指向：
 * - HAL 的 `I2C_HandleTypeDef`
 * - 你自己封装的 I2C 设备上下文
 * - 软件 I2C 的状态结构体
 */
typedef struct
{
    BspI2cOps ops;
    void *low_level_context;
    uint32_t default_timeout_ms;
} BspI2cConfig;

/**
 * @brief I2C 总线对象。
 *
 * 这就是纯 C 下“类似面向对象”的核心做法：
 * 每一条总线各有自己的配置、超时和底层上下文，
 * 上层驱动只要持有一个 `BspI2c *`，就能像操作一个对象一样复用多路 I2C。
 */
typedef struct
{
    BspI2cConfig config;
} BspI2c;

void BspI2c_Init(BspI2c *bus, const BspI2cConfig *config);
void BspI2c_SetDefaultTimeout(BspI2c *bus, uint32_t timeout_ms);

uint8_t BspI2c_ReadBit(BspI2c *bus,
                       uint8_t dev_addr,
                       uint8_t reg_addr,
                       uint8_t bit_num,
                       uint8_t *data,
                       uint32_t timeout_ms);
uint8_t BspI2c_ReadBitW(BspI2c *bus,
                        uint8_t dev_addr,
                        uint8_t reg_addr,
                        uint8_t bit_num,
                        uint16_t *data,
                        uint32_t timeout_ms);
uint8_t BspI2c_ReadBits(BspI2c *bus,
                        uint8_t dev_addr,
                        uint8_t reg_addr,
                        uint8_t bit_start,
                        uint8_t length,
                        uint8_t *data,
                        uint32_t timeout_ms);
uint8_t BspI2c_ReadBitsW(BspI2c *bus,
                         uint8_t dev_addr,
                         uint8_t reg_addr,
                         uint8_t bit_start,
                         uint8_t length,
                         uint16_t *data,
                         uint32_t timeout_ms);
uint8_t BspI2c_ReadByte(BspI2c *bus,
                        uint8_t dev_addr,
                        uint8_t reg_addr,
                        uint8_t *data,
                        uint32_t timeout_ms);
uint8_t BspI2c_ReadWord(BspI2c *bus,
                        uint8_t dev_addr,
                        uint8_t reg_addr,
                        uint16_t *data,
                        uint32_t timeout_ms);
uint8_t BspI2c_ReadBytes(BspI2c *bus,
                         uint8_t dev_addr,
                         uint8_t reg_addr,
                         uint8_t length,
                         uint8_t *data,
                         uint32_t timeout_ms);
uint8_t BspI2c_ReadWords(BspI2c *bus,
                         uint8_t dev_addr,
                         uint8_t reg_addr,
                         uint8_t length,
                         uint16_t *data,
                         uint32_t timeout_ms);

uint8_t BspI2c_WriteBit(BspI2c *bus,
                        uint8_t dev_addr,
                        uint8_t reg_addr,
                        uint8_t bit_num,
                        uint8_t data);
uint8_t BspI2c_WriteBitW(BspI2c *bus,
                         uint8_t dev_addr,
                         uint8_t reg_addr,
                         uint8_t bit_num,
                         uint16_t data);
uint8_t BspI2c_WriteBits(BspI2c *bus,
                         uint8_t dev_addr,
                         uint8_t reg_addr,
                         uint8_t bit_start,
                         uint8_t length,
                         uint8_t data);
uint8_t BspI2c_WriteBitsW(BspI2c *bus,
                          uint8_t dev_addr,
                          uint8_t reg_addr,
                          uint8_t bit_start,
                          uint8_t length,
                          uint16_t data);
uint8_t BspI2c_WriteByte(BspI2c *bus,
                         uint8_t dev_addr,
                         uint8_t reg_addr,
                         uint8_t data);
uint8_t BspI2c_WriteWord(BspI2c *bus,
                         uint8_t dev_addr,
                         uint8_t reg_addr,
                         uint16_t data);
uint8_t BspI2c_WriteBytes(BspI2c *bus,
                          uint8_t dev_addr,
                          uint8_t reg_addr,
                          uint8_t length,
                          const uint8_t *data);
uint8_t BspI2c_WriteWords(BspI2c *bus,
                          uint8_t dev_addr,
                          uint8_t reg_addr,
                          uint8_t length,
                          const uint16_t *data);
uint8_t BspI2c_MasterTransmit(BspI2c *bus,
                              uint8_t dev_addr,
                              const uint8_t *data,
                              uint16_t size,
                              uint32_t timeout_ms);
uint8_t BspI2c_MasterReceive(BspI2c *bus,
                             uint8_t dev_addr,
                             uint8_t *data,
                             uint16_t size,
                             uint32_t timeout_ms);
void BspI2c_Recover(BspI2c *bus);

#ifdef __cplusplus
}
#endif

#endif
