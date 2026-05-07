#ifndef BSP_UART_H
#define BSP_UART_H

#include <stddef.h>
#include <stdint.h>

#include "usart.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief 串口接收新增字节块时的回调类型。
 */
typedef void (*BspUartRxChunkCallback)(void *context, const uint8_t *data, size_t length);

/**
 * @brief 不带参数的简单事件回调类型。
 */
typedef void (*BspUartSimpleCallback)(void *context);

/**
 * @brief UART BSP 对外暴露的回调集合。
 */
typedef struct
{
    BspUartRxChunkCallback on_rx_chunk;
    BspUartSimpleCallback on_error_recovered;
    BspUartSimpleCallback on_tx_complete;
    void *context;
} BspUartCallbacks;

/**
 * @brief UART BSP 初始化配置。
 */
typedef struct
{
    UART_HandleTypeDef *huart;
    uint8_t *rx_dma_buffer;
    size_t rx_dma_buffer_size;
    uint8_t *tx_dma_buffer;
    size_t tx_dma_buffer_size;
    void *tx_semaphore;
    uint32_t tx_timeout_ms;
} BspUartConfig;

/**
 * @brief UART BSP 运行时对象。
 */
typedef struct
{
    BspUartConfig config;
    BspUartCallbacks callbacks;
    size_t last_rx_index;
    uint8_t tx_in_progress;
} BspUart;

/**
 * @brief 初始化一个 UART BSP 对象。
 *
 * 这个函数会保存底层配置、清空运行时状态，并把该对象注册到内部索引表中，
 * 方便后续在 HAL 的统一 TX 完成回调里反查到对应的 BSP UART。
 *
 * @param uart   待初始化的 UART BSP 对象。
 * @param config 初始化配置。
 */
void BspUart_Init(BspUart *uart, const BspUartConfig *config);

/**
 * @brief 为 UART BSP 对象设置回调函数。
 *
 * @param uart      目标 UART BSP 对象。
 * @param callbacks 回调集合。
 */
void BspUart_SetCallbacks(BspUart *uart, const BspUartCallbacks *callbacks);

/**
 * @brief 启动 RX DMA 循环接收。
 *
 * @param uart 目标 UART BSP 对象。
 * @return `1U` 表示启动成功，`0U` 表示参数错误或 HAL 调用失败。
 */
uint8_t BspUart_StartReceiveDma(BspUart *uart);

/**
 * @brief 轮询 DMA 当前写指针，并把新增字节块回调给上层。
 *
 * @param uart 目标 UART BSP 对象。
 */
void BspUart_Poll(BspUart *uart);

/**
 * @brief 通过 DMA 发送一段原始字节流。
 *
 * @param uart   目标 UART BSP 对象。
 * @param data   待发送数据首地址。
 * @param length 待发送字节数。
 * @return `1U` 表示发送流程成功启动，`0U` 表示参数错误、超时或 HAL 调用失败。
 */
uint8_t BspUart_Write(BspUart *uart, const uint8_t *data, size_t length);

/**
 * @brief 发送一个以 `\0` 结尾的字符串。
 *
 * @param uart 目标 UART BSP 对象。
 * @param text 待发送字符串。
 * @return `1U` 表示发送流程成功启动，`0U` 表示发送失败。
 */
uint8_t BspUart_WriteString(BspUart *uart, const char *text);

/**
 * @brief 在 TX DMA 完成中断里通知 UART BSP 当前一次发送结束。
 *
 * @param uart 目标 UART BSP 对象。
 */
void BspUart_OnTxCompleteFromIsr(BspUart *uart);

/**
 * @brief 判断 HAL UART 实例是否和当前 BSP UART 对象匹配。
 *
 * @param uart     目标 UART BSP 对象。
 * @param instance HAL 回调里传上的 UART 实例指针。
 * @return `1U` 表示匹配成功，`0U` 表示不匹配。
 */
uint8_t BspUart_MatchInstance(const BspUart *uart, const void *instance);

#ifdef __cplusplus
}
#endif

#endif
