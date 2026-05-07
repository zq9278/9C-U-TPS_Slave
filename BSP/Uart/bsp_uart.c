#include "BSP/Uart/bsp_uart.h"

#include <string.h>

#include "FreeRTOS.h"
#include "semphr.h"

enum
{
    BSP_UART_MAX_PORTS = 8
};

static BspUart *g_bsp_uart_registry[BSP_UART_MAX_PORTS];
static size_t g_bsp_uart_registry_count = 0U;

/**
 * @brief 把 UART BSP 对象注册到内部表中。
 *
 * 这样在 HAL 的统一 TX 完成回调里，我们就可以通过 `UART_HandleTypeDef *`
 * 反查到属于哪个 `BspUart` 对象。
 */
static void BspUart_Register(BspUart *uart)
{
    size_t index;

    if (uart == NULL)
    {
        return;
    }

    for (index = 0U; index < g_bsp_uart_registry_count; ++index)
    {
        if (g_bsp_uart_registry[index] == uart)
        {
            return;
        }
    }

    if (g_bsp_uart_registry_count < BSP_UART_MAX_PORTS)
    {
        g_bsp_uart_registry[g_bsp_uart_registry_count++] = uart;
    }
}

/**
 * @brief 读取 RX DMA 当前写入到了环形缓冲区的哪个位置。
 *
 * 这里通过 `NDTR` 推导 DMA 还剩多少字节未搬运，再换算出当前写指针。
 */
static size_t BspUart_GetCurrentWriteIndex(const BspUart *uart)
{
    size_t remaining;

    if ((uart == NULL) ||
        (uart->config.huart == NULL) ||
        (uart->config.huart->hdmarx == NULL) ||
        (uart->config.huart->hdmarx->Instance == NULL) ||
        (uart->config.rx_dma_buffer_size == 0U))
    {
        return 0U;
    }

    remaining = (size_t)uart->config.huart->hdmarx->Instance->CNDTR;
    if (remaining > uart->config.rx_dma_buffer_size)
    {
        remaining = uart->config.rx_dma_buffer_size;
    }

    return uart->config.rx_dma_buffer_size - remaining;
}

/**
 * @brief 把 RX 环形缓冲区中的一段新增字节回调给上层。
 *
 * BSP 只负责告诉上层“新增了哪一段数据”，并不在这里做 ASCII 或协议解析。
 */
static void BspUart_ProcessRxChunk(BspUart *uart, size_t start, size_t length)
{
    if ((uart == NULL) ||
        (uart->callbacks.on_rx_chunk == NULL) ||
        (uart->config.rx_dma_buffer == NULL) ||
        (length == 0U) ||
        (start >= uart->config.rx_dma_buffer_size))
    {
        return;
    }

    if ((start + length) > uart->config.rx_dma_buffer_size)
    {
        length = uart->config.rx_dma_buffer_size - start;
    }

    uart->callbacks.on_rx_chunk(uart->callbacks.context,
                                &uart->config.rx_dma_buffer[start],
                                length);
}

/**
 * @brief 如果 HAL 检测到 UART 接收错误，则尝试自动恢复 DMA 接收。
 *
 * 恢复步骤是：
 * 1. 中止当前接收；
 * 2. 清错误码；
 * 3. 重新启动 RX DMA；
 * 4. 通过回调通知上层。
 */
static void BspUart_RecoverReceiveIfNeeded(BspUart *uart)
{
    if ((uart == NULL) || (uart->config.huart == NULL))
    {
        return;
    }

    if (uart->config.huart->ErrorCode == HAL_UART_ERROR_NONE)
    {
        return;
    }

    (void)HAL_UART_AbortReceive(uart->config.huart);
    uart->config.huart->ErrorCode = HAL_UART_ERROR_NONE;
    (void)BspUart_StartReceiveDma(uart);

    if (uart->callbacks.on_error_recovered != NULL)
    {
        uart->callbacks.on_error_recovered(uart->callbacks.context);
    }
}

/**
 * @brief 初始化 UART BSP 对象。
 */
void BspUart_Init(BspUart *uart, const BspUartConfig *config)
{
    if ((uart == NULL) || (config == NULL))
    {
        return;
    }

    (void)memset(uart, 0, sizeof(*uart));
    uart->config = *config;
    BspUart_Register(uart);
}

/**
 * @brief 设置 UART BSP 回调函数。
 */
void BspUart_SetCallbacks(BspUart *uart, const BspUartCallbacks *callbacks)
{
    if ((uart == NULL) || (callbacks == NULL))
    {
        return;
    }

    uart->callbacks = *callbacks;
}

/**
 * @brief 启动 RX DMA 循环接收。
 */
uint8_t BspUart_StartReceiveDma(BspUart *uart)
{
    if ((uart == NULL) ||
        (uart->config.huart == NULL) ||
        (uart->config.rx_dma_buffer == NULL) ||
        (uart->config.rx_dma_buffer_size == 0U))
    {
        return 0U;
    }

    if (uart->config.huart->hdmarx != NULL)
    {
        if (uart->config.huart->hdmarx->Init.Mode != DMA_CIRCULAR)
        {
            uart->config.huart->hdmarx->Init.Mode = DMA_CIRCULAR;
            (void)HAL_DMA_DeInit(uart->config.huart->hdmarx);
            if (HAL_DMA_Init(uart->config.huart->hdmarx) != HAL_OK)
            {
                return 0U;
            }
        }
    }

    if (HAL_UART_Receive_DMA(uart->config.huart,
                             uart->config.rx_dma_buffer,
                             (uint16_t)uart->config.rx_dma_buffer_size) != HAL_OK)
    {
        return 0U;
    }

    __HAL_DMA_DISABLE_IT(uart->config.huart->hdmarx, DMA_IT_HT);
    uart->last_rx_index = BspUart_GetCurrentWriteIndex(uart);
    return 1U;
}

/**
 * @brief 轮询 DMA 写指针，并把本轮新增字节交给上层。
 *
 * 这就是“DMA Circular RX + 任务轮询”方案的核心处理逻辑。
 */
void BspUart_Poll(BspUart *uart)
{
    size_t new_index;

    if (uart == NULL)
    {
        return;
    }

    BspUart_RecoverReceiveIfNeeded(uart);
    new_index = BspUart_GetCurrentWriteIndex(uart);

    if (new_index < uart->last_rx_index)
    {
        BspUart_ProcessRxChunk(uart,
                               uart->last_rx_index,
                               uart->config.rx_dma_buffer_size - uart->last_rx_index);
        uart->last_rx_index = 0U;
    }

    if (new_index > uart->last_rx_index)
    {
        BspUart_ProcessRxChunk(uart,
                               uart->last_rx_index,
                               new_index - uart->last_rx_index);
        uart->last_rx_index = new_index;
    }
}

/**
 * @brief 通过 TX DMA 发送一段字节流。
 *
 * 同一个 UART 的多次发送通过信号量串行化，避免 DMA 发送未结束时上层又覆盖
 * TX 缓冲区。
 */
uint8_t BspUart_Write(BspUart *uart, const uint8_t *data, size_t length)
{
    SemaphoreHandle_t semaphore;

    if ((uart == NULL) ||
        (uart->config.huart == NULL) ||
        (data == NULL) ||
        (length == 0U) ||
        (uart->config.tx_dma_buffer == NULL) ||
        (uart->config.tx_dma_buffer_size == 0U) ||
        (uart->config.tx_semaphore == NULL))
    {
        return 0U;
    }

    semaphore = (SemaphoreHandle_t)uart->config.tx_semaphore;

    while (length > 0U)
    {
        const size_t chunk = (length > uart->config.tx_dma_buffer_size)
                                 ? uart->config.tx_dma_buffer_size
                                 : length;

        if (xSemaphoreTake(semaphore, pdMS_TO_TICKS(uart->config.tx_timeout_ms)) != pdTRUE)
        {
            return 0U;
        }

        (void)memcpy(uart->config.tx_dma_buffer, data, chunk);
        uart->tx_in_progress = 1U;

        if (HAL_UART_Transmit_DMA(uart->config.huart,
                                  uart->config.tx_dma_buffer,
                                  (uint16_t)chunk) != HAL_OK)
        {
            uart->tx_in_progress = 0U;
            (void)xSemaphoreGive(semaphore);
            return 0U;
        }

        data += chunk;
        length -= chunk;
    }

    return 1U;
}

/**
 * @brief 发送一个以 `\0` 结尾的字符串。
 */
uint8_t BspUart_WriteString(BspUart *uart, const char *text)
{
    size_t length = 0U;

    if (text == NULL)
    {
        return 0U;
    }

    while (text[length] != '\0')
    {
        ++length;
    }

    return BspUart_Write(uart, (const uint8_t *)text, length);
}

/**
 * @brief 在 TX DMA 完成中断里释放发送权并通知上层。
 */
void BspUart_OnTxCompleteFromIsr(BspUart *uart)
{
    BaseType_t higher_priority_task_woken = pdFALSE;
    SemaphoreHandle_t semaphore;

    if ((uart == NULL) || (uart->config.tx_semaphore == NULL))
    {
        return;
    }

    semaphore = (SemaphoreHandle_t)uart->config.tx_semaphore;
    uart->tx_in_progress = 0U;
    (void)xSemaphoreGiveFromISR(semaphore, &higher_priority_task_woken);

    if (uart->callbacks.on_tx_complete != NULL)
    {
        uart->callbacks.on_tx_complete(uart->callbacks.context);
    }

    portYIELD_FROM_ISR(higher_priority_task_woken);
}

/**
 * @brief 判断 HAL UART 实例是否属于当前 BSP UART 对象。
 */
uint8_t BspUart_MatchInstance(const BspUart *uart, const void *instance)
{
    if ((uart == NULL) || (uart->config.huart == NULL))
    {
        return 0U;
    }

    return (uint8_t)(uart->config.huart == instance);
}

/**
 * @brief HAL 层统一的 UART TX DMA 完成回调。
 *
 * 这里通过内部注册表查找对应的 `BspUart` 对象，再把中断事件转交给 BSP 层处理。
 */
void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
{
    size_t index;

    for (index = 0U; index < g_bsp_uart_registry_count; ++index)
    {
        if (BspUart_MatchInstance(g_bsp_uart_registry[index], huart) != 0U)
        {
            BspUart_OnTxCompleteFromIsr(g_bsp_uart_registry[index]);
            break;
        }
    }
}
