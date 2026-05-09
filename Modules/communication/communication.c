#include "Modules/communication/communication.h"

#include <string.h>
#include "BSP/Uart/bsp_uart.h"
#include "FreeRTOS.h"
#include "Modules/communication/Protocol/rk3576_protocol.h"
#include "semphr.h"
#include "usart.h"

/* 底层 UART DMA 收发缓冲大小。 */
#define COMM_UART_RX_BUFFER_SIZE 256U
#define COMM_UART_TX_BUFFER_SIZE 160U

/* 单个 UART 接口的资源封装。 */
typedef struct
{
    BspUart port;
    uint8_t rx_dma_buffer[COMM_UART_RX_BUFFER_SIZE];
    uint8_t tx_dma_buffer[COMM_UART_TX_BUFFER_SIZE];
    SemaphoreHandle_t tx_semaphore;
} CommunicationUartInterface;

static CommunicationUartInterface s_log_uart1;
static CommunicationUartInterface s_rk3576_uart3;
static Rk3576Protocol s_rk3576_protocol;
static CommunicationCallbacks s_callbacks;
static uint8_t s_initialized = 0U;

/* 初始化单个 UART 接口并启动 DMA 接收。 */
static void Communication_OnLogUartChunk(void *context, const uint8_t *data, size_t length);
static void Communication_OnRk3576UartChunk(void *context, const uint8_t *data, size_t length);
static void Communication_OnRk3576Frame(void *context, const ProtocolFrameView *frame);
static uint8_t Communication_WriteRk3576Bytes(void *context, const uint8_t *data, size_t length);
static uint8_t Communication_InitUartInterface(CommunicationUartInterface *interface,
                                               UART_HandleTypeDef *huart,
                                               BspUartRxChunkCallback rx_callback);

static uint8_t Communication_InitUartInterface(CommunicationUartInterface *interface,
                                               UART_HandleTypeDef *huart,
                                               BspUartRxChunkCallback rx_callback)
{
    BspUartConfig config;
    BspUartCallbacks callbacks;

    if ((interface == NULL) || (huart == NULL) || (rx_callback == NULL))
    {
        return 0U;
    }

    interface->tx_semaphore = xSemaphoreCreateBinary();
    if (interface->tx_semaphore == NULL)
    {
        return 0U;
    }
    (void)xSemaphoreGive(interface->tx_semaphore);

    (void)memset(&config, 0, sizeof(config));
    config.huart = huart;
    config.rx_dma_buffer = interface->rx_dma_buffer;
    config.rx_dma_buffer_size = sizeof(interface->rx_dma_buffer);
    config.tx_dma_buffer = interface->tx_dma_buffer;
    config.tx_dma_buffer_size = sizeof(interface->tx_dma_buffer);
    config.tx_semaphore = interface->tx_semaphore;
    config.tx_timeout_ms = 50U;
    BspUart_Init(&interface->port, &config);

    (void)memset(&callbacks, 0, sizeof(callbacks));
    callbacks.on_rx_chunk = rx_callback;
    callbacks.context = interface;
    BspUart_SetCallbacks(&interface->port, &callbacks);

    return BspUart_StartReceiveDma(&interface->port);
}

/* UART1 收到新数据块后转交给上层。 */
static void Communication_OnLogUartChunk(void *context, const uint8_t *data, size_t length)
{
    (void)context;
    if (s_callbacks.on_log_rx != NULL)
    {
        s_callbacks.on_log_rx(s_callbacks.context, COMM_CHANNEL_UART1, data, length);
    }
}

/* UART3 收到新业务数据块后交给 RK3576 协议状态机。 */
static void Communication_OnRk3576UartChunk(void *context, const uint8_t *data, size_t length)
{
    (void)context;
    Rk3576Protocol_Input(&s_rk3576_protocol, data, length);
}

/* 收到完整业务帧后通知上层业务逻辑。 */
static void Communication_OnRk3576Frame(void *context, const ProtocolFrameView *frame)
{
    (void)context;
    if (s_callbacks.on_protocol_frame != NULL)
    {
        s_callbacks.on_protocol_frame(s_callbacks.context,
                                      COMM_INTERFACE_RK3576_UART3,
                                      frame);
    }
}

/* RK3576 协议层最终落到 UART3 DMA 发送。 */
static uint8_t Communication_WriteRk3576Bytes(void *context, const uint8_t *data, size_t length)
{
    CommunicationUartInterface *interface = (CommunicationUartInterface *)context;

    if (interface == NULL)
    {
        return 0U;
    }

    return BspUart_Write(&interface->port, data, length);
}

void Communication_Init(void)
{
    if (s_initialized != 0U)
    {
        return;
    }

    (void)memset(&s_log_uart1, 0, sizeof(s_log_uart1));
    (void)memset(&s_rk3576_uart3, 0, sizeof(s_rk3576_uart3));
    (void)memset(&s_callbacks, 0, sizeof(s_callbacks));

    /* UART1 用于日志与 PID 调试。 */
    if (Communication_InitUartInterface(&s_log_uart1,
                                        &huart1,
                                        Communication_OnLogUartChunk) == 0U)
    {
        return;
    }

    /* UART3 用于 RK3576 业务协议。 */
    if (Communication_InitUartInterface(&s_rk3576_uart3,
                                        &huart3,
                                        Communication_OnRk3576UartChunk) == 0U)
    {
        return;
    }

    Rk3576Protocol_Init(&s_rk3576_protocol,
                        Communication_WriteRk3576Bytes,
                        &s_rk3576_uart3);
    Rk3576Protocol_SetFrameCallback(&s_rk3576_protocol,
                                    Communication_OnRk3576Frame,
                                    NULL);

    s_initialized = 1U;
    Communication_WriteLog("[uart1] log ready\r\n");
}

void Communication_SetCallbacks(const CommunicationCallbacks *callbacks)
{
    if (callbacks == NULL)
    {
        (void)memset(&s_callbacks, 0, sizeof(s_callbacks));
        return;
    }

    s_callbacks = *callbacks;
}

void Communication_PollRx(void)
{
    if (s_initialized == 0U)
    {
        return;
    }

    /* 两个 UART 通道都通过统一轮询接口推进接收状态。 */
    BspUart_Poll(&s_log_uart1.port);
    BspUart_Poll(&s_rk3576_uart3.port);
}

uint8_t Communication_SendFrame(CommunicationInterfaceId interface_id,
                                uint16_t frame_id,
                                uint8_t data_type,
                                const void *payload,
                                uint16_t payload_length)
{
    if (s_initialized == 0U)
    {
        return 0U;
    }

    /* 当前只有 RK3576 业务接口支持协议帧发送。 */
    switch (interface_id)
    {
    case COMM_INTERFACE_RK3576_UART3:
        return Rk3576Protocol_SendFrame(&s_rk3576_protocol,
                                        frame_id,
                                        data_type,
                                        (const uint8_t *)payload,
                                        payload_length);

    case COMM_INTERFACE_LOG_UART1:
    default:
        return 0U;
    }
}

uint8_t Communication_SendU8(uint16_t frame_id, uint8_t value)
{
    return Communication_SendFrame(COMM_INTERFACE_RK3576_UART3,
                                   frame_id,
                                   PROTOCOL_DATA_TYPE_UINT8,
                                   &value,
                                   sizeof(value));
}

uint8_t Communication_SendU16(uint16_t frame_id, uint16_t value)
{
    return Communication_SendFrame(COMM_INTERFACE_RK3576_UART3,
                                   frame_id,
                                   PROTOCOL_DATA_TYPE_UINT16,
                                   &value,
                                   sizeof(value));
}

uint8_t Communication_SendU32(uint16_t frame_id, uint32_t value)
{
    return Communication_SendFrame(COMM_INTERFACE_RK3576_UART3,
                                   frame_id,
                                   PROTOCOL_DATA_TYPE_UINT32,
                                   &value,
                                   sizeof(value));
}

uint8_t Communication_SendF32(uint16_t frame_id, float value)
{
    return Communication_SendFrame(COMM_INTERFACE_RK3576_UART3,
                                   frame_id,
                                   PROTOCOL_DATA_TYPE_FLOAT,
                                   &value,
                                   sizeof(value));
}

uint8_t Communication_SendText(uint16_t frame_id, const char *text)
{
    uint16_t length = 0U;

    if (text == NULL)
    {
        return 0U;
    }

    while ((text[length] != '\0') && (length < PROTOCOL_FRAME_MAX_PAYLOAD_LENGTH))
    {
        ++length;
    }

    return Communication_SendFrame(COMM_INTERFACE_RK3576_UART3,
                                   frame_id,
                                   PROTOCOL_DATA_TYPE_TEXT,
                                   text,
                                   length);
}

void Communication_WriteLog(const char *text)
{
    if ((text == NULL) || (s_initialized == 0U))
    {
        return;
    }

    (void)BspUart_WriteString(&s_log_uart1.port, text);
}

int Communication_WriteLogBuffer(const uint8_t *data, size_t length)
{
    if ((data == NULL) || (length == 0U) || (s_initialized == 0U))
    {
        return 0;
    }

    return (BspUart_Write(&s_log_uart1.port, data, length) != 0U) ? (int)length : 0;
}
