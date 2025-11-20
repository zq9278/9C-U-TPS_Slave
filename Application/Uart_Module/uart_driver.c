/*
 * UART driver + protocol framing (AA55 | id | type | len | data | crc | 0D0A)
 */

#include <string.h>
#include <stdbool.h>
#include "uart_driver.h"
#include "Uart_Communicate.h"

// External UART handles
extern UART_HandleTypeDef huart1; // debug
extern UART_HandleTypeDef huart3; // business

// Ports
UartPort_t rk3576_uart_port = {
    .huart = &huart3,
    .name  = "UART3",
};
UartPort_t debug_uart_port = {
    .huart = &huart1,
    .name  = "UART1",
};

// DMA RX buffers
uint8_t uart1_dma_rx_buf[UART_RX_DMA_BUFFER_SIZE];
uint8_t uart2_dma_rx_buf[UART_RX_DMA_BUFFER_SIZE];
// DMA TX buffer (single in-flight frame, protected by uartTxDoneSem)
static uint8_t uart2_dma_tx_buf[UART_TX_MSG_MAX_LEN];

void rk3576_uart_port_Init(UartPort_t *port)
{
    port->tx_queue = xQueueCreate(UART_TX_QUEUE_LENGTH, sizeof(UartTxMessage_t));
    port->rx_queue = xQueueCreate(UART_RX_QUEUE_SIZE, sizeof(UartRxMessage_t));
    configASSERT(port->tx_queue != NULL);
    configASSERT(port->rx_queue != NULL);
    port->uartTxDoneSem = xSemaphoreCreateBinary();
    xSemaphoreGive(port->uartTxDoneSem);

    port->dma_rx_buf = uart2_dma_rx_buf;
    port->parser = parse_rk3576_uart_port_stream;
    port->sender = send_rk3576_uart_port_frame;
    port->crc    = crc16_modbus;

    if (port->huart->hdmarx != NULL) {
        HAL_UARTEx_ReceiveToIdle_DMA(port->huart, port->dma_rx_buf, UART_RX_DMA_BUFFER_SIZE);
        __HAL_UART_ENABLE_IT(port->huart, UART_IT_IDLE);
    } else {
        HAL_UARTEx_ReceiveToIdle_IT(port->huart, port->dma_rx_buf, UART_RX_DMA_BUFFER_SIZE);
    }
}

void debug_uart_port_Init(UartPort_t *port)
{
    port->tx_queue = xQueueCreate(LOG_QUEUE_LEN, sizeof(LogMessage_t));
    port->rx_queue = xQueueCreate(UART_RX_QUEUE_SIZE, sizeof(AsciiCmdMessage_t));
    port->dma_rx_buf = uart1_dma_rx_buf;
    port->parser = parse_debug_uart_port_stream;
    port->crc    = crc16_modbus;

    if (port->huart->hdmarx != NULL) {
        HAL_UARTEx_ReceiveToIdle_DMA(port->huart, port->dma_rx_buf, UART_RX_DMA_BUFFER_SIZE);
        __HAL_UART_ENABLE_IT(port->huart, UART_IT_IDLE);
    } else {
        HAL_UARTEx_ReceiveToIdle_IT(port->huart, port->dma_rx_buf, UART_RX_DMA_BUFFER_SIZE);
    }
}

void rk3576_uart_port_RxCallback(UartPort_t *port, uint16_t size)
{
    __HAL_UART_CLEAR_IDLEFLAG(port->huart);
    UartRxMessage_t msg;
    memcpy(msg.data, port->dma_rx_buf, size);
    msg.length = size;
    HAL_UARTEx_ReceiveToIdle_DMA(port->huart, port->dma_rx_buf, UART_RX_DMA_BUFFER_SIZE);
    BaseType_t hpw = pdFALSE;
    xQueueSendFromISR(port->rx_queue, &msg, &hpw);
    portYIELD_FROM_ISR(hpw);
}

void debug_uart_port_RxCallback(UartPort_t *port, uint16_t size)
{
    __HAL_UART_CLEAR_IDLEFLAG(port->huart);
    UartRxMessage_t msg;
    memcpy(msg.data, port->dma_rx_buf, size);
    msg.length = size;
    HAL_UARTEx_ReceiveToIdle_DMA(port->huart, port->dma_rx_buf, UART_RX_DMA_BUFFER_SIZE);
    BaseType_t hpw = pdFALSE;
    xQueueSendFromISR(port->rx_queue, &msg, &hpw);
    portYIELD_FROM_ISR(hpw);
}

void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size)
{
    if (huart == rk3576_uart_port.huart) rk3576_uart_port_RxCallback(&rk3576_uart_port, Size);
    if (huart == debug_uart_port.huart)  debug_uart_port_RxCallback(&debug_uart_port, Size);
}

bool send_rk3576_uart_port_frame(DataType_t type, uint16_t frame_id, const void *data)
{
    uint16_t data_len;
    switch (type) {
        case DATA_FLOAT:    data_len = sizeof(float); break;
        case DATA_UINT8_T:  data_len = sizeof(uint8_t); break;
        case DATA_UINT16_T: data_len = sizeof(uint16_t); break;
        case DATA_UINT32_T: data_len = sizeof(uint32_t); break;
        case DATA_TYPE_TEXT: {
            data_len = (uint16_t)strlen((const char*)data);
            if (data_len > FRAME_MAX_DATA_LEN) data_len = FRAME_MAX_DATA_LEN;
            break;
        }
        default: return false;
    }
    if (data_len > FRAME_MAX_DATA_LEN) return false;

    // Serialize access to UART TX DMA using the same semaphore used by SerialTxTask
    if (rk3576_uart_port.uartTxDoneSem == NULL) return false;
    if (xSemaphoreTake(rk3576_uart_port.uartTxDoneSem, pdMS_TO_TICKS(200)) != pdTRUE) {
        // Busy or timeout
        return false;
    }

    // Build frame into persistent DMA buffer
    memset(uart2_dma_tx_buf, 0, sizeof(uart2_dma_tx_buf));
    uint8_t *p = uart2_dma_tx_buf;

    *p++ = FRAME_HEADER_1;
    *p++ = FRAME_HEADER_2;
    memcpy(p, &frame_id, 2); p += 2;
    *p++ = (uint8_t)type;
    memcpy(p, &data_len, 2); p += 2;

    if (type == DATA_UINT16_T) {
        uint16_t v; memcpy(&v, data, 2);
        *p++ = (uint8_t)(v & 0xFF);
        *p++ = (uint8_t)((v >> 8) & 0xFF);
    } else if (type == DATA_UINT32_T) {
        uint32_t v; memcpy(&v, data, 4);
        *p++ = (uint8_t)(v & 0xFF);
        *p++ = (uint8_t)((v >> 8) & 0xFF);
        *p++ = (uint8_t)((v >> 16) & 0xFF);
        *p++ = (uint8_t)((v >> 24) & 0xFF);
    } else {
        memcpy(p, data, data_len); p += data_len;
    }

    uint16_t crc = crc16_modbus(uart2_dma_tx_buf + 2, (uint16_t)(2 + 1 + 2 + data_len));
    memcpy(p, &crc, 2); p += 2;
    *p++ = FRAME_TAIL_1;
    *p++ = FRAME_TAIL_2;

    uint16_t len = (uint16_t)(p - uart2_dma_tx_buf);

    // Launch DMA transmit directly
    HAL_StatusTypeDef st = HAL_UART_Transmit_DMA(rk3576_uart_port.huart, uart2_dma_tx_buf, len);
    if (st != HAL_OK) {
        // Release semaphore on failure to avoid deadlock
        xSemaphoreGive(rk3576_uart_port.uartTxDoneSem);
        return false;
    }
    // Semaphore will be released in HAL_UART_TxCpltCallback
    return true;
}

void parse_rk3576_uart_port_stream(const uint8_t *buf, uint16_t len)
{
    uint16_t index = 0;
    while (index + 11 <= len) {
        if (buf[index] == FRAME_HEADER_1 && buf[index + 1] == FRAME_HEADER_2) {
            const uint8_t *p = buf + index + 2;
            uint16_t frame_id; memcpy(&frame_id, p, 2); p += 2;
            uint8_t  data_type = *p++;
            (void)data_type;
            uint16_t data_len; memcpy(&data_len, p, 2); p += 2;
            if (data_len > FRAME_MAX_DATA_LEN || index + 11 + data_len > len) break;
            const uint8_t *data_ptr = p; p += data_len;
            uint16_t crc_recv; memcpy(&crc_recv, p, 2); p += 2;
            if (p[0] != FRAME_TAIL_1 || p[1] != FRAME_TAIL_2) { index++; continue; }
            uint16_t crc_calc = rk3576_uart_port.crc(buf + index + 2, (uint16_t)(2 + 1 + 2 + data_len));
            if (crc_recv != crc_calc) { index += (uint16_t)(2+2+1+2+data_len+2+2); continue; }
            UartFrame_Dispatch((FrameId_t)frame_id, data_ptr, data_len);
            index += (uint16_t)(2+2+1+2+data_len+2+2);
        } else {
            index++;
        }
    }
}

void parse_debug_uart_port_stream(const uint8_t *buf, uint16_t len)
{
    // same as business port stream
    parse_rk3576_uart_port_stream(buf, len);
}

uint16_t crc16_modbus(const uint8_t *buf, uint16_t len)
{
    uint16_t crc = 0xFFFF;
    for (uint16_t i = 0; i < len; i++) {
        crc ^= buf[i];
        for (uint8_t j = 0; j < 8; j++) {
            if (crc & 0x0001) crc = (uint16_t)((crc >> 1) ^ 0xA001);
            else crc >>= 1;
        }
    }
    return crc;
}

void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
    if (huart == debug_uart_port.huart) {
        (void)HAL_UARTEx_ReceiveToIdle_DMA(&huart1, uart1_dma_rx_buf, UART_RX_DMA_BUFFER_SIZE);
    }
}

void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
{
    BaseType_t hpw = pdFALSE;
    if (huart == rk3576_uart_port.huart)
        xSemaphoreGiveFromISR(rk3576_uart_port.uartTxDoneSem, &hpw);
    portYIELD_FROM_ISR(hpw);
}
