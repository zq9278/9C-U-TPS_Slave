/* UART driver public definitions (clean) */

#ifndef UART_DRIVER_H
#define UART_DRIVER_H

#include <stdbool.h>
#include "FreeRTOS.h"
#include "semphr.h"
#include "stm32g0xx_hal.h"
#include "LOG.h"

/* Queues and buffers */
#define UART_TX_QUEUE_LENGTH     8
#define UART_TX_MSG_MAX_LEN      50
#define UART_RX_DMA_BUFFER_SIZE  100
#define UART_RX_QUEUE_SIZE       10

/* Frame markers */
#define FRAME_HEADER_1  0xAA
#define FRAME_HEADER_2  0x55
#define FRAME_TAIL_1    0x0D
#define FRAME_TAIL_2    0x0A

/* Limits */
#define FRAME_MAX_DATA_LEN  32
#define ASCII_CMD_MAX_LEN   128

/* Data type */
typedef enum {
    DATA_TYPE_NONE   = 0x00,
    DATA_UINT8_T     = 0x01,
    DATA_FLOAT       = 0x02,
    DATA_TYPE_TEXT   = 0x03,
    DATA_UINT16_T    = 0x04,
    DATA_UINT32_T    = 0x05,
} DataType_t;

/* Port abstraction */
typedef struct __attribute__((packed)) {
    UART_HandleTypeDef *huart;
    uint8_t           *dma_rx_buf;
    QueueHandle_t      rx_queue;
    QueueHandle_t      tx_queue;
    const char        *name;
    void     (*parser)(const uint8_t *buf, uint16_t len);
    bool      (*sender)(DataType_t type, uint16_t frame_id, const void *data);
    uint16_t  (*crc)(const uint8_t *buf, uint16_t len);
    SemaphoreHandle_t  uartTxDoneSem;
} UartPort_t;

/* Frame (reference) */
typedef struct __attribute__((packed)){
    uint8_t  header[2];
    uint16_t frame_id;
    uint8_t  data_type;
    uint16_t data_length;
    uint8_t  data[FRAME_MAX_DATA_LEN];
    uint16_t checksum;
    uint8_t  tail[2];
} __attribute__((packed)) Frame_t;

/* RX/TX messages */
typedef struct __attribute__((packed)){
    uint8_t  data[UART_RX_DMA_BUFFER_SIZE];
    uint16_t length;
} UartRxMessage_t;

typedef struct __attribute__((packed)){
    uint8_t  data[UART_TX_MSG_MAX_LEN];
    uint16_t length;
} UartTxMessage_t;

typedef struct __attribute__((packed)){
    char line[ASCII_CMD_MAX_LEN];
} AsciiCmdMessage_t;

typedef struct __attribute__((packed)) {
    uint8_t fan;
    uint8_t heat;
    uint8_t mist;
} ConfigData_t;

/* API */
void rk3576_uart_port_Init(UartPort_t *port);
void debug_uart_port_Init(UartPort_t *port);
void rk3576_uart_port_RxCallback(UartPort_t *port, uint16_t size);
void debug_uart_port_RxCallback(UartPort_t *port, uint16_t size);
void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size);
bool send_rk3576_uart_port_frame(DataType_t type, uint16_t frame_id, const void *data);
void parse_rk3576_uart_port_stream(const uint8_t *buf, uint16_t len);
void parse_debug_uart_port_stream(const uint8_t *buf, uint16_t len);
uint16_t crc16_modbus(const uint8_t *buf, uint16_t len);
void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart);
void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart);

#endif // UART_DRIVER_H

