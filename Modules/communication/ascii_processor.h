#ifndef MODULES_COMMUNICATION_ASCII_PROCESSOR_H
#define MODULES_COMMUNICATION_ASCII_PROCESSOR_H

#include <stddef.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief 当前支持的 ASCII 输入逻辑通道。
 *
 * 这里故意不直接暴露 HAL 的 UART 句柄，而是向上层暴露逻辑通道概念。
 * 这样即使底层物理串口更换了，上层业务依然只需要关心“来自 UART1 调试口”
 * 还是“来自 UART3 业务口”。
 */
typedef enum
{
    COMM_CHANNEL_UART1 = 0,
    COMM_CHANNEL_UART3 = 1,
    COMM_CHANNEL_COUNT
} CommunicationChannel;

/**
 * @brief 一整行 ASCII 文本接收完成时的回调类型。
 */
typedef void (*AsciiLineCallback)(void *context,
                                  CommunicationChannel channel,
                                  const char *line,
                                  size_t length);

/**
 * @brief ASCII 解析器回调集合。
 */
typedef struct
{
    AsciiLineCallback on_line_received;
    void *context;
} AsciiProcessorCallbacks;

/**
 * @brief 单个通道的 ASCII 行缓存状态。
 */
typedef struct
{
    uint8_t line_buffer[256];
    size_t line_length;
    uint8_t read_active;
} AsciiChannelState;

/**
 * @brief 按行解析 ASCII 字节流的解析器对象。
 *
 * 设计目标很直接：
 * 1. 上层把连续字节流喂进来；
 * 2. 遇到 `\r` 或 `\n` 就认定当前一行结束；
 * 3. 把完整一行通过回调抛给上层；
 * 4. 不同通道各自维护各自的行缓存，互不干扰。
 */
typedef struct
{
    AsciiProcessorCallbacks callbacks;
    AsciiChannelState states[COMM_CHANNEL_COUNT];
} AsciiProcessor;

/**
 * @brief 初始化 ASCII 流解析器。
 *
 * 会清空所有通道的行缓存和解析状态，让每个通道都回到“等待新一行”的初始状态。
 *
 * @param processor 待初始化的解析器对象。
 */
void AsciiProcessor_Init(AsciiProcessor *processor);

/**
 * @brief 设置“整行文本接收完成”回调。
 *
 * @param processor 目标解析器对象。
 * @param callbacks 回调集合。
 */
void AsciiProcessor_SetCallbacks(AsciiProcessor *processor,
                                 const AsciiProcessorCallbacks *callbacks);

/**
 * @brief 向指定通道输入一段连续字节流并尝试按行解析。
 *
 * @param processor 目标解析器对象。
 * @param channel   逻辑输入通道。
 * @param data      本次新增字节流。
 * @param length    本次新增字节数。
 */
void AsciiProcessor_ProcessStream(AsciiProcessor *processor,
                                  CommunicationChannel channel,
                                  const uint8_t *data,
                                  size_t length);

#ifdef __cplusplus
}
#endif

#endif
