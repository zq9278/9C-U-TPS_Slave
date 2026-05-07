#include "Modules/communication/ascii_processor.h"

#include <string.h>

/**
 * @brief 复位单个逻辑通道的按行接收状态。
 *
 * 每当一整行已经交给上层处理完成，或者当前行因为超长被放弃时，
 * 都会把该通道恢复到“等待下一行”的初始状态。
 */
static void AsciiProcessor_ResetChannel(AsciiChannelState *state)
{
    if (state == NULL)
    {
        return;
    }

    state->line_length = 0U;
    state->read_active = 1U;
    (void)memset(state->line_buffer, 0, sizeof(state->line_buffer));
}

/**
 * @brief 把当前通道已经累积完成的一整行文本通过回调交给上层。
 *
 * 这里不负责清状态，调用者通常会在回调结束后再调用
 * `AsciiProcessor_ResetChannel()` 开始接收下一行。
 */
static void AsciiProcessor_EmitLine(AsciiProcessor *processor,
                                    CommunicationChannel channel,
                                    AsciiChannelState *state)
{
    char line[257];

    if ((processor == NULL) || (state == NULL))
    {
        return;
    }

    if (processor->callbacks.on_line_received == NULL)
    {
        return;
    }

    if (state->line_length == 0U)
    {
        return;
    }

    (void)memcpy(line, state->line_buffer, state->line_length);
    line[state->line_length] = '\0';

    processor->callbacks.on_line_received(processor->callbacks.context,
                                          channel,
                                          line,
                                          state->line_length);
}

/**
 * @brief 初始化 ASCII 解析器对象。
 *
 * 会清空所有通道的缓存和解析状态，让每个通道都从空行开始接收。
 */
void AsciiProcessor_Init(AsciiProcessor *processor)
{
    size_t i;

    if (processor == NULL)
    {
        return;
    }

    (void)memset(processor, 0, sizeof(*processor));

    for (i = 0U; i < COMM_CHANNEL_COUNT; ++i)
    {
        AsciiProcessor_ResetChannel(&processor->states[i]);
    }
}

/**
 * @brief 设置整行回调函数。
 */
void AsciiProcessor_SetCallbacks(AsciiProcessor *processor,
                                 const AsciiProcessorCallbacks *callbacks)
{
    if ((processor == NULL) || (callbacks == NULL))
    {
        return;
    }

    processor->callbacks = *callbacks;
}

/**
 * @brief 扫描一段连续字节流，并按换行符切出完整文本行。
 *
 * 这套逻辑很适合 UART DMA 场景：
 * 1. 每次 `Poll()` 只喂进“本轮新增字节”；
 * 2. 普通字符累计到当前行缓存；
 * 3. 遇到 `\r` 或 `\n` 时输出一整行；
 * 4. 如果当前行超出缓存长度，则丢弃这一整行，直到下一个换行重新同步。
 */
void AsciiProcessor_ProcessStream(AsciiProcessor *processor,
                                  CommunicationChannel channel,
                                  const uint8_t *data,
                                  size_t length)
{
    AsciiChannelState *state;

    if ((processor == NULL) ||
        (data == NULL) ||
        (length == 0U) ||
        ((size_t)channel >= COMM_CHANNEL_COUNT))
    {
        return;
    }

    state = &processor->states[(size_t)channel];

    while (length-- > 0U)
    {
        const uint8_t ch = *data++;
        const uint8_t is_eol = (uint8_t)((ch == '\r') || (ch == '\n'));

        if (is_eol != 0U)
        {
            if (state->read_active != 0U)
            {
                AsciiProcessor_EmitLine(processor, channel, state);
            }

            AsciiProcessor_ResetChannel(state);
            continue;
        }

        if (state->read_active == 0U)
        {
            continue;
        }

        if (state->line_length >= sizeof(state->line_buffer))
        {
            /*
             * 当前行过长时，放弃这一整行并等待后续换行重新同步，
             * 避免异常输入把解析器长期拖进错误状态。
             */
            state->line_length = 0U;
            state->read_active = 0U;
            continue;
        }

        state->line_buffer[state->line_length++] = ch;
    }
}
