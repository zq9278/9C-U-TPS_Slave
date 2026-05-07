#include "Modules/communication/Protocol/protocol_common.h"
#include "Modules/communication/Protocol/rk3576_protocol.h"

#include <string.h>

static void Rk3576Protocol_ResetRxState(ProtocolRxState *state)
{
    if (state == NULL)
    {
        return;
    }

    (void)memset(state, 0, sizeof(*state));
}

uint16_t Protocol_Crc16Modbus(const uint8_t *data, size_t length)
{
    uint16_t crc = 0xFFFFU;
    size_t index = 0U;

    if (data == NULL)
    {
        return crc;
    }

    while (index < length)
    {
        uint8_t bit = 0U;
        crc ^= data[index++];

        for (bit = 0U; bit < 8U; ++bit)
        {
            if ((crc & 0x0001U) != 0U)
            {
                crc = (uint16_t)((crc >> 1U) ^ 0xA001U);
            }
            else
            {
                crc >>= 1U;
            }
        }
    }

    return crc;
}

uint8_t Protocol_BuildFrame(uint16_t frame_id,
                            uint8_t data_type,
                            const uint8_t *payload,
                            uint16_t payload_length,
                            uint8_t *out_buffer,
                            uint16_t *out_length)
{
    uint8_t *cursor;
    uint16_t crc;

    if ((out_buffer == NULL) || (out_length == NULL))
    {
        return 0U;
    }
    if ((payload == NULL) && (payload_length != 0U))
    {
        return 0U;
    }
    if (payload_length > PROTOCOL_FRAME_MAX_PAYLOAD_LENGTH)
    {
        return 0U;
    }

    cursor = out_buffer;
    *cursor++ = PROTOCOL_FRAME_HEADER_1;
    *cursor++ = PROTOCOL_FRAME_HEADER_2;
    *cursor++ = (uint8_t)(frame_id & 0xFFU);
    *cursor++ = (uint8_t)((frame_id >> 8U) & 0xFFU);
    *cursor++ = data_type;
    *cursor++ = (uint8_t)(payload_length & 0xFFU);
    *cursor++ = (uint8_t)((payload_length >> 8U) & 0xFFU);

    if (payload_length > 0U)
    {
        (void)memcpy(cursor, payload, payload_length);
        cursor += payload_length;
    }

    crc = Protocol_Crc16Modbus(&out_buffer[2], (size_t)(5U + payload_length));
    *cursor++ = (uint8_t)(crc & 0xFFU);
    *cursor++ = (uint8_t)((crc >> 8U) & 0xFFU);
    *cursor++ = PROTOCOL_FRAME_TAIL_1;
    *cursor++ = PROTOCOL_FRAME_TAIL_2;

    *out_length = (uint16_t)(cursor - out_buffer);
    return 1U;
}

uint8_t Protocol_SendFrame(const ProtocolTransport *transport,
                           uint16_t frame_id,
                           uint8_t data_type,
                           const uint8_t *payload,
                           uint16_t payload_length)
{
    uint8_t frame[PROTOCOL_FRAME_MAX_LENGTH];
    uint16_t frame_length = 0U;

    if ((transport == NULL) || (transport->send_bytes == NULL))
    {
        return 0U;
    }

    if (Protocol_BuildFrame(frame_id, data_type, payload, payload_length, frame, &frame_length) == 0U)
    {
        return 0U;
    }

    return transport->send_bytes(transport->send_context, frame, frame_length);
}

void Rk3576Protocol_Init(Rk3576Protocol *protocol,
                         ProtocolSendBytesCallback send_callback,
                         void *send_context)
{
    if (protocol == NULL)
    {
        return;
    }

    (void)memset(protocol, 0, sizeof(*protocol));
    protocol->transport.send_bytes = send_callback;
    protocol->transport.send_context = send_context;
}

void Rk3576Protocol_SetFrameCallback(Rk3576Protocol *protocol,
                                     void (*callback)(void *context, const ProtocolFrameView *frame),
                                     void *context)
{
    if (protocol == NULL)
    {
        return;
    }

    protocol->on_valid_frame = callback;
    protocol->on_valid_frame_context = context;
}

void Rk3576Protocol_Input(Rk3576Protocol *protocol, const uint8_t *data, size_t length)
{
    ProtocolRxState *state;

    if ((protocol == NULL) || (data == NULL) || (length == 0U))
    {
        return;
    }

    state = &protocol->rx_state;

    while (length-- > 0U)
    {
        const uint8_t ch = *data++;

        if (state->collecting == 0U)
        {
            if (ch == PROTOCOL_FRAME_HEADER_1)
            {
                state->collecting = 1U;
                state->length = 1U;
                state->buffer[0] = ch;
            }
            continue;
        }

        if ((state->length == 1U) && (ch != PROTOCOL_FRAME_HEADER_2))
        {
            if (ch == PROTOCOL_FRAME_HEADER_1)
            {
                state->collecting = 1U;
                state->length = 1U;
                state->buffer[0] = ch;
            }
            else
            {
                Rk3576Protocol_ResetRxState(state);
            }
            continue;
        }

        if (state->length >= sizeof(state->buffer))
        {
            Rk3576Protocol_ResetRxState(state);
            continue;
        }

        state->buffer[state->length++] = ch;

        if ((state->length >= 7U) && (state->expected_length == 0U))
        {
            const uint16_t payload_length =
                (uint16_t)state->buffer[5] |
                (uint16_t)((uint16_t)state->buffer[6] << 8U);

            state->expected_length = (uint16_t)(PROTOCOL_FRAME_OVERHEAD + payload_length);
            if ((payload_length > PROTOCOL_FRAME_MAX_PAYLOAD_LENGTH) ||
                (state->expected_length > sizeof(state->buffer)))
            {
                Rk3576Protocol_ResetRxState(state);
            }
        }

        if ((state->expected_length != 0U) && (state->length >= state->expected_length))
        {
            const uint16_t payload_length =
                (uint16_t)state->buffer[5] |
                (uint16_t)((uint16_t)state->buffer[6] << 8U);
            const uint16_t crc_received =
                (uint16_t)state->buffer[7U + payload_length] |
                (uint16_t)((uint16_t)state->buffer[8U + payload_length] << 8U);
            const uint16_t crc_expected =
                Protocol_Crc16Modbus(&state->buffer[2], (size_t)(5U + payload_length));
            const uint8_t tail_ok =
                (uint8_t)((state->buffer[9U + payload_length] == PROTOCOL_FRAME_TAIL_1) &&
                          (state->buffer[10U + payload_length] == PROTOCOL_FRAME_TAIL_2));

            if ((crc_received == crc_expected) && (tail_ok != 0U))
            {
                ProtocolFrameView frame;

                frame.frame_id =
                    (uint16_t)state->buffer[2] |
                    (uint16_t)((uint16_t)state->buffer[3] << 8U);
                frame.data_type = state->buffer[4];
                frame.payload_length = payload_length;
                frame.payload = &state->buffer[7];

                if (protocol->on_valid_frame != NULL)
                {
                    protocol->on_valid_frame(protocol->on_valid_frame_context, &frame);
                }
            }

            Rk3576Protocol_ResetRxState(state);
        }
    }
}

uint8_t Rk3576Protocol_SendFrame(Rk3576Protocol *protocol,
                                 uint16_t frame_id,
                                 uint8_t data_type,
                                 const uint8_t *payload,
                                 uint16_t payload_length)
{
    if (protocol == NULL)
    {
        return 0U;
    }

    return Protocol_SendFrame(&protocol->transport,
                              frame_id,
                              data_type,
                              payload,
                              payload_length);
}
