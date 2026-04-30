/*
 * Host protocol dispatch: convert UART frames to application commands.
 */

#include "Uart_Communicate.h"

#include <string.h>
#include "FreeRTOS.h"
#include "queue.h"
#include "AppMain/system_app.h"
#include "HeaterShieldStatus/heater_shield_status.h"
#include "LOG.h"

class HostCommandPublisher {
public:
    bool pushU8(app_cmd_id_t id, uint8_t value) const
    {
        app_cmd_t cmd = {};
        cmd.id = id;
        cmd.v.u8 = value;
        return publish(cmd);
    }

    bool pushU16(app_cmd_id_t id, uint16_t value) const
    {
        app_cmd_t cmd = {};
        cmd.id = id;
        cmd.v.u16 = value;
        return publish(cmd);
    }

    bool pushF32(app_cmd_id_t id, float value) const
    {
        app_cmd_t cmd = {};
        cmd.id = id;
        cmd.v.f32 = value;
        return publish(cmd);
    }

private:
    bool publish(const app_cmd_t &cmd) const
    {
        app_event_t event = {};
        event.id = APP_EVT_HOST_COMMAND;
        event.v.host_cmd = cmd;
        return xQueueSend(gAppEventQueue, &event, 0) == pdPASS;
    }
};

class HostTelemetryPublisher {
public:
    bool sendU8(uint16_t frameId, uint8_t value) const
    {
        tx_frame_t tx = {};
        tx.type = TX_DATA_UINT8;
        tx.frame_id = frameId;
        tx.v.u8 = value;
        return xQueueSend(gTxQueue, &tx, 0) == pdPASS;
    }
};

class HostProtocol {
public:
    void dispatch(FrameId_t frameId, const uint8_t *data, uint16_t length)
    {
        switch (frameId) {
            case U8_HEARTBEAT_REQ:
                telemetry_.sendU8(U8_HEARTBEAT_ACK, 1);
                break;

            case F32_PRESSURE_SET_KPA:
                commands_.pushF32(APP_CMD_SET_PRESSURE_KPA, compensatePressureSet(readFloat(data, length)));
                break;

            case F32_LEFT_TEMP_SET_C:
                commands_.pushF32(APP_CMD_SET_TEMP, readFloat(data, length));
                break;

            case U8_LEFT_EYE_ENABLE:
                commands_.pushU8(APP_CMD_LEFT_ENABLE, readU8(data, length));
                break;

            case U8_RIGHT_EYE_ENABLE:
                commands_.pushU8(APP_CMD_RIGHT_ENABLE, readU8(data, length));
                break;

            case U8_LEFT_HEATER_FUSE_BLOW_CMD:
                HeaterShieldStatus_RequestFuseBlow(1, 1);
                break;

            case U8_RIGHT_HEATER_FUSE_BLOW_CMD:
                HeaterShieldStatus_RequestFuseBlow(1, 1);
                break;

            case U8_MODE_SELECT:
                commands_.pushU8(APP_CMD_MODE_SELECT, readU8(data, length));
                break;

            case U16_TREAT_TIME_MIN:
                commands_.pushU16(APP_CMD_SET_TREATMENT_TIME, readU16(data, length));
                break;

            case U8_START_TREATMENT:
                commands_.pushU8(APP_CMD_START, 1);
                break;

            case U8_STOP_TREATMENT:
                commands_.pushU8(APP_CMD_STOP, 0);
                break;

            case U8_PAUSE_RESUME_TREATMENT:
                commands_.pushU8(APP_CMD_PAUSE_RESUME, readU8(data, length));
                break;

            case U8_SAVE_SETTINGS:
                commands_.pushU8(APP_CMD_SAVE_PARAM, 0);
                break;

            default:
                LOG("unknown frame_id 0x%04X len=%d\r\n", frameId, length);
                break;
        }
    }

    static uint8_t readU8(const uint8_t *data, uint16_t length)
    {
        uint8_t value = 0;
        if (length >= sizeof(value)) {
            memcpy(&value, data, sizeof(value));
        }
        return value;
    }

    static uint16_t readU16(const uint8_t *data, uint16_t length)
    {
        uint16_t value = 0;
        if (length >= sizeof(value)) {
            memcpy(&value, data, sizeof(value));
        }
        return value;
    }

    static float readFloat(const uint8_t *data, uint16_t length)
    {
        float value = 0.0f;
        if (length >= sizeof(value)) {
            memcpy(&value, data, sizeof(value));
        }
        return value;
    }

private:
    static constexpr float kPressureSetCompAdd = 0.0f;
    static constexpr float kPressureSetCompMin = 0.0f;
    static constexpr float kPressureSetCompMax = 400.0f;

    static float compensatePressureSet(float rawSet)
    {
        float value = rawSet + kPressureSetCompAdd;
        if (value < kPressureSetCompMin) {
            value = kPressureSetCompMin;
        }
        if (value > kPressureSetCompMax) {
            value = kPressureSetCompMax;
        }
        return value;
    }

    HostCommandPublisher commands_;
    HostTelemetryPublisher telemetry_;
};

static HostProtocol gHostProtocol;

extern "C" void handle_config_data(const uint8_t *data_ptr, uint16_t data_len)
{
    (void)data_ptr;
    (void)data_len;
}

extern "C" uint8_t handle_uint8_t_data(const uint8_t *data_ptr, uint16_t data_len)
{
    return HostProtocol::readU8(data_ptr, data_len);
}

extern "C" uint16_t handle_uint16_t_data(const uint8_t *data_ptr, uint16_t data_len)
{
    return HostProtocol::readU16(data_ptr, data_len);
}

extern "C" float handle_float_data(const uint8_t *data_ptr, uint16_t data_len)
{
    return HostProtocol::readFloat(data_ptr, data_len);
}

extern "C" void UartFrame_Dispatch(FrameId_t frame_id, const uint8_t *data_ptr, uint16_t data_len)
{
    gHostProtocol.dispatch(frame_id, data_ptr, data_len);
}
