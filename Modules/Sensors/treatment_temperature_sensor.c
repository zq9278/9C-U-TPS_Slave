#include "treatment_temperature_sensor.h"

#include <stddef.h>
#include <string.h>
#define MODULE_LOG_ENABLED MODULE_LOG_SENSOR_TEMP_ENABLE
#include "Modules/Log/module_log.h"
#include "UserDrivers/user_drivers_board.h"

static void TreatmentTemperatureSensor_ResetScan(TreatmentTemperatureSensor *sensor)
{
    if (sensor == NULL)
    {
        return;
    }

    sensor->next_channel = USER_ADS1248_CHANNEL_HEAT2;
    sensor->active_channel = USER_ADS1248_CHANNEL_HEAT2;
    sensor->scan_state = TEMP_SCAN_STATE_IDLE;
}

static uint8_t TreatmentTemperatureSensor_IsSingleMode(temp_acquire_mode_t mode)
{
    return (uint8_t)((mode == TEMP_ACQUIRE_MODE_SINGLE_LEFT) ||
                     (mode == TEMP_ACQUIRE_MODE_SINGLE_RIGHT));
}

static UserAds1248Channel TreatmentTemperatureSensor_ChannelForMode(temp_acquire_mode_t mode)
{
    if (mode == TEMP_ACQUIRE_MODE_SINGLE_RIGHT)
    {
        return USER_ADS1248_CHANNEL_HEAT1;
    }

    return USER_ADS1248_CHANNEL_HEAT2;
}

static void TreatmentTemperatureSensor_AdvanceNextChannel(TreatmentTemperatureSensor *sensor)
{
    if (sensor == NULL)
    {
        return;
    }

    sensor->next_channel = (sensor->active_channel == USER_ADS1248_CHANNEL_HEAT1)
                               ? USER_ADS1248_CHANNEL_HEAT2
                               : USER_ADS1248_CHANNEL_HEAT1;
}

static void TreatmentTemperatureSensor_LogReadFailure(const TreatmentTemperatureSensor *sensor,
                                                      UserAds1248Channel channel,
                                                      uint32_t raw_value,
                                                      const char *phase)
{
    if ((sensor != NULL) && (sensor->suppress_rtd_fail_log == 0U))
    {
        LOG_W("[Sensors] RTD %s failed: ch=%u raw=0x%06lX",
              phase,
              (unsigned)channel,
              (unsigned long)raw_value);
    }
}

void TreatmentTemperatureSensor_Init(TreatmentTemperatureSensor *sensor,
                                     SemaphoreHandle_t rtd_drdy_sem)
{
    if (sensor == NULL)
    {
        return;
    }

    (void)memset(sensor, 0, sizeof(*sensor));
    TreatmentTemperatureSensor_ResetScan(sensor);
    sensor->mode = TEMP_ACQUIRE_MODE_IDLE;

    UserDrivers_BoardInit();
    UserDrivers_BoardSetAds1248DrdySemaphore(rtd_drdy_sem);
    if (UserAds1248_Init(&gUserAds1248) == 0U)
    {
        LOG_W("[Sensors] ADS1248 init failed");
    }
}

void TreatmentTemperatureSensor_SetMode(TreatmentTemperatureSensor *sensor,
                                        temp_acquire_mode_t mode,
                                        uint8_t suppress_rtd_fail_log)
{
    if (sensor == NULL)
    {
        return;
    }

    sensor->suppress_rtd_fail_log = suppress_rtd_fail_log;

    if (sensor->mode != mode)
    {
        sensor->mode = mode;
        TreatmentTemperatureSensor_ResetScan(sensor);
        if (mode == TEMP_ACQUIRE_MODE_DUAL_SCAN)
        {
            sensor->scan_state = TEMP_SCAN_STATE_SWITCH_CHANNEL;
        }
        else if (TreatmentTemperatureSensor_IsSingleMode(mode) != 0U)
        {
            sensor->next_channel = TreatmentTemperatureSensor_ChannelForMode(mode);
            sensor->active_channel = sensor->next_channel;
            sensor->scan_state = TEMP_SCAN_STATE_SWITCH_CHANNEL;
        }
    }
}

void TreatmentTemperatureSensor_Poll(TreatmentTemperatureSensor *sensor,
                                     volatile sensor_data_t *out)
{
    uint32_t temp_raw = 0U;
    float temp_c = USER_ADS1248_INVALID_TEMP_C;
    uint8_t read_ok = 0U;

    if ((sensor == NULL) || (out == NULL))
    {
        return;
    }

    switch (sensor->mode)
    {
    case TEMP_ACQUIRE_MODE_DUAL_SCAN:
    case TEMP_ACQUIRE_MODE_SINGLE_LEFT:
    case TEMP_ACQUIRE_MODE_SINGLE_RIGHT:
        break;

    case TEMP_ACQUIRE_MODE_IDLE:
    default:
        TreatmentTemperatureSensor_ResetScan(sensor);
        out->tempL = USER_ADS1248_INVALID_TEMP_C;
        out->tempR = USER_ADS1248_INVALID_TEMP_C;
        return;
    }

    switch ((treatment_temp_scan_state_t)sensor->scan_state)
    {
    case TEMP_SCAN_STATE_IDLE:
        if (TreatmentTemperatureSensor_IsSingleMode(sensor->mode) != 0U)
        {
            sensor->next_channel = TreatmentTemperatureSensor_ChannelForMode(sensor->mode);
        }
        sensor->scan_state = TEMP_SCAN_STATE_SWITCH_CHANNEL;
        break;

    case TEMP_SCAN_STATE_SWITCH_CHANNEL:
        sensor->active_channel = sensor->next_channel;
        if (UserAds1248_SelectChannel(&gUserAds1248, sensor->active_channel) == 0U)
        {
            TreatmentTemperatureSensor_LogReadFailure(sensor,
                                                      sensor->active_channel,
                                                      0U,
                                                      "select");
            return;
        }
        sensor->scan_state = TEMP_SCAN_STATE_WAIT_DISCARD_SAMPLE;
        break;

    case TEMP_SCAN_STATE_WAIT_DISCARD_SAMPLE:
        if (UserAds1248_IsDataReady(&gUserAds1248) == 0U)
        {
            return;
        }
        if (UserAds1248_ReadRaw(&gUserAds1248, &temp_raw) == 0U)
        {
            TreatmentTemperatureSensor_LogReadFailure(sensor,
                                                      sensor->active_channel,
                                                      temp_raw,
                                                      "discard");
            sensor->scan_state = TEMP_SCAN_STATE_SWITCH_CHANNEL;
            return;
        }
        sensor->scan_state = TEMP_SCAN_STATE_WAIT_VALID_SAMPLE;
        break;

    case TEMP_SCAN_STATE_WAIT_VALID_SAMPLE:
        if (UserAds1248_IsDataReady(&gUserAds1248) == 0U)
        {
            return;
        }
        if ((UserAds1248_ReadRaw(&gUserAds1248, &temp_raw) == 0U) ||
            (UserAds1248_CodeToTemperatureC(temp_raw, &temp_c) == 0U))
        {
            TreatmentTemperatureSensor_LogReadFailure(sensor,
                                                      sensor->active_channel,
                                                      temp_raw,
                                                      "sample");
            sensor->scan_state = TEMP_SCAN_STATE_SWITCH_CHANNEL;
            return;
        }

        read_ok = 1U;
        if (sensor->mode == TEMP_ACQUIRE_MODE_DUAL_SCAN)
        {
            TreatmentTemperatureSensor_AdvanceNextChannel(sensor);
            sensor->scan_state = TEMP_SCAN_STATE_SWITCH_CHANNEL;
        }
        else
        {
            sensor->next_channel = sensor->active_channel;
            sensor->scan_state = TEMP_SCAN_STATE_WAIT_VALID_SAMPLE;
        }
        break;

    default:
        TreatmentTemperatureSensor_ResetScan(sensor);
        sensor->scan_state = TEMP_SCAN_STATE_SWITCH_CHANNEL;
        return;
    }

    if (read_ok != 0U)
    {
        if (sensor->mode == TEMP_ACQUIRE_MODE_SINGLE_LEFT)
        {
            out->tempR = USER_ADS1248_INVALID_TEMP_C;
        }
        else if (sensor->mode == TEMP_ACQUIRE_MODE_SINGLE_RIGHT)
        {
            out->tempL = USER_ADS1248_INVALID_TEMP_C;
        }

        /* Heat2 对应左眼，Heat1 对应右眼。 */
        if (sensor->active_channel == USER_ADS1248_CHANNEL_HEAT2)
        {
            out->tempL = temp_c;
        }
        else
        {
            out->tempR = temp_c;
        }
    }
}
