#include "treatment_temperature_sensor.h"

#include <stddef.h>
#include <string.h>
#include "Modules/Log/module_log.h"
#include "UserDrivers/user_drivers_board.h"

static void TreatmentTemperatureSensor_LogMode(temp_acquire_mode_t mode)
{
    switch (mode)
    {
    case TEMP_ACQUIRE_MODE_LEFT_FIXED:
        LOG_I("[Sensors] temperature mode=left_fixed");
        break;
    case TEMP_ACQUIRE_MODE_RIGHT_FIXED:
        LOG_I("[Sensors] temperature mode=right_fixed");
        break;
    case TEMP_ACQUIRE_MODE_DUAL_SCAN:
        LOG_I("[Sensors] temperature mode=dual_scan");
        break;
    case TEMP_ACQUIRE_MODE_IDLE:
    default:
        LOG_I("[Sensors] temperature mode=idle");
        break;
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
    sensor->next_channel = USER_ADS1248_CHANNEL_HEAT2;
    sensor->mode = TEMP_ACQUIRE_MODE_IDLE;

    UserDrivers_BoardInit();
    UserDrivers_BoardSetAds1248DrdySemaphore(rtd_drdy_sem);
    if (UserAds1248_Init(&gUserAds1248) != 0U)
    {
        LOG_I("[Sensors] ADS1248 dual-channel scan enabled");
    }
    else
    {
        LOG_W("[Sensors] ADS1248 init failed");
    }
}

void TreatmentTemperatureSensor_SetMode(TreatmentTemperatureSensor *sensor,
                                        temp_acquire_mode_t mode)
{
    if (sensor == NULL)
    {
        return;
    }

    if (sensor->mode != mode)
    {
        sensor->mode = mode;
        sensor->next_channel = USER_ADS1248_CHANNEL_HEAT2;
        TreatmentTemperatureSensor_LogMode(mode);
    }
}

void TreatmentTemperatureSensor_Poll(TreatmentTemperatureSensor *sensor,
                                     volatile sensor_data_t *out)
{
    const UserAds1248Channel left_channel = USER_ADS1248_CHANNEL_HEAT2;
    const UserAds1248Channel right_channel = USER_ADS1248_CHANNEL_HEAT1;
    uint32_t temp_raw = 0U;
    float temp_c = USER_ADS1248_INVALID_TEMP_C;
    uint8_t read_ok = 0U;
    UserAds1248Channel read_channel;

    if ((sensor == NULL) || (out == NULL))
    {
        return;
    }

    switch (sensor->mode)
    {
    case TEMP_ACQUIRE_MODE_LEFT_FIXED:
        out->tempR = USER_ADS1248_INVALID_TEMP_C;
        read_channel = left_channel;
        read_ok = UserAds1248_ReadSingleChannelTemperatureC(&gUserAds1248,
                                                            read_channel,
                                                            &temp_c,
                                                            &temp_raw);
        break;

    case TEMP_ACQUIRE_MODE_RIGHT_FIXED:
        out->tempL = USER_ADS1248_INVALID_TEMP_C;
        read_channel = right_channel;
        read_ok = UserAds1248_ReadSingleChannelTemperatureC(&gUserAds1248,
                                                            read_channel,
                                                            &temp_c,
                                                            &temp_raw);
        break;

    case TEMP_ACQUIRE_MODE_DUAL_SCAN:
        read_channel = sensor->next_channel;
        read_ok = UserAds1248_ReadTemperatureC(&gUserAds1248,
                                               read_channel,
                                               &temp_c,
                                               &temp_raw);
        sensor->next_channel = (sensor->next_channel == USER_ADS1248_CHANNEL_HEAT1)
                                   ? USER_ADS1248_CHANNEL_HEAT2
                                   : USER_ADS1248_CHANNEL_HEAT1;
        break;

    case TEMP_ACQUIRE_MODE_IDLE:
    default:
        out->tempL = USER_ADS1248_INVALID_TEMP_C;
        out->tempR = USER_ADS1248_INVALID_TEMP_C;
        return;
    }

    if (read_ok != 0U)
    {
        if (read_channel == left_channel)
        {
            out->tempL = temp_c;
        }
        else
        {
            out->tempR = temp_c;
        }
    }
    else
    {
        LOG_W("[Sensors] RTD read failed: ch=%u raw=0x%06lX",
              (unsigned)read_channel,
              (unsigned long)temp_raw);
    }
}
