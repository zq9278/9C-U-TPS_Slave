#include "treatment_temperature_sensor.h"

#include <stddef.h>
#include <string.h>
#define MODULE_LOG_ENABLED MODULE_LOG_SENSOR_TEMP_ENABLE
#include "Modules/Log/module_log.h"
#include "UserDrivers/user_drivers_board.h"

/* 初始化 ADS1248 及其 DRDY 信号量依赖。 */
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

    /* 模式切换时从固定首通道重新开始，保证左右采样顺序可预测。 */
    if (sensor->mode != mode)
    {
        sensor->mode = mode;
        sensor->next_channel = USER_ADS1248_CHANNEL_HEAT2;
    }
}

void TreatmentTemperatureSensor_Poll(TreatmentTemperatureSensor *sensor,
                                     volatile sensor_data_t *out)
{
    uint32_t temp_raw = 0U;
    float temp_c = USER_ADS1248_INVALID_TEMP_C;
    uint8_t read_ok = 0U;
    UserAds1248Channel read_channel;

    if ((sensor == NULL) || (out == NULL))
    {
        return;
    }

    /* 当前实现只保留双通道轮询和空闲两种采样模式。 */
    switch (sensor->mode)
    {
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
        /* Heat2 对应左眼，Heat1 对应右眼。 */
        if (read_channel == USER_ADS1248_CHANNEL_HEAT2)
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
        if (sensor->suppress_rtd_fail_log == 0U)
        {
            LOG_W("[Sensors] RTD read failed: ch=%u raw=0x%06lX",
                  (unsigned)read_channel,
                  (unsigned long)temp_raw);
        }
    }
}
