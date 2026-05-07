#include "treatment_pressure_sensor.h"

#include <stddef.h>
#include <string.h>
#include "Modules/Log/module_log.h"
#include "UserDrivers/user_drivers_board.h"

#define TREATMENT_PRESSURE_SENSOR_SAMPLE_PERIOD_MS 10U
#define TREATMENT_PRESSURE_SENSOR_SCALE_KPA 7.50062f
#define TREATMENT_PRESSURE_SENSOR_LEFT_CHANNEL 1U
#define TREATMENT_PRESSURE_SENSOR_RIGHT_CHANNEL 0U

static float TreatmentPressureSensor_Median3(const float *buf, uint8_t count)
{
    float tmp[TREATMENT_PRESSURE_SENSOR_MEDIAN_WINDOW];
    uint8_t i;
    uint8_t j;

    if (count == 0U)
    {
        return 0.0f;
    }

    for (i = 0U; i < count; ++i)
    {
        tmp[i] = buf[i];
    }

    for (i = 0U; (uint8_t)(i + 1U) < count; ++i)
    {
        for (j = (uint8_t)(i + 1U); j < count; ++j)
        {
            if (tmp[j] < tmp[i])
            {
                float swap = tmp[i];
                tmp[i] = tmp[j];
                tmp[j] = swap;
            }
        }
    }

    if ((count & 1U) != 0U)
    {
        return tmp[count / 2U];
    }

    return 0.5f * (tmp[(count / 2U) - 1U] + tmp[count / 2U]);
}

static void TreatmentPressureSensor_PushSample(TreatmentPressureSensor *sensor,
                                               uint8_t channel,
                                               float value_kpa,
                                               float *filtered_kpa)
{
    sensor->sample_buf[channel][sensor->sample_index[channel]] = value_kpa;
    sensor->sample_index[channel] =
        (uint8_t)((sensor->sample_index[channel] + 1U) % TREATMENT_PRESSURE_SENSOR_MEDIAN_WINDOW);

    if (sensor->sample_count[channel] < TREATMENT_PRESSURE_SENSOR_MEDIAN_WINDOW)
    {
        ++sensor->sample_count[channel];
    }

    *filtered_kpa = TreatmentPressureSensor_Median3(sensor->sample_buf[channel],
                                                    sensor->sample_count[channel]);
}

void TreatmentPressureSensor_Init(TreatmentPressureSensor *sensor)
{
    if (sensor == NULL)
    {
        return;
    }

    (void)memset(sensor, 0, sizeof(*sensor));
    sensor->next_sample_tick = 0U;
    UserDrivers_BoardInit();
}

void TreatmentPressureSensor_Poll(TreatmentPressureSensor *sensor,
                                  TickType_t now,
                                  volatile sensor_data_t *out)
{
    if ((sensor == NULL) || (out == NULL))
    {
        return;
    }

    if ((int32_t)(now - sensor->next_sample_tick) < 0)
    {
        return;
    }

    sensor->next_sample_tick = now + pdMS_TO_TICKS(TREATMENT_PRESSURE_SENSOR_SAMPLE_PERIOD_MS);

    if (UserPressureSensor_Read(&gUserPressureSensor) != 0U)
    {
        float left_kpa =
            ((float)gUserPressureSensor.left.pressure_raw / 1000.0f) *
            TREATMENT_PRESSURE_SENSOR_SCALE_KPA;
        float right_kpa =
            ((float)gUserPressureSensor.right.pressure_raw / 1000.0f) *
            TREATMENT_PRESSURE_SENSOR_SCALE_KPA;
        float filtered_left;
        float filtered_right;

        TreatmentPressureSensor_PushSample(sensor,
                                           TREATMENT_PRESSURE_SENSOR_LEFT_CHANNEL,
                                           left_kpa,
                                           &filtered_left);
        TreatmentPressureSensor_PushSample(sensor,
                                           TREATMENT_PRESSURE_SENSOR_RIGHT_CHANNEL,
                                           right_kpa,
                                           &filtered_right);
        out->pressL = filtered_left;
        out->pressR = filtered_right;
    }
    else
    {
        LOG_W("[Sensors] pressure read failed");
    }
}
