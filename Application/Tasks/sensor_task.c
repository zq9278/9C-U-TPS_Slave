#include "sensor_task.h"
#include "Pressure_sensor.h"
#include "ads1248_v2.h"
#include "LOG.h"
#include "main.h"
#include "FreeRTOS.h"
#include "task.h"

#define MEDIAN_WINDOW 3
static float median_filter(const float *buf, uint8_t count)
{
    float tmp[MEDIAN_WINDOW];
    for (uint8_t i = 0; i < count; ++i) tmp[i] = buf[i];
    for (uint8_t i = 0; i + 1 < count; ++i) {
        for (uint8_t j = i + 1; j < count; ++j) {
            if (tmp[j] < tmp[i]) {
                float t = tmp[i];
                tmp[i] = tmp[j];
                tmp[j] = t;
            }
        }
    }
    if (count == 0) return 0.0f;
    if (count & 1) return tmp[count / 2];
    return 0.5f * (tmp[count / 2 - 1] + tmp[count / 2]);
}

extern u16 left_pressure, right_pressure;

/* RTD_RDY is configured as EXTI falling in gpio.c. The ADS1248 v2 driver waits
 * on this semaphore so temperature acquisition follows the same ready-edge
 * model as the older implementation.
 */
SemaphoreHandle_t gRtdDrdySem = NULL;

void SensorTask(void *argument)
{
    (void)argument;
    uint32_t temp_raw = 0;
    float temp_c = ADS1248V2_INVALID_TEMP_C;
    ads1248v2_channel_t temp_channel = ADS1248V2_CHANNEL_HEAT2;
    float press_buf[2][MEDIAN_WINDOW] = {0};
    uint8_t press_count[2] = {0};
    uint8_t press_idx[2] = {0};

    if (gRtdDrdySem == NULL) {
        /* One binary semaphore is enough because RTD_RDY is a single ready
         * line shared by the ADS1248 temperature converter.
         */
        gRtdDrdySem = xSemaphoreCreateBinary();
    }

    /* New ADS1248 v2 path:
     * initialize once, then alternate between the two NTC channels using the
     * driver helper that performs channel select, first-sample discard and
     * temperature conversion internally.
     */
    ADS1248V2_Init();
    LOG_I("[RTD2] dual-channel scan enabled");

    for (;;)
    {
        /* Alternate channels every loop so we return to the normal
         * left/right temperature acquisition pattern while still using the
         * new ADS1248 v2 driver.
         */
        if (ADS1248V2_ReadTemperatureC(temp_channel, &temp_c, &temp_raw)) {
            if (temp_channel == ADS1248V2_CHANNEL_HEAT1) {
                gSensorData.tempL = temp_c;
            } else {
                gSensorData.tempR = temp_c;
            }
        } else {
            LOG_W("[RTD2] read failed: ch=%u raw=0x%06lX",
                  (unsigned)temp_channel,
                  (unsigned long)temp_raw);
        }

        temp_channel = (temp_channel == ADS1248V2_CHANNEL_HEAT1)
                     ? ADS1248V2_CHANNEL_HEAT2
                     : ADS1248V2_CHANNEL_HEAT1;

        pressure_sensor_read();
        {
            float pressL_kpa = left_pressure / 1000.0f;
            float pressR_kpa = right_pressure / 1000.0f;

            press_buf[1][press_idx[1]] = pressL_kpa;
            press_idx[1] = (press_idx[1] + 1) % MEDIAN_WINDOW;
            if (press_count[1] < MEDIAN_WINDOW) press_count[1]++;

            press_buf[0][press_idx[0]] = pressR_kpa;
            press_idx[0] = (press_idx[0] + 1) % MEDIAN_WINDOW;
            if (press_count[0] < MEDIAN_WINDOW) press_count[0]++;

            gSensorData.pressL = median_filter(press_buf[1], press_count[1]) * 7.50062f;
            gSensorData.pressR = median_filter(press_buf[0], press_count[0]) * 7.50062f;
        }

        vTaskDelay(pdMS_TO_TICKS(2));

        gSensorData.tick = xTaskGetTickCount();
    }
}

void HAL_GPIO_EXTI_Falling_Callback(uint16_t GPIO_Pin)
{
    if ((GPIO_Pin == RTD_RDY_Pin) && (gRtdDrdySem != NULL)) {
        BaseType_t xHigherPriorityTaskWoken = pdFALSE;
        xSemaphoreGiveFromISR(gRtdDrdySem, &xHigherPriorityTaskWoken);
        portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
    }
}
