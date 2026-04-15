#include "sensor_task.h"
#include "Pressure_sensor.h"
#include "ads1248.h"
#include "LOG.h"
#include "main.h"
#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h"

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

SemaphoreHandle_t gRtdDrdySem = NULL;

void SensorTask(void *argument)
{
    (void)argument;
    ADS1248_Init();
    uint8_t RTDChannel = RTD1; // 0:right, 1:left per project mapping
    bool drop_first_sample = false;
    float press_buf[2][MEDIAN_WINDOW] = {0};
    uint8_t press_count[2] = {0};
    uint8_t press_idx[2] = {0};

    gRtdDrdySem = xSemaphoreCreateBinary();
    configASSERT(gRtdDrdySem != NULL);

    for (;;)
    {
        if (xSemaphoreTake(gRtdDrdySem, pdMS_TO_TICKS(100)) == pdTRUE)
        {
            if (drop_first_sample) {
                (void)ADS1248_Read();
                drop_first_sample = false;
            } else {
                u32 raw = ADS1248_Read();
                u16 temp_code = ADC2Temperature(raw);
                float temp_c = temp_code / 100.0f;
                bool bad_code = (raw == 0) || (raw == 0x7FFFFF) || (raw == 0xFFFFFF);
                bool valid = (!bad_code) &&
                             (temp_code != ADS1248_TEMP_INVALID) &&
                             (temp_c > -20.0f) &&
                             (temp_c < 80.0f);

                if (valid) {
                    if (RTDChannel == 0) gSensorData.tempR = temp_c;
                    else gSensorData.tempL = temp_c;
                } else {
                    LOG_W("[RTD] invalid ch=%u raw=0x%06lX temp_code=0x%04X temp=%.2f",
                          (unsigned)RTDChannel,
                          (unsigned long)raw,
                          (unsigned)temp_code,
                          (double)temp_c);
                }

                RTDChannel ^= 1;
                ADS1248_ChangeChannel(RTDChannel);
                drop_first_sample = true;
            }
        }

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
    if (GPIO_Pin == RTD_RDY_Pin && gRtdDrdySem != NULL) {
        BaseType_t hpw = pdFALSE;
        xSemaphoreGiveFromISR(gRtdDrdySem, &hpw);
        portYIELD_FROM_ISR(hpw);
    }
}
