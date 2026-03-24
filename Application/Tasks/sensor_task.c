#include "sensor_task.h"
#include "Pressure_sensor.h"
#include "ads1248.h"
#include "main.h"
#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h"
#include "LOG.h"

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

extern u16 left_pressure, right_pressure; // from Pressure_sensor.c (Pa*? scaled to 0..40000)

SemaphoreHandle_t gRtdDrdySem = NULL;

void SensorTask(void *argument)
{
    (void)argument;
    ADS1248_Init();
    uint8_t RTDChannel = RTD1; // 0:right, 1:left per project mapping
    bool drop_first_sample = false; // discard first conversion after channel switch
    float temp_buf[2][MEDIAN_WINDOW] = {0};
    uint8_t temp_count[2] = {0};
    uint8_t temp_idx[2] = {0};
    float press_buf[2][MEDIAN_WINDOW] = {0};
    uint8_t press_count[2] = {0};
    uint8_t press_idx[2] = {0};
    gRtdDrdySem = xSemaphoreCreateBinary();
    configASSERT(gRtdDrdySem != NULL);
for(;;)
{
    // RTD via SPI (wait for DRDY falling edge -> semaphore from EXTI)
    if (xSemaphoreTake(gRtdDrdySem, pdMS_TO_TICKS(50)) == pdTRUE) // shorter wait to speed cadence
    {
        if (drop_first_sample) {
            (void)ADS1248_Read(); // discard first sample on new channel
            drop_first_sample = false;
            continue;
        }
        int32_t raw = ADS1248_Read();
        float temp_c = ADC2Temperature(raw) / 100.0f; // project-provided conversion returns °C*100
        float prev = (RTDChannel == 0) ? gSensorData.tempR : gSensorData.tempL;
        bool bad_code = (raw == 0) || (raw == 0x7FFFFF) || (raw == 0xFFFFFF);
        bool valid = (!bad_code) && (temp_c > -20.0f) && (temp_c < 80.0f);
        if (valid) {
            temp_buf[RTDChannel][temp_idx[RTDChannel]] = temp_c;
            temp_idx[RTDChannel] = (temp_idx[RTDChannel] + 1) % MEDIAN_WINDOW;
            if (temp_count[RTDChannel] < MEDIAN_WINDOW) temp_count[RTDChannel]++;
            float filtered = median_filter(temp_buf[RTDChannel], temp_count[RTDChannel]);
            if (RTDChannel == 0) gSensorData.tempR = filtered;
            else gSensorData.tempL = filtered;
        } else {
            // keep previous value to avoid zeros/gaps
            if (RTDChannel == 0) gSensorData.tempR = prev;
            else gSensorData.tempL = prev;
        }
        RTDChannel ^= 1; // 切换通道
        ADS1248_ChangeChannel(RTDChannel);
        drop_first_sample = true; // drop first reading after channel switch
    } 

    // Pressure sensors via I2C
    pressure_sensor_read();
    float pressL_kpa = left_pressure / 1000.0f;
    float pressR_kpa = right_pressure / 1000.0f;
    press_buf[1][press_idx[1]] = pressL_kpa;
    press_idx[1] = (press_idx[1] + 1) % MEDIAN_WINDOW;
    if (press_count[1] < MEDIAN_WINDOW) press_count[1]++;
    press_buf[0][press_idx[0]] = pressR_kpa;
    press_idx[0] = (press_idx[0] + 1) % MEDIAN_WINDOW;
    if (press_count[0] < MEDIAN_WINDOW) press_count[0]++;
    gSensorData.pressL = median_filter(press_buf[1], press_count[1]) * 7.50062f;  // convert kPa -> mmHg
    gSensorData.pressR = median_filter(press_buf[0], press_count[0]) * 7.50062f;

    vTaskDelay(pdMS_TO_TICKS(1)); // faster loop to raise effective sampling rate

    gSensorData.tick = xTaskGetTickCount();

    //每 1 秒汇总一次
    static uint16_t log_div = 0;
    if (++log_div >= 100) {
        log_div = 0;
        // LOG_I("[Summary] TL=%.2f°C TR=%.2f°C PL=%.2fkPa PR=%.2fkPa tick=%lu",
        //       gSensorData.tempL, gSensorData.tempR,
        //       gSensorData.pressL, gSensorData.pressR,
        //       (unsigned long)gSensorData.tick);
    }
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
