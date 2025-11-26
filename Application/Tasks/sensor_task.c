#include "sensor_task.h"
#include "Pressure_sensor.h"
#include "ads1248.h"
#include "main.h"
#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h"
#include "LOG.h"

extern u16 left_pressure, right_pressure; // from Pressure_sensor.c (Pa*? scaled to 0..40000)

SemaphoreHandle_t gRtdDrdySem = NULL;

void SensorTask(void *argument)
{
    (void)argument;
    ADS1248_Init();
    uint8_t RTDChannel = RTD1; // 0:right, 1:left per project mapping
    gRtdDrdySem = xSemaphoreCreateBinary();
    configASSERT(gRtdDrdySem != NULL);
for(;;)
{
    // RTD via SPI (wait for DRDY falling edge -> semaphore from EXTI)
    if (xSemaphoreTake(gRtdDrdySem, pdMS_TO_TICKS(100)) == pdTRUE)
    {
        int32_t raw = ADS1248_Read();
        float t = ADC2Temperature(raw); // project-provided conversion, returns °C
        if (RTDChannel == 0)gSensorData.tempR = t/100.0f;
        else gSensorData.tempL = t/100.0f;
        RTDChannel ^= 1; // 切换通道
        ADS1248_ChangeChannel(RTDChannel);
    } 

    // Pressure sensors via I2C
    pressure_sensor_read();
    float pressL_kpa = left_pressure / 1000.0f;
    float pressR_kpa = right_pressure / 1000.0f;
    gSensorData.pressL = pressL_kpa * 7.50062f;  // convert kPa -> mmHg
    gSensorData.pressR = pressR_kpa * 7.50062f;

    vTaskDelay(pdMS_TO_TICKS(2));

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
