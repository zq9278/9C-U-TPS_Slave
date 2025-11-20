#include "sensor_task.h"
#include "Pressure_sensor.h"
#include "ads1248.h"
#include "main.h"
#include "FreeRTOS.h"
#include "task.h"
#include "LOG.h"

extern u16 left_pressure, right_pressure; // from Pressure_sensor.c (Pa*? scaled to 0..40000)

void SensorTask(void *argument)
{
    (void)argument;
    ADS1248_Init();
    uint8_t RTDChannel = RTD1; // 0:right, 1:left per project mapping

for(;;)
{
    // RTD via SPI (read when RDY low)
    if (HAL_GPIO_ReadPin(RTD_RDY_GPIO_Port, RTD_RDY_Pin) == GPIO_PIN_RESET)
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
    gSensorData.pressL = left_pressure / 1000.0f;  // to kPa
    gSensorData.pressR = right_pressure / 1000.0f;

    vTaskDelay(pdMS_TO_TICKS(20));

    gSensorData.tick = xTaskGetTickCount();

    // 每 1 秒汇总一次
    // static uint16_t log_div = 0;
    // if (++log_div >= 10) {
    //     log_div = 0;
    //     LOG_I("[Summary] TL=%.2f°C TR=%.2f°C PL=%.2fkPa PR=%.2fkPa tick=%lu",
    //           gSensorData.tempL, gSensorData.tempR,
    //           gSensorData.pressL, gSensorData.pressR,
    //           (unsigned long)gSensorData.tick);
    // }
}

}
