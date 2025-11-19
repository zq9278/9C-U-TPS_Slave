#include "sensor_task.h"
#include "Pressure_sensor.h"
#include "ads1248.h"
#include "main.h"
#include "FreeRTOS.h"
#include "task.h"

extern u16 left_pressure, right_pressure; // from Pressure_sensor.c (Pa*? scaled to 0..40000)

void SensorTask(void *argument)
{
    (void)argument;
    uint8_t rtd_ch = 0; // 0:right, 1:left per project mapping

    for(;;)
    {
        vTaskDelay(pdMS_TO_TICKS(10));

        // Pressure sensors via I2C
        pressure_sensor_read();
        gSensorData.pressL = left_pressure / 1000.0f;  // to kPa
        gSensorData.pressR = right_pressure / 1000.0f;

        // RTD via SPI (read when RDY low)
        if (HAL_GPIO_ReadPin(RTD_RDY_GPIO_Port, RTD_RDY_Pin) == GPIO_PIN_RESET)
        {
            int32_t raw = ADS1248_Read();
            float t = ADC2Temperature(raw); // project-provided conversion, returns °C
            if (rtd_ch == 0) gSensorData.tempR = t; else gSensorData.tempL = t;
            rtd_ch ^= 1;
            ADS1248_ChangeChannel(rtd_ch);
        }

        gSensorData.tick = xTaskGetTickCount();
    }
}
