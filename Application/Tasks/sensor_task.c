#include "sensor_task.h"
#include "Pressure_sensor.h"
#include "ads1248.h"
#include "main.h"
#include "FreeRTOS.h"
#include "task.h"
#include "LOG.h"

extern u16 left_pressure, right_pressure; // from Pressure_sensor.c (Pa*? scaled to 0..40000)

// void SensorTask(void *argument)
// {
//     (void)argument;
//     ADS1248_Init();
//     ADS1248_SendCommand(0x08); // START conversions
//     uint8_t RTDChannel = RTD1; // 0:right, 1:left per project mapping
//     uint32_t rtd_miss_cnt = 0;

// for(;;)
// {
//     // RTD via SPI (read when RDY low)
//     if (HAL_GPIO_ReadPin(RTD_RDY_GPIO_Port, RTD_RDY_Pin) == GPIO_PIN_RESET)
//     {
//         int32_t raw = ADS1248_Read();
//         float t = ADC2Temperature(raw); // project-provided conversion, returns °C
//         if (RTDChannel == 0)gSensorData.tempR = t/100.0f;
//         else gSensorData.tempL = t/100.0f;
//         if (raw == 0 || t == 0.0f || t > 15000.0f) {
//             LOG_W("RTD raw=%ld t=%.2fC ch=%u (suspect wiring/REF)", (long)raw, t/100.0f, RTDChannel);
//         }
//         RTDChannel ^= 1; // 切换通道
//         ADS1248_ChangeChannel(RTDChannel);
//         rtd_miss_cnt = 0;
//     } else {
//         if (++rtd_miss_cnt >= 100) { // ~2s no data
//             rtd_miss_cnt = 0;
//             LOG_W("RTD RDY not ready, temps stale (ch=%u)", RTDChannel);
//         }
//     }

//     // Pressure sensors via I2C
//     pressure_sensor_read();
//     gSensorData.pressL = left_pressure / 1000.0f;  // to kPa
//     gSensorData.pressR = right_pressure / 1000.0f;

//     vTaskDelay(pdMS_TO_TICKS(20));

//     gSensorData.tick = xTaskGetTickCount();

//     //每 1 秒汇总一次
//     static uint16_t log_div = 0;
//     if (++log_div >= 100) {
//         log_div = 0;
//         LOG_I("[Summary] TL=%.2f°C TR=%.2f°C PL=%.2fkPa PR=%.2fkPa tick=%lu",
//               gSensorData.tempL, gSensorData.tempR,
//               gSensorData.pressL, gSensorData.pressR,
//               (unsigned long)gSensorData.tick);
//     }
// }

// }


void SensorTask(void *argument)
{
    (void)argument;
    ADS1248_Init();

    // 启动连续转换
    ADS1248_SendCommand(0x08); // START

    uint8_t RTDChannel = RTD2;       // 先假设 RTD2 起始
    uint8_t discard_next = 0;        // 是否丢弃下一次转换
    uint32_t rtd_miss_cnt = 0;

    // 先显式设一次通道，并丢弃一次
    ADS1248_ChangeChannel(RTDChannel);
    discard_next = 1;

    for (;;)
    {
        // RTD via SPI (read when RDY low)
        if (HAL_GPIO_ReadPin(RTD_RDY_GPIO_Port, RTD_RDY_Pin) == GPIO_PIN_RESET)
        {
            int32_t raw = ADS1248_Read();

            if (discard_next) {
                // 这次数据只是用来“冲掉”滤波器历史，不做任何处理
                discard_next = 0;
                // 重新计 miss
                rtd_miss_cnt = 0;
            } else {
                float t = ADC2Temperature(raw); // 返回 *100 的温度

                if (RTDChannel == RTD1) {
                    gSensorData.tempL = t / 100.0f;
                } else {
                    gSensorData.tempR = t / 100.0f;
                }

                if (raw == 0 || t == 0.0f || t > 15000.0f) {
                    LOG_W("RTD raw=%ld t=%.2fC ch=%u (suspect wiring/REF)",
                          (long)raw, t/100.0f, RTDChannel);
                }

                // 切换到另一通道
                RTDChannel ^= 1;
                ADS1248_ChangeChannel(RTDChannel);
                discard_next = 1;
                rtd_miss_cnt = 0;
            }
        } else {
            if (++rtd_miss_cnt >= 100) { // ~2s no data
                rtd_miss_cnt = 0;
                LOG_W("RTD RDY not ready, temps stale (ch=%u)", RTDChannel);
            }
        }

        // Pressure sensors via I2C
        pressure_sensor_read();
        gSensorData.pressL = left_pressure / 1000.0f;  // to kPa
        gSensorData.pressR = right_pressure / 1000.0f;

        vTaskDelay(pdMS_TO_TICKS(20));

        gSensorData.tick = xTaskGetTickCount();

        // 每 1 秒汇总一次
        static uint16_t log_div = 0;
        if (++log_div >= 50) {    // 20ms * 50 ≈ 1s
            log_div = 0;
            LOG_I("[Summary] TL=%.2f°C TR=%.2f°C PL=%.2fkPa PR=%.2fkPa tick=%lu",
                  gSensorData.tempL, gSensorData.tempR,
                  gSensorData.pressL, gSensorData.pressR,
                  (unsigned long)gSensorData.tick);
        }
    }
}
