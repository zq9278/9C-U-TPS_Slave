/*
 * @Author: zhangqi
 * @Date: 2024-12-09 14:39:13
 * @Last Modified by: zhangqi
 * @Last Modified time: 2024-12-09 18:24:58
 */

#include "Pressure_sensor.h"
#include "i2c.h"
#include "cmsis_os.h"
#include <stdint.h>

extern I2C_HandleTypeDef hi2c1;
extern I2C_HandleTypeDef hi2c2;
uint8_t right_raw_data[6] = {0};
uint8_t left_raw_data[6] = {0};
uint8_t measure_cmd = 0xAC;//发送开始测量命令
uint8_t right_sensor_status = 0;//压力传感器状态
uint8_t left_sensor_status = 0;//压力传感器状态
//float left_pressure_temp, right_pressure_temp;
float left_temperature_temp, right_temperature_temp;
u16 right_pressure, left_pressure;
uint8_t right_temperature, left_temperature;


void pressure_sensor_read() {
  HAL_I2C_Master_Receive(&hi2c1, 0xF1, &left_sensor_status, 1, HAL_MAX_DELAY);//左眼
  if (left_sensor_status & (1 << 5)) {
  } else {
    HAL_I2C_Master_Transmit(&hi2c1, 0xF0, &measure_cmd, 1, HAL_MAX_DELAY);
    osDelay(10);
  }
  HAL_I2C_Master_Receive(&hi2c1, 0xF1, left_raw_data, 6, HAL_MAX_DELAY);
  uint32_t left_pressure_raw =
      (left_raw_data[1] << 16) | (left_raw_data[2] << 8) | left_raw_data[3];
//  uint16_t left_temp_raw = (left_raw_data[4] << 8) | left_raw_data[5];
//  left_pressure_temp =
//      (left_pressure_raw / (float)(1 << 24)) * 40.0; // 鍋囪?鹃噺绋嬩负 0-100KPa
//  left_temperature_temp = (left_temp_raw / 65536.0f) * 190.0 - 40.0;
  left_pressure = (left_pressure_raw>>8)*40000/65536;
  left_temperature = (uint8_t)(left_temperature_temp);

  HAL_I2C_Master_Receive(&hi2c2, 0xF1, &right_sensor_status, 1, HAL_MAX_DELAY);  //右眼
  if (right_sensor_status & (1 << 5)) {
  } else {
    HAL_I2C_Master_Transmit(&hi2c2, 0xF0, &measure_cmd, 1, HAL_MAX_DELAY);
    osDelay(10);
  }
  HAL_I2C_Master_Receive(&hi2c2, 0xF1, right_raw_data, 6, HAL_MAX_DELAY);
  uint32_t right_pressure_raw =
      (right_raw_data[1] << 16) | (right_raw_data[2] << 8) | right_raw_data[3];
//  uint16_t right_temp_raw = (right_raw_data[4] << 8) | right_raw_data[5];
//  right_pressure_temp =    
//      (right_pressure_raw / (float)(1 << 24)) * 40.0; // 鍋囪?鹃噺绋嬩负 0-40KPa
//  right_temperature_temp = (right_temp_raw / 65536.0f) * 190.0 - 40.0;
  right_pressure = (right_pressure_raw>>8)*40000/65536;
  right_temperature = (uint8_t)(right_temperature_temp);


    
}


