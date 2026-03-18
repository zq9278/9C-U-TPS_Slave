/*
 * @Author: zhangqi
 * @Date: 2024-12-09 14:39:13
 * @Last Modified by: zhangqi
 * @Last Modified time: 2024-12-09 18:24:58
 */

#include "Pressure_sensor.h"
#include "i2c.h"
#include "cmsis_os.h"
#include "LOG.h"
#include <stdint.h>

extern I2C_HandleTypeDef hi2c1;
extern I2C_HandleTypeDef hi2c2;

uint8_t right_raw_data[6] = {0};
uint8_t left_raw_data[6] = {0};
uint8_t measure_cmd = 0xAC;
uint8_t right_sensor_status = 0;
uint8_t left_sensor_status = 0;
float left_temperature_temp, right_temperature_temp;
u16 right_pressure, left_pressure;
uint8_t right_temperature, left_temperature;

#define PRESSURE_SENSOR_I2C_TIMEOUT_MS 20U
#define PRESSURE_SENSOR_I2C_RETRY_COUNT 2U

static HAL_StatusTypeDef pressure_i2c_read_one(I2C_HandleTypeDef *hi2c,
                                               uint8_t *sensor_status,
                                               uint8_t *raw_data,
                                               u16 *pressure_out,
                                               const char *name)
{
  for (uint8_t attempt = 0; attempt < PRESSURE_SENSOR_I2C_RETRY_COUNT; ++attempt) {
    HAL_StatusTypeDef st = HAL_I2C_Master_Receive(hi2c, 0xF1, sensor_status, 1, PRESSURE_SENSOR_I2C_TIMEOUT_MS);
    if (st != HAL_OK) {
      LOG_W("%s I2C status read failed: st=%d err=0x%08lx attempt=%u",
            name, st, HAL_I2C_GetError(hi2c), (unsigned)(attempt + 1U));
      I2C_Recover(hi2c);
      continue;
    }

    if (((*sensor_status) & (1U << 5)) == 0U) {
      st = HAL_I2C_Master_Transmit(hi2c, 0xF0, &measure_cmd, 1, PRESSURE_SENSOR_I2C_TIMEOUT_MS);
      if (st != HAL_OK) {
        LOG_W("%s I2C measure cmd failed: st=%d err=0x%08lx attempt=%u",
              name, st, HAL_I2C_GetError(hi2c), (unsigned)(attempt + 1U));
        I2C_Recover(hi2c);
        continue;
      }
      osDelay(10);
    }

    st = HAL_I2C_Master_Receive(hi2c, 0xF1, raw_data, 6, PRESSURE_SENSOR_I2C_TIMEOUT_MS);
    if (st != HAL_OK) {
      LOG_W("%s I2C data read failed: st=%d err=0x%08lx attempt=%u",
            name, st, HAL_I2C_GetError(hi2c), (unsigned)(attempt + 1U));
      I2C_Recover(hi2c);
      continue;
    }

    {
      uint32_t pressure_raw = ((uint32_t)raw_data[1] << 16) |
                              ((uint32_t)raw_data[2] << 8) |
                              (uint32_t)raw_data[3];
      *pressure_out = (u16)(((pressure_raw >> 8) * 100000U) / 65536U);
    }
    return HAL_OK;
  }

  return HAL_ERROR;
}

void pressure_sensor_read(void)
{
  u16 last_left_pressure = left_pressure;
  u16 last_right_pressure = right_pressure;

  if (pressure_i2c_read_one(&hi2c1, &left_sensor_status, left_raw_data, &left_pressure, "pressL") != HAL_OK) {
    left_pressure = last_left_pressure;
  }
  left_temperature = (uint8_t)(left_temperature_temp);

  if (pressure_i2c_read_one(&hi2c2, &right_sensor_status, right_raw_data, &right_pressure, "pressR") != HAL_OK) {
    right_pressure = last_right_pressure;
  }
  right_temperature = (uint8_t)(right_temperature_temp);
}
