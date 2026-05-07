#ifndef USERDRIVERS_BOARD_H
#define USERDRIVERS_BOARD_H

#include "BSP/Gpio/bsp_gpio.h"
#include "BSP/I2c/bsp_i2c.h"
#include "BSP/Spi/bsp_spi.h"
#include "FreeRTOS.h"
#include "semphr.h"
#include "UserDrivers/Ads1248/ads1248.h"
#include "UserDrivers/Ds18b20/ds18b20_driver.h"
#include "UserDrivers/Eeprom24cxx/eeprom24cxx.h"
#include "UserDrivers/PressureSensor/pressure_sensor_driver.h"

#ifdef __cplusplus
extern "C" {
#endif

extern BspI2c gUserEeSoftI2c;
extern BspI2c gUserPressureI2c1;
extern BspI2c gUserPressureI2c2;
extern BspSpi gUserRtdSpi2;

extern UserEeprom24cxx gUserEeprom24c02;
extern UserAds1248 gUserAds1248;
extern UserDs18b20 gUserDs18b20;
extern UserPressureSensor gUserPressureSensor;

void UserDrivers_BoardInit(void);
void UserDrivers_BoardSetAds1248DrdySemaphore(SemaphoreHandle_t sem);

#ifdef __cplusplus
}
#endif

#endif
