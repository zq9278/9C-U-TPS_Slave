#ifndef USERDRIVERS_ADS1248_H
#define USERDRIVERS_ADS1248_H

#include <stdint.h>
#include "BSP/Gpio/bsp_gpio.h"
#include "BSP/Spi/bsp_spi.h"
#include "FreeRTOS.h"
#include "semphr.h"

#ifdef __cplusplus
extern "C" {
#endif

#define USER_ADS1248_RREF_OHM        30000.0f
#define USER_ADS1248_ADC_FULL_SCALE  8388608.0f
#define USER_ADS1248_INVALID_TEMP_C  (-1000.0f)
#define USER_ADS1248_DRDY_TIMEOUT_MS 250U

typedef enum
{
    USER_ADS1248_CHANNEL_HEAT1 = 0,
    USER_ADS1248_CHANNEL_HEAT2 = 1
} UserAds1248Channel;

typedef void (*UserAds1248DelayMs)(uint32_t ms);

typedef struct
{
    uint8_t mux0;
    uint8_t mux1;
    uint8_t sys0;
    uint8_t idac0;
    uint8_t idac1;
} UserAds1248ChannelConfig;

typedef struct
{
    BspSpi *spi;
    BspGpioPin *start_pin;
    BspGpioPin *drdy_pin;
    SemaphoreHandle_t drdy_sem;
    UserAds1248DelayMs delay_ms;
    UserAds1248Channel current_channel;
} UserAds1248;

void UserAds1248_InitDevice(UserAds1248 *dev,
                            BspSpi *spi,
                            BspGpioPin *start_pin,
                            BspGpioPin *drdy_pin,
                            SemaphoreHandle_t drdy_sem,
                            UserAds1248DelayMs delay_ms);
void UserAds1248_SetDrdySemaphore(UserAds1248 *dev, SemaphoreHandle_t drdy_sem);
void UserAds1248_Reset(UserAds1248 *dev);
uint8_t UserAds1248_Init(UserAds1248 *dev);
uint8_t UserAds1248_InitSingleChannel(UserAds1248 *dev, UserAds1248Channel channel);
uint8_t UserAds1248_SelectChannel(UserAds1248 *dev, UserAds1248Channel channel);
uint8_t UserAds1248_WaitDrdy(UserAds1248 *dev, uint32_t timeout_ms);
uint8_t UserAds1248_ReadRaw(UserAds1248 *dev, uint32_t *raw_out);
uint8_t UserAds1248_ReadRegister(UserAds1248 *dev, uint8_t reg, uint8_t *value_out);
uint8_t UserAds1248_CodeToTemperatureC(uint32_t raw_code, float *temp_c_out);
/*
 * Dual-channel mode API:
 * caller may alternate channel 0/1 outside, and this function will switch
 * ADS1248 channel when needed before waiting/reading one conversion.
 */
uint8_t UserAds1248_ReadTemperatureC(UserAds1248 *dev,
                                     UserAds1248Channel channel,
                                     float *temp_c_out,
                                     uint32_t *raw_out);
/*
 * Single-channel mode API:
 * lock ADS1248 to one channel first, then keep reading the same channel.
 * If current channel differs, the helper will re-init to the requested
 * single channel once and then read the conversion result.
 */
uint8_t UserAds1248_ReadSingleChannelTemperatureC(UserAds1248 *dev,
                                                  UserAds1248Channel channel,
                                                  float *temp_c_out,
                                                  uint32_t *raw_out);

#ifdef __cplusplus
}
#endif

#endif
