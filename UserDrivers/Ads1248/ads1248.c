#include "ads1248.h"

#include <stddef.h>

#define ADS1248_CMD_WAKEUP   0x00U
#define ADS1248_CMD_SYNC     0x04U
#define ADS1248_CMD_RESET    0x06U
#define ADS1248_CMD_START    0x08U
#define ADS1248_CMD_STOP     0x0AU
#define ADS1248_CMD_RDATA    0x12U
#define ADS1248_CMD_SDATAC   0x16U
#define ADS1248_CMD_RREG     0x20U
#define ADS1248_CMD_WREG     0x40U
#define ADS1248_CMD_NOP      0xFFU

#define ADS1248_REG_MUX0     0x00U
#define ADS1248_REG_MUX1     0x02U
#define ADS1248_REG_SYS0     0x03U
#define ADS1248_REG_IDAC0    0x0AU
#define ADS1248_REG_IDAC1    0x0BU

#define ADS1248_STARTUP_SETTLE_MS    20U
#define ADS1248_STARTUP_DISCARD_CNT  3U
#define ADS1248_SYS0_PGA1_DR80SPS    0x05U

static const uint16_t kTempTableHot[] = {
    5000, 5101, 5206, 5314, 5428, 5546, 5670, 5800, 5937, 6080,
    6232, 6393, 6563, 6744, 6938, 7144, 7368, 7610, 7871, 8159,
    8477, 8828, 9226, 9679, 10200
};

static const uint16_t kTempTableMid[] = {
    4993, 4973, 4953, 4933, 4914, 4895, 4875, 4856, 4837, 4819, 4800, 4782, 4763, 4745, 4727, 4709,
    4691, 4673, 4656, 4638, 4621, 4604, 4587, 4570, 4553, 4536, 4519, 4503, 4486, 4470, 4454, 4438,
    4421, 4406, 4390, 4374, 4358, 4343, 4327, 4312, 4297, 4282, 4267, 4252, 4237, 4222, 4207, 4193,
    4178, 4164, 4149, 4135, 4121, 4107, 4093, 4079, 4065, 4051, 4037, 4024, 4010, 3996, 3983, 3970,
    3956, 3943, 3930, 3917, 3904, 3891, 3878, 3865, 3853, 3840, 3827, 3815, 3802, 3790, 3778, 3765,
    3753, 3741, 3729, 3717, 3705, 3693, 3681, 3669, 3658, 3646, 3634, 3623, 3611, 3600, 3588, 3577,
    3566, 3555, 3543, 3532, 3521, 3510, 3499, 3488, 3477, 3467, 3456, 3445, 3434, 3424, 3413, 3403,
    3392, 3382, 3371, 3361, 3351, 3340, 3330, 3320, 3310, 3300, 3290, 3280, 3270, 3260, 3250, 3240,
    3230, 3220, 3211, 3201, 3191, 3182, 3172, 3163, 3153, 3144, 3134, 3125, 3116, 3107, 3097, 3088,
    3079, 3070, 3061, 3052, 3043, 3034, 3025, 3016, 3007, 2998
};

static const uint16_t kTempTableCold[] = {
    62, 127, 195, 266, 339, 414, 493, 576, 661, 751,
    845, 943, 1047, 1156, 1271, 1393, 1522, 1660, 1808, 1966,
    2137, 2323, 2526, 2748, 2995
};

static const UserAds1248ChannelConfig kChannelConfig[] = {
    /* Heat1: AIN2/AIN3 + REFP1/REFN1, excitation routed only to IEXC2. */
    { 0x13U, 0x28U, ADS1248_SYS0_PGA1_DR80SPS, 0x01U, 0xF9U },
    /* Heat2: AIN0/AIN1 + REFP0/REFN0, excitation routed only to IEXC1. */
    { 0x01U, 0x20U, ADS1248_SYS0_PGA1_DR80SPS, 0x01U, 0xF8U },
};

static void AdsDelay(UserAds1248 *dev, uint32_t ms)
{
    if ((dev != NULL) && (dev->delay_ms != NULL))
    {
        dev->delay_ms(ms);
    }
}

static uint8_t AdsSendCommand(UserAds1248 *dev, uint8_t cmd)
{
    uint8_t ok;

    if ((dev == NULL) || (dev->spi == NULL))
    {
        return 0U;
    }

    ok = BspSpi_Transmit(dev->spi, &cmd, 1U, 100U);
    AdsDelay(dev, 1U);
    return ok;
}

static uint8_t AdsWriteRegister(UserAds1248 *dev, uint8_t reg, uint8_t value)
{
    uint8_t tx[3];

    if ((dev == NULL) || (dev->spi == NULL))
    {
        return 0U;
    }

    tx[0] = (uint8_t)(ADS1248_CMD_WREG | reg);
    tx[1] = 0x00U;
    tx[2] = value;
    return BspSpi_Transmit(dev->spi, tx, 3U, 100U);
}

static uint16_t AdsInterp(uint16_t r_start,
                          uint16_t r_end,
                          uint16_t t_start,
                          uint16_t t_end,
                          uint16_t resistance)
{
    int32_t delta_r = (int32_t)r_start - (int32_t)resistance;
    int32_t delta_t = (int32_t)t_end - (int32_t)t_start;
    int32_t delta_res = (int32_t)r_start - (int32_t)r_end;

    if (delta_res == 0)
    {
        return t_start;
    }

    return (uint16_t)(t_start + (delta_r * delta_t) / delta_res);
}

static uint8_t AdsReadTemperatureFromCurrentChannel(UserAds1248 *dev,
                                                    float *temp_c_out,
                                                    uint32_t *raw_out)
{
    uint32_t raw_value = 0U;

    if ((dev == NULL) || (temp_c_out == NULL))
    {
        return 0U;
    }

    if ((UserAds1248_WaitDrdy(dev, USER_ADS1248_DRDY_TIMEOUT_MS) == 0U) ||
        (UserAds1248_ReadRaw(dev, &raw_value) == 0U))
    {
        return 0U;
    }

    if (raw_out != NULL)
    {
        *raw_out = raw_value;
    }

    return UserAds1248_CodeToTemperatureC(raw_value, temp_c_out);
}

void UserAds1248_InitDevice(UserAds1248 *dev,
                            BspSpi *spi,
                            BspGpioPin *start_pin,
                            BspGpioPin *drdy_pin,
                            SemaphoreHandle_t drdy_sem,
                            UserAds1248DelayMs delay_ms)
{
    if (dev == NULL)
    {
        return;
    }

    dev->spi = spi;
    dev->start_pin = start_pin;
    dev->drdy_pin = drdy_pin;
    dev->drdy_sem = drdy_sem;
    dev->delay_ms = delay_ms;
    dev->current_channel = USER_ADS1248_CHANNEL_HEAT2;
}

void UserAds1248_SetDrdySemaphore(UserAds1248 *dev, SemaphoreHandle_t drdy_sem)
{
    if (dev != NULL)
    {
        dev->drdy_sem = drdy_sem;
    }
}

void UserAds1248_Reset(UserAds1248 *dev)
{
    if (dev == NULL)
    {
        return;
    }

    if (dev->start_pin != NULL)
    {
        BspGpio_Set(dev->start_pin);
    }
    AdsDelay(dev, 2U);
    (void)AdsSendCommand(dev, ADS1248_CMD_RESET);
    AdsDelay(dev, 5U);
    (void)AdsSendCommand(dev, ADS1248_CMD_SDATAC);
}

uint8_t UserAds1248_Init(UserAds1248 *dev)
{
    uint32_t raw_discard = 0U;
    uint32_t i;

    UserAds1248_Reset(dev);
    if (UserAds1248_SelectChannel(dev, USER_ADS1248_CHANNEL_HEAT2) == 0U)
    {
        return 0U;
    }

    AdsDelay(dev, ADS1248_STARTUP_SETTLE_MS);
    for (i = 0U; i < ADS1248_STARTUP_DISCARD_CNT; ++i)
    {
        if ((UserAds1248_WaitDrdy(dev, USER_ADS1248_DRDY_TIMEOUT_MS) == 0U) ||
            (UserAds1248_ReadRaw(dev, &raw_discard) == 0U))
        {
            break;
        }
    }

    return 1U;
}

uint8_t UserAds1248_SelectChannel(UserAds1248 *dev, UserAds1248Channel channel)
{
    const UserAds1248ChannelConfig *cfg;

    if ((dev == NULL) || ((uint32_t)channel >= (sizeof(kChannelConfig) / sizeof(kChannelConfig[0]))))
    {
        return 0U;
    }

    cfg = &kChannelConfig[channel];
    if (AdsSendCommand(dev, ADS1248_CMD_SDATAC) == 0U) return 0U;
    if (AdsSendCommand(dev, ADS1248_CMD_STOP) == 0U) return 0U;
    if (AdsWriteRegister(dev, ADS1248_REG_MUX0, cfg->mux0) == 0U) return 0U;
    if (AdsWriteRegister(dev, ADS1248_REG_MUX1, cfg->mux1) == 0U) return 0U;
    if (AdsWriteRegister(dev, ADS1248_REG_SYS0, cfg->sys0) == 0U) return 0U;
    if (AdsWriteRegister(dev, ADS1248_REG_IDAC0, cfg->idac0) == 0U) return 0U;
    if (AdsWriteRegister(dev, ADS1248_REG_IDAC1, cfg->idac1) == 0U) return 0U;
    if (AdsSendCommand(dev, ADS1248_CMD_SYNC) == 0U) return 0U;
    if (AdsSendCommand(dev, ADS1248_CMD_WAKEUP) == 0U) return 0U;
    if (AdsSendCommand(dev, ADS1248_CMD_START) == 0U) return 0U;

    if (dev->drdy_sem != NULL)
    {
        while (xSemaphoreTake(dev->drdy_sem, 0) == pdTRUE)
        {
        }
    }
    dev->current_channel = channel;
    return 1U;
}

uint8_t UserAds1248_WaitDrdy(UserAds1248 *dev, uint32_t timeout_ms)
{
    if ((dev == NULL) || (dev->drdy_pin == NULL))
    {
        return 0U;
    }

    if (BspGpio_Read(dev->drdy_pin) == GPIO_PIN_RESET)
    {
        return 1U;
    }

    if ((dev->drdy_sem != NULL) &&
        (xSemaphoreTake(dev->drdy_sem, pdMS_TO_TICKS(timeout_ms)) == pdTRUE) &&
        (BspGpio_Read(dev->drdy_pin) == GPIO_PIN_RESET))
    {
        return 1U;
    }

    return 0U;
}

uint8_t UserAds1248_ReadRaw(UserAds1248 *dev, uint32_t *raw_out)
{
    uint8_t tx[5] = { ADS1248_CMD_RDATA, ADS1248_CMD_NOP, ADS1248_CMD_NOP, ADS1248_CMD_NOP, ADS1248_CMD_NOP };
    uint8_t rx[5] = {0};

    if ((dev == NULL) || (dev->spi == NULL) || (raw_out == NULL))
    {
        return 0U;
    }
    if (BspSpi_TransmitReceive(dev->spi, tx, rx, 5U, 100U) == 0U)
    {
        return 0U;
    }

    *raw_out = ((uint32_t)rx[1] << 16) | ((uint32_t)rx[2] << 8) | (uint32_t)rx[3];
    return 1U;
}

uint8_t UserAds1248_ReadRegister(UserAds1248 *dev, uint8_t reg, uint8_t *value_out)
{
    uint8_t tx[3] = { (uint8_t)(ADS1248_CMD_RREG | reg), 0x00U, ADS1248_CMD_NOP };
    uint8_t rx[3] = {0};

    if ((dev == NULL) || (dev->spi == NULL) || (value_out == NULL))
    {
        return 0U;
    }
    if (BspSpi_TransmitReceive(dev->spi, tx, rx, 3U, 100U) == 0U)
    {
        return 0U;
    }

    *value_out = rx[2];
    return 1U;
}

uint8_t UserAds1248_CodeToTemperatureC(uint32_t raw_code, float *temp_c_out)
{
    float resistance;
    uint16_t temp_x100;

    if ((temp_c_out == NULL) || (raw_code == 0U) || (raw_code == 0xFFFFFFU) || (raw_code == 0x7FFFFFU))
    {
        return 0U;
    }

    resistance = ((float)raw_code * USER_ADS1248_RREF_OHM) / USER_ADS1248_ADC_FULL_SCALE;

    if (resistance > 8301.0f)
    {
        int32_t idx = (int32_t)((27445.0f - resistance) / 796.0f);
        if (idx >= ((int32_t)(sizeof(kTempTableCold) / sizeof(kTempTableCold[0])) - 1))
        {
            temp_x100 = kTempTableCold[(sizeof(kTempTableCold) / sizeof(kTempTableCold[0])) - 1];
        }
        else if (idx < 0)
        {
            return 0U;
        }
        else
        {
            temp_x100 = AdsInterp((uint16_t)(27445 - idx * 796),
                                  (uint16_t)(27445 - idx * 796 - 796),
                                  kTempTableCold[idx],
                                  kTempTableCold[idx + 1],
                                  (uint16_t)resistance);
        }
    }
    else if (resistance >= 4170.0f)
    {
        int32_t idx = (int32_t)((resistance - 4170.0f) / 27.0f);
        if (idx < 0) idx = 0;
        if (idx >= ((int32_t)(sizeof(kTempTableMid) / sizeof(kTempTableMid[0])) - 1))
        {
            idx = (int32_t)(sizeof(kTempTableMid) / sizeof(kTempTableMid[0])) - 2;
        }
        temp_x100 = AdsInterp((uint16_t)(4170 + idx * 27),
                              (uint16_t)(4170 + idx * 27 + 27),
                              kTempTableMid[idx],
                              kTempTableMid[idx + 1],
                              (uint16_t)resistance);
    }
    else
    {
        int32_t idx = (int32_t)((4170.0f - resistance) / 132.0f);
        if (idx < 0) idx = 0;
        if (idx >= ((int32_t)(sizeof(kTempTableHot) / sizeof(kTempTableHot[0])) - 1))
        {
            idx = (int32_t)(sizeof(kTempTableHot) / sizeof(kTempTableHot[0])) - 2;
        }
        temp_x100 = AdsInterp((uint16_t)(4170 - idx * 132),
                              (uint16_t)(4170 - idx * 132 - 132),
                              kTempTableHot[idx],
                              kTempTableHot[idx + 1],
                              (uint16_t)resistance);
    }

    *temp_c_out = ((float)temp_x100) / 100.0f;
    return (uint8_t)((*temp_c_out > -20.0f) && (*temp_c_out < 100.0f));
}

uint8_t UserAds1248_ReadTemperatureC(UserAds1248 *dev,
                                     UserAds1248Channel channel,
                                     float *temp_c_out,
                                     uint32_t *raw_out)
{    
    if ((dev == NULL) || (temp_c_out == NULL))
    {
        return 0U;
    }

    if ((channel != dev->current_channel) && (UserAds1248_SelectChannel(dev, channel) == 0U))
    {
        return 0U;
    }

    return AdsReadTemperatureFromCurrentChannel(dev, temp_c_out, raw_out);
}

