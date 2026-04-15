#include "ads1248_v2.h"
#include "LOG.h"
#include "sensor_task.h"
#include "spi.h"
#include "FreeRTOS.h"
#include "semphr.h"

#define ADS1248V2_CMD_WAKEUP   0x00U
#define ADS1248V2_CMD_SLEEP    0x02U
#define ADS1248V2_CMD_SYNC     0x04U
#define ADS1248V2_CMD_RESET    0x06U
#define ADS1248V2_CMD_START    0x08U
#define ADS1248V2_CMD_STOP     0x0AU
#define ADS1248V2_CMD_RDATA    0x12U
#define ADS1248V2_CMD_SDATAC   0x16U
#define ADS1248V2_CMD_RREG     0x20U
#define ADS1248V2_CMD_WREG     0x40U
#define ADS1248V2_CMD_NOP      0xFFU

#define ADS1248V2_REG_MUX0     0x00U
#define ADS1248V2_REG_MUX1     0x02U
#define ADS1248V2_REG_SYS0     0x03U
#define ADS1248V2_REG_IDAC0    0x0AU
#define ADS1248V2_REG_IDAC1    0x0BU

/* Power-up stabilization:
 * after reset + initial channel selection, give the converter a short settle
 * window and throw away a few start-up samples before normal scanning begins.
 */
#define ADS1248V2_STARTUP_SETTLE_MS    20U
#define ADS1248V2_STARTUP_DISCARD_CNT  3U

/* SYS0 = PGA[6:4] + DR[3:0]
 * Keep PGA = 1 and raise the output data rate to 80 SPS for comparison.
 * This lets us quickly verify whether the current issue is sensitive to the
 * conversion rate configuration.
 */
#define ADS1248V2_SYS0_PGA1_DR80SPS 0x05U

#define ADS1248V2_CS_LOW()     HAL_GPIO_WritePin(RTD_CS_GPIO_Port, RTD_CS_Pin, GPIO_PIN_RESET)
#define ADS1248V2_CS_HIGH()    HAL_GPIO_WritePin(RTD_CS_GPIO_Port, RTD_CS_Pin, GPIO_PIN_SET)
#define ADS1248V2_START_HIGH() HAL_GPIO_WritePin(RTD_START_GPIO_Port, RTD_START_Pin, GPIO_PIN_SET)
#define ADS1248V2_DRDY_IS_LOW() (HAL_GPIO_ReadPin(RTD_RDY_GPIO_Port, RTD_RDY_Pin) == GPIO_PIN_RESET)

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

static const ads1248v2_channel_config_t kChannelConfig[] = {
    /* Schematic mapping:
     * Heat1_NTC2+ -> AIN2, Heat1_NTC2- -> AIN3, reference pair REFP1/REFN1,
     * current excitation routed to IEXC2.
     */
    { 0x13U, 0x28U, ADS1248V2_SYS0_PGA1_DR80SPS, 0x01U, 0x89U },
    /* Schematic mapping:
     * Heat2_NTC2+ -> AIN0, Heat2_NTC2- -> AIN1, reference pair REFP0/REFN0,
     * current excitation routed to IEXC1.
     */
    { 0x01U, 0x20U, ADS1248V2_SYS0_PGA1_DR80SPS, 0x01U, 0x8AU },
};

static ads1248v2_channel_t s_current_channel = ADS1248V2_CHANNEL_HEAT2;

static void ads1248v2_clear_drdy_event(void)
{
    /* Drop stale DRDY edges before waiting for the next conversion after a
     * channel switch or conversion restart.
     */
    if (gRtdDrdySem != NULL) {
        while (xSemaphoreTake(gRtdDrdySem, 0) == pdTRUE) {
        }
    }
}

static bool ads1248v2_tx(const uint8_t *data, uint16_t len)
{
    ADS1248V2_CS_LOW();
    HAL_StatusTypeDef st = HAL_SPI_Transmit(&hspi2, (uint8_t *)data, len, 100U);
    ADS1248V2_CS_HIGH();
    return st == HAL_OK;
}

static bool ads1248v2_txrx(const uint8_t *tx, uint8_t *rx, uint16_t len)
{
    ADS1248V2_CS_LOW();
    HAL_StatusTypeDef st = HAL_SPI_TransmitReceive(&hspi2, (uint8_t *)tx, rx, len, 100U);
    ADS1248V2_CS_HIGH();
    return st == HAL_OK;
}

static bool ads1248v2_send_command(uint8_t cmd)
{
    bool ok = ads1248v2_tx(&cmd, 1U);
    HAL_Delay(1);
    return ok;
}

static bool ads1248v2_write_register(uint8_t reg, uint8_t value)
{
    uint8_t cmd[3] = { (uint8_t)(ADS1248V2_CMD_WREG | reg), 0x00U, value };
    return ads1248v2_tx(cmd, 3U);
}

static uint16_t ads1248v2_interp(uint16_t r_start, uint16_t r_end, uint16_t t_start, uint16_t t_end, uint16_t resistance)
{
    int32_t delta_r = (int32_t)r_start - (int32_t)resistance;
    int32_t delta_t = (int32_t)t_end - (int32_t)t_start;
    int32_t delta_res = (int32_t)r_start - (int32_t)r_end;
    if (delta_res == 0) {
        return t_start;
    }
    return (uint16_t)(t_start + (delta_r * delta_t) / delta_res);
}

void ADS1248V2_Reset(void)
{
    ADS1248V2_START_HIGH();
    HAL_Delay(2);
    (void)ads1248v2_send_command(ADS1248V2_CMD_RESET);
    HAL_Delay(5);
    (void)ads1248v2_send_command(ADS1248V2_CMD_SDATAC);
}

void ADS1248V2_Init(void)
{
    uint32_t raw_discard = 0;

    ADS1248V2_Reset();
    if (!ADS1248V2_SelectChannel(ADS1248V2_CHANNEL_HEAT2)) {
        return;
    }

    /* Startup settle stage:
     * give the analog front-end a short time to settle, then discard the
     * first few conversions so the normal scan starts from a cleaner state.
     */
    HAL_Delay(ADS1248V2_STARTUP_SETTLE_MS);
    for (uint32_t i = 0; i < ADS1248V2_STARTUP_DISCARD_CNT; ++i) {
        if (!ADS1248V2_WaitDrdy(ADS1248V2_DRDY_TIMEOUT_MS)) {
            break;
        }
        if (!ADS1248V2_ReadRaw(&raw_discard)) {
            break;
        }
    }
}

bool ADS1248V2_InitSingleChannel(ads1248v2_channel_t channel)
{
    /* Diagnostic init path:
     * reset the ADC once, configure one channel once, then leave the
     * conversion chain running so the caller only has to wait/read.
     */
    ADS1248V2_Reset();
    return ADS1248V2_SelectChannel(channel);
}

bool ADS1248V2_SelectChannel(ads1248v2_channel_t channel)
{
    const ads1248v2_channel_config_t *cfg;

    if ((uint32_t)channel >= (sizeof(kChannelConfig) / sizeof(kChannelConfig[0]))) {
        return false;
    }

    cfg = &kChannelConfig[channel];

    /* Align the v2 driver with the legacy timing sequence first:
     * put the ADC into SDATAC, stop the current conversion, rewrite the
     * selected channel registers, then resync and restart conversion.
     * We can simplify this again later after the conversion chain is stable.
     */
    if (!ads1248v2_send_command(ADS1248V2_CMD_SDATAC)) return false;
    if (!ads1248v2_send_command(ADS1248V2_CMD_STOP))   return false;
    if (!ads1248v2_write_register(ADS1248V2_REG_MUX0,  cfg->mux0))  return false;
    if (!ads1248v2_write_register(ADS1248V2_REG_MUX1,  cfg->mux1))  return false;
    if (!ads1248v2_write_register(ADS1248V2_REG_SYS0,  cfg->sys0))  return false;
    if (!ads1248v2_write_register(ADS1248V2_REG_IDAC0, cfg->idac0)) return false;
    if (!ads1248v2_write_register(ADS1248V2_REG_IDAC1, cfg->idac1)) return false;
    if (!ads1248v2_send_command(ADS1248V2_CMD_SYNC))   return false;
    if (!ads1248v2_send_command(ADS1248V2_CMD_WAKEUP)) return false;
    if (!ads1248v2_send_command(ADS1248V2_CMD_START))  return false;

    ads1248v2_clear_drdy_event();
    s_current_channel = channel;
    return true;
}

bool ADS1248V2_WaitDrdy(uint32_t timeout_ms)
{
    /* If DRDY is already low when we arrive here, the conversion is ready
     * and we can use the sample immediately.
     */
    if (ADS1248V2_DRDY_IS_LOW()) {
        return true;
    }

    /* Otherwise wait for the EXTI falling edge driven by RTD_RDY. */
    if ((gRtdDrdySem != NULL) &&
        (xSemaphoreTake(gRtdDrdySem, pdMS_TO_TICKS(timeout_ms)) == pdTRUE) &&
        ADS1248V2_DRDY_IS_LOW()) {
        return true;
    }

    LOG_W("[RTD2] drdy timeout pin=%u timeout_ms=%lu",
          (unsigned)HAL_GPIO_ReadPin(RTD_RDY_GPIO_Port, RTD_RDY_Pin),
          (unsigned long)timeout_ms);
    return false;
}

bool ADS1248V2_ReadRaw(uint32_t *raw_out)
{
    uint8_t tx[5] = { ADS1248V2_CMD_RDATA, ADS1248V2_CMD_NOP, ADS1248V2_CMD_NOP, ADS1248V2_CMD_NOP, ADS1248V2_CMD_NOP };
    uint8_t rx[5] = {0};

    if (raw_out == NULL) {
        return false;
    }
    if (!ads1248v2_txrx(tx, rx, 5U)) {
        return false;
    }

    *raw_out = ((uint32_t)rx[1] << 16) |
               ((uint32_t)rx[2] << 8)  |
               (uint32_t)rx[3];
    return true;
}

bool ADS1248V2_ReadRegister(uint8_t reg, uint8_t *value_out)
{
    uint8_t tx[3] = { (uint8_t)(ADS1248V2_CMD_RREG | reg), 0x00U, ADS1248V2_CMD_NOP };
    uint8_t rx[3] = {0};

    if (value_out == NULL) {
        return false;
    }
    if (!ads1248v2_txrx(tx, rx, 3U)) {
        return false;
    }

    *value_out = rx[2];
    return true;
}

bool ADS1248V2_CodeToTemperatureC(uint32_t raw_code, float *temp_c_out)
{
    float resistance;
    uint16_t temp_x100;

    if ((temp_c_out == NULL) || (raw_code == 0U) || (raw_code == 0xFFFFFFU) || (raw_code == 0x7FFFFFU)) {
        return false;
    }

    resistance = ((float)raw_code * ADS1248V2_RREF_OHM) / ADS1248V2_ADC_FULL_SCALE;

    if (resistance > 8301.0f) {
        int32_t idx = (int32_t)((27445.0f - resistance) / 796.0f);
        /* Keep the same boundary behavior as the legacy driver:
         * slightly out-of-range cold-side values are clamped to the nearest
         * table endpoint instead of being treated as a hard conversion error.
         */
        if (idx >= ((int32_t)(sizeof(kTempTableCold) / sizeof(kTempTableCold[0])) - 1)) {
            temp_x100 = kTempTableCold[(sizeof(kTempTableCold) / sizeof(kTempTableCold[0])) - 1];
        } else if (idx < 0) {
            return false;
        } else {
            temp_x100 = ads1248v2_interp((uint16_t)(27445 - idx * 796),
                                         (uint16_t)(27445 - idx * 796 - 796),
                                         kTempTableCold[idx],
                                         kTempTableCold[idx + 1],
                                         (uint16_t)resistance);
        }
    } else if (resistance >= 4170.0f) {
        int32_t idx = (int32_t)((resistance - 4170.0f) / 27.0f);
        if (idx < 0) {
            idx = 0;
        }
        if (idx >= ((int32_t)(sizeof(kTempTableMid) / sizeof(kTempTableMid[0])) - 1)) {
            idx = (int32_t)(sizeof(kTempTableMid) / sizeof(kTempTableMid[0])) - 2;
        }
        temp_x100 = ads1248v2_interp((uint16_t)(4170 + idx * 27),
                                     (uint16_t)(4170 + idx * 27 + 27),
                                     kTempTableMid[idx],
                                     kTempTableMid[idx + 1],
                                     (uint16_t)resistance);
    } else {
        int32_t idx = (int32_t)((4170.0f - resistance) / 132.0f);
        if (idx < 0) {
            idx = 0;
        }
        if (idx >= ((int32_t)(sizeof(kTempTableHot) / sizeof(kTempTableHot[0])) - 1)) {
            idx = (int32_t)(sizeof(kTempTableHot) / sizeof(kTempTableHot[0])) - 2;
        }
        temp_x100 = ads1248v2_interp((uint16_t)(4170 - idx * 132),
                                     (uint16_t)(4170 - idx * 132 - 132),
                                     kTempTableHot[idx],
                                     kTempTableHot[idx + 1],
                                     (uint16_t)resistance);
    }

    *temp_c_out = ((float)temp_x100) / 100.0f;
    return (*temp_c_out > -20.0f) && (*temp_c_out < 100.0f);
}

bool ADS1248V2_ReadTemperatureC(ads1248v2_channel_t channel, float *temp_c_out, uint32_t *raw_out)
{
    uint32_t raw_value = 0;
    float temp_c = ADS1248V2_INVALID_TEMP_C;
    uint8_t mux0_read = 0U;
    uint8_t mux1_read = 0U;
    uint8_t idac0_read = 0U;
    uint8_t idac1_read = 0U;

    if (temp_c_out == NULL) {
        return false;
    }

    /* Only switch when the requested channel changes. If switching fails,
     * log the exact channel so the host can see which path is broken.
     */
    if ((channel != s_current_channel) && !ADS1248V2_SelectChannel(channel)) {
        LOG_W("[RTD2] select channel failed: ch=%u", (unsigned)channel);
        return false;
    }

    /* Test path:
     * do not discard the first conversion after channel switching.
     * This lets us verify whether the current异常点和“丢首样再等第二拍”有关。
     */
    if (!ADS1248V2_WaitDrdy(ADS1248V2_DRDY_TIMEOUT_MS)) {
        (void)ADS1248V2_ReadRegister(ADS1248V2_REG_MUX0, &mux0_read);
        (void)ADS1248V2_ReadRegister(ADS1248V2_REG_MUX1, &mux1_read);
        (void)ADS1248V2_ReadRegister(ADS1248V2_REG_IDAC0, &idac0_read);
        (void)ADS1248V2_ReadRegister(ADS1248V2_REG_IDAC1, &idac1_read);
        LOG_W("[RTD2] wait drdy timeout(data): ch=%u pin=%u mux0=0x%02X mux1=0x%02X idac0=0x%02X idac1=0x%02X",
              (unsigned)channel,
              (unsigned)HAL_GPIO_ReadPin(RTD_RDY_GPIO_Port, RTD_RDY_Pin),
              (unsigned)mux0_read,
              (unsigned)mux1_read,
              (unsigned)idac0_read,
              (unsigned)idac1_read);
        return false;
    }
    if (!ADS1248V2_ReadRaw(&raw_value)) {
        LOG_W("[RTD2] read raw failed(data): ch=%u", (unsigned)channel);
        return false;
    }

    if (raw_out != NULL) {
        *raw_out = raw_value;
    }

    if (!ADS1248V2_CodeToTemperatureC(raw_value, &temp_c)) {
        LOG_W("[RTD2] convert failed: ch=%u raw=0x%06lX",
              (unsigned)channel,
              (unsigned long)raw_value);
        return false;
    }

    *temp_c_out = temp_c;
    return true;
}
