#ifndef ADS1248_V2_H
#define ADS1248_V2_H

#include "main.h"
#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#define ADS1248V2_RREF_OHM              30000.0f
#define ADS1248V2_ADC_FULL_SCALE        8388608.0f
#define ADS1248V2_INVALID_TEMP_C        (-1000.0f)
/* At the slower diagnostic data rates we allow a wider ready timeout so the
 * task does not misclassify a legitimate slow conversion as a hard failure.
 */
#define ADS1248V2_DRDY_TIMEOUT_MS       250U

typedef enum {
    /* Heat1 NTC2:
     *   AIN2/AIN3
     *   REFP1/REFN1
     *   excitation sourced from IEXC2
     */
    ADS1248V2_CHANNEL_HEAT1 = 0,
    /* Heat2 NTC2:
     *   AIN0/AIN1
     *   REFP0/REFN0
     *   excitation sourced from IEXC1
     */
    ADS1248V2_CHANNEL_HEAT2 = 1,
} ads1248v2_channel_t;

typedef struct {
    uint8_t mux0;
    uint8_t mux1;
    uint8_t sys0;
    uint8_t idac0;
    uint8_t idac1;
} ads1248v2_channel_config_t;

void ADS1248V2_Init(void);
/* Initialize ADS1248 once and lock it to a single channel for diagnostics.
 * After this call, the caller can repeatedly wait for DRDY and read raw data
 * without any runtime channel switching.
 */
bool ADS1248V2_InitSingleChannel(ads1248v2_channel_t channel);
void ADS1248V2_Reset(void);
bool ADS1248V2_SelectChannel(ads1248v2_channel_t channel);
bool ADS1248V2_WaitDrdy(uint32_t timeout_ms);
bool ADS1248V2_ReadRaw(uint32_t *raw_out);
bool ADS1248V2_ReadRegister(uint8_t reg, uint8_t *value_out);
bool ADS1248V2_ReadTemperatureC(ads1248v2_channel_t channel, float *temp_c_out, uint32_t *raw_out);
bool ADS1248V2_CodeToTemperatureC(uint32_t raw_code, float *temp_c_out);

#ifdef __cplusplus
}
#endif

#endif
