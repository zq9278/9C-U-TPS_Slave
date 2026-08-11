#ifndef MODULES_DISPLAY_TREATMENT_OLED_DISPLAY_H
#define MODULES_DISPLAY_TREATMENT_OLED_DISPLAY_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef enum
{
    TREATMENT_OLED_STATUS_STOPPED = 0,
    TREATMENT_OLED_STATUS_HEATING,
    TREATMENT_OLED_STATUS_PAUSED,
    TREATMENT_OLED_STATUS_FAULT
} TreatmentOledState;

typedef struct
{
    TreatmentOledState state;
    float temp_left_c;
    float temp_right_c;
    float target_temp_c;
    uint16_t pwm_left;
    uint16_t pwm_right;
    uint8_t sensor_valid;
    uint8_t left_enabled;
    uint8_t right_enabled;
} TreatmentOledSnapshot;

void TreatmentOledDisplay_Init(void);
void TreatmentOledDisplay_Render(const TreatmentOledSnapshot *snapshot);

#ifdef __cplusplus
}
#endif

#endif
