#ifndef COMPONENTS_ACTUATORS_TREATMENT_ACTUATORS_H
#define COMPONENTS_ACTUATORS_TREATMENT_ACTUATORS_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef enum
{
    TREATMENT_SIDE_LEFT = 0,
    TREATMENT_SIDE_RIGHT = 1
} TreatmentSide;

void TreatmentActuators_Init(void);
void TreatmentActuators_SetIdle(void);
void TreatmentActuators_ApplyPressureRoute(uint8_t enable_left, uint8_t enable_right);
void TreatmentActuators_SetPressureVentAll(void);
void TreatmentActuators_SetWaveValve(uint8_t enabled);
void TreatmentActuators_SetPumpPwm(uint16_t pwm);
void TreatmentActuators_SetHeaterPower(TreatmentSide side, uint8_t enabled);
void TreatmentActuators_SetHeaterPwm(TreatmentSide side, uint16_t pwm);
void TreatmentActuators_ResetHeaterOtp(TreatmentSide side);

#ifdef __cplusplus
}
#endif

#endif
