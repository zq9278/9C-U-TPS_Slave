#ifndef MODULES_HEAT_TREATMENT_HEATING_CONTROL_H
#define MODULES_HEAT_TREATMENT_HEATING_CONTROL_H

#include "treatment_app_controller.h"

#ifdef __cplusplus
extern "C" {
#endif

void TreatmentHeatingControl_InitPid(TreatmentAppController *controller);
void TreatmentHeatingControl_ResetPid(TreatmentAppController *controller);
void TreatmentHeatingControl_UpdateSetpoint(TreatmentAppController *controller);
void TreatmentHeatingControl_ApplyOutputs(TreatmentAppController *controller,
                                         const volatile sensor_data_t *sensor,
                                         float dt_s,
                                         TreatmentAppRuntime *runtime);
void TreatmentHeatingControl_SetPidGains(pid_debug_target_t target, float kp, float ki, float kd);
void TreatmentHeatingControl_GetPidGains(pid_debug_target_t target, float *kp, float *ki, float *kd);
uint32_t TreatmentHeatingControl_GetPidVersion(pid_debug_target_t target);

#ifdef __cplusplus
}
#endif

#endif
