#ifndef MODULES_PRESSURE_TREATMENT_PRESSURE_CONTROL_H
#define MODULES_PRESSURE_TREATMENT_PRESSURE_CONTROL_H

#include "treatment_app_controller.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef struct
{
    TreatmentPhase phase;
    TreatmentPressurePidStage pid_stage;
    uint8_t inflating;
    float target_pressure_kpa;
    uint32_t cycle_elapsed_ms;
} TreatmentPressurePlan;

void TreatmentPressureControl_InitPid(TreatmentAppController *controller);
void TreatmentPressureControl_ResetPid(TreatmentAppController *controller);
void TreatmentPressureControl_BuildPlan(const TreatmentAppController *controller,
                                        TickType_t now_tick,
                                        TreatmentPressurePlan *plan);
void TreatmentPressureControl_ApplyPlan(TreatmentAppController *controller,
                                        const volatile sensor_data_t *sensor,
                                        float dt_s,
                                        const TreatmentPressurePlan *plan,
                                        TreatmentAppRuntime *runtime);
const char *TreatmentPressureControl_PhaseName(TreatmentPhase phase);
void TreatmentPressureControl_SetPidGainsForController(const TreatmentAppController *controller,
                                                       TreatmentPressurePidStage stage,
                                                       float kp,
                                                       float ki,
                                                       float kd);
void TreatmentPressureControl_GetPidGainsForController(const TreatmentAppController *controller,
                                                       TreatmentPressurePidStage stage,
                                                       float *kp,
                                                       float *ki,
                                                       float *kd);
void TreatmentPressureControl_SetPidGains(TreatmentPressurePidStage stage, float kp, float ki, float kd);
void TreatmentPressureControl_GetPidGains(TreatmentPressurePidStage stage, float *kp, float *ki, float *kd);
uint32_t TreatmentPressureControl_GetPidVersion(void);

#ifdef __cplusplus
}
#endif

#endif
