#ifndef MODULES_PID_CONTROLLER_H
#define MODULES_PID_CONTROLLER_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef enum
{
    PID_CONTROLLER_DERIVATIVE_ON_MEASUREMENT = 0,
    PID_CONTROLLER_DERIVATIVE_ON_ERROR = 1
} PidControllerDerivativeMode;

typedef struct
{
    float kp;
    float ki;
    float kd;
    float setpoint;
    float output_min;
    float output_max;
    float integral_min;
    float integral_max;
    float default_dt_s;
    float derivative_filter_alpha;
    PidControllerDerivativeMode derivative_mode;
} PidControllerConfig;

typedef struct
{
    float p;
    float i;
    float d;
    float error;
    float integral;
    float derivative;
    float setpoint;
    float measurement;
    float output;
    float dt_s;
    uint8_t output_limited;
} PidControllerDebug;

typedef struct
{
    PidControllerConfig config;
    PidControllerDebug debug;
    float previous_error;
    float previous_measurement;
    float derivative_filtered;
    float integral;
    uint8_t has_previous;
} PidController;

void PidController_Init(PidController *pid, const PidControllerConfig *config);
void PidController_InitDefaults(PidController *pid);
void PidController_Reset(PidController *pid);

void PidController_SetGains(PidController *pid, float kp, float ki, float kd);
void PidController_SetSetpoint(PidController *pid, float setpoint);
void PidController_SetOutputLimits(PidController *pid, float min_value, float max_value);
void PidController_SetIntegralLimits(PidController *pid, float min_value, float max_value);
void PidController_SetDefaultDt(PidController *pid, float default_dt_s);
void PidController_SetDerivativeFilterAlpha(PidController *pid, float alpha);
void PidController_SetDerivativeMode(PidController *pid, PidControllerDerivativeMode mode);

float PidController_Compute(PidController *pid, float measurement);
float PidController_ComputeDt(PidController *pid, float measurement, float dt_s);

#ifdef __cplusplus
}
#endif

#endif
