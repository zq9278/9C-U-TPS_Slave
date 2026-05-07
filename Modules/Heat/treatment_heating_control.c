#include "treatment_heating_control.h"

#include "TreatmentActuators.h"

#define HEAT_PID_DEFAULT_DT_S 0.002f

static float s_heat_left_kp = 0.001f;
static float s_heat_left_ki = 0.0000f;
static float s_heat_left_kd = 0.0000f;
static float s_heat_right_kp = 0.001f;
static float s_heat_right_ki = 0.0000f;
static float s_heat_right_kd = 0.0000f;
static uint32_t s_heat_left_pid_version = 0U;
static uint32_t s_heat_right_pid_version = 0U;

static uint16_t TreatmentHeatingControl_ClampPwm(float value, uint16_t max_value)
{
    if (value <= 0.0f)
    {
        return 0U;
    }
    if (value >= (float)max_value)
    {
        return max_value;
    }
    return (uint16_t)value;
}

static void TreatmentHeatingControl_LoadPid(TreatmentAppController *controller,
                                            pid_debug_target_t target)
{
    if (controller == NULL)
    {
        return;
    }

    switch (target)
    {
    case PID_DEBUG_TARGET_HEAT_LEFT:
        PidController_SetGains(&controller->heat_left_pid,
                               s_heat_left_kp,
                               s_heat_left_ki,
                               s_heat_left_kd);
        PidController_Reset(&controller->heat_left_pid);
        PidController_SetSetpoint(&controller->heat_left_pid,
                                  controller->cfg.temp_target * 100.0f);
        controller->active_heat_left_profile_version = s_heat_left_pid_version;
        break;

    case PID_DEBUG_TARGET_HEAT_RIGHT:
        PidController_SetGains(&controller->heat_right_pid,
                               s_heat_right_kp,
                               s_heat_right_ki,
                               s_heat_right_kd);
        PidController_Reset(&controller->heat_right_pid);
        PidController_SetSetpoint(&controller->heat_right_pid,
                                  controller->cfg.temp_target * 100.0f);
        controller->active_heat_right_profile_version = s_heat_right_pid_version;
        break;

    default:
        break;
    }
}

void TreatmentHeatingControl_InitPid(TreatmentAppController *controller)
{
    PidControllerConfig cfg;

    cfg.kp = s_heat_left_kp;
    cfg.ki = s_heat_left_ki;
    cfg.kd = s_heat_left_kd;
    cfg.setpoint = 0.0f;
    cfg.output_min = 0.0f;
    cfg.output_max = 1999.0f;
    cfg.integral_min = 0.0f;
    cfg.integral_max = 100000.0f;
    cfg.default_dt_s = HEAT_PID_DEFAULT_DT_S;
    cfg.derivative_filter_alpha = 0.85f;
    cfg.derivative_mode = PID_CONTROLLER_DERIVATIVE_ON_MEASUREMENT;
    PidController_Init(&controller->heat_left_pid, &cfg);
    controller->active_heat_left_profile_version = s_heat_left_pid_version;
    cfg.kp = s_heat_right_kp;
    cfg.ki = s_heat_right_ki;
    cfg.kd = s_heat_right_kd;
    PidController_Init(&controller->heat_right_pid, &cfg);
    controller->active_heat_right_profile_version = s_heat_right_pid_version;
}

void TreatmentHeatingControl_ResetPid(TreatmentAppController *controller)
{
    if (controller == NULL)
    {
        return;
    }

    PidController_Reset(&controller->heat_left_pid);
    PidController_Reset(&controller->heat_right_pid);
    controller->active_heat_left_profile_version = s_heat_left_pid_version;
    controller->active_heat_right_profile_version = s_heat_right_pid_version;
}

void TreatmentHeatingControl_UpdateSetpoint(TreatmentAppController *controller)
{
    float setpoint;

    if (controller == NULL)
    {
        return;
    }

    setpoint = controller->cfg.temp_target * 100.0f;
    PidController_SetSetpoint(&controller->heat_left_pid, setpoint);
    PidController_SetSetpoint(&controller->heat_right_pid, setpoint);
}

void TreatmentHeatingControl_ApplyOutputs(TreatmentAppController *controller,
                                         const volatile sensor_data_t *sensor,
                                         float dt_s,
                                         TreatmentAppRuntime *runtime)
{
    float heat_output;

    if ((controller == NULL) || (sensor == NULL) || (runtime == NULL))
    {
        return;
    }

    if (controller->active_heat_left_profile_version != s_heat_left_pid_version)
    {
        TreatmentHeatingControl_LoadPid(controller, PID_DEBUG_TARGET_HEAT_LEFT);
    }
    if (controller->active_heat_right_profile_version != s_heat_right_pid_version)
    {
        TreatmentHeatingControl_LoadPid(controller, PID_DEBUG_TARGET_HEAT_RIGHT);
    }

    if (controller->cfg.press_enable_L != 0U)
    {
        TreatmentActuators_SetHeaterPower(TREATMENT_SIDE_LEFT, 1U);
        heat_output = PidController_ComputeDt(&controller->heat_left_pid,
                                              sensor->tempL * 100.0f,
                                              dt_s);
        runtime->heat_left_pwm = TreatmentHeatingControl_ClampPwm(heat_output, 1999U);
        TreatmentActuators_SetHeaterPwm(TREATMENT_SIDE_LEFT, runtime->heat_left_pwm);
    }
    else
    {
        TreatmentActuators_SetHeaterPwm(TREATMENT_SIDE_LEFT, 0U);
        TreatmentActuators_SetHeaterPower(TREATMENT_SIDE_LEFT, 0U);
    }

    if (controller->cfg.press_enable_R != 0U)
    {
        TreatmentActuators_SetHeaterPower(TREATMENT_SIDE_RIGHT, 1U);
        heat_output = PidController_ComputeDt(&controller->heat_right_pid,
                                              sensor->tempR * 100.0f,
                                              dt_s);
        runtime->heat_right_pwm = TreatmentHeatingControl_ClampPwm(heat_output, 1999U);
        TreatmentActuators_SetHeaterPwm(TREATMENT_SIDE_RIGHT, runtime->heat_right_pwm);
    }
    else
    {
        TreatmentActuators_SetHeaterPwm(TREATMENT_SIDE_RIGHT, 0U);
        TreatmentActuators_SetHeaterPower(TREATMENT_SIDE_RIGHT, 0U);
    }
}

void TreatmentHeatingControl_SetPidGains(pid_debug_target_t target, float kp, float ki, float kd)
{
    switch (target)
    {
    case PID_DEBUG_TARGET_HEAT_LEFT:
        s_heat_left_kp = kp;
        s_heat_left_ki = ki;
        s_heat_left_kd = kd;
        ++s_heat_left_pid_version;
        break;

    case PID_DEBUG_TARGET_HEAT_RIGHT:
        s_heat_right_kp = kp;
        s_heat_right_ki = ki;
        s_heat_right_kd = kd;
        ++s_heat_right_pid_version;
        break;

    default:
        break;
    }
}

void TreatmentHeatingControl_GetPidGains(pid_debug_target_t target, float *kp, float *ki, float *kd)
{
    if ((kp == NULL) || (ki == NULL) || (kd == NULL))
    {
        return;
    }

    switch (target)
    {
    case PID_DEBUG_TARGET_HEAT_LEFT:
        *kp = s_heat_left_kp;
        *ki = s_heat_left_ki;
        *kd = s_heat_left_kd;
        break;

    case PID_DEBUG_TARGET_HEAT_RIGHT:
        *kp = s_heat_right_kp;
        *ki = s_heat_right_ki;
        *kd = s_heat_right_kd;
        break;

    default:
        *kp = 0.0f;
        *ki = 0.0f;
        *kd = 0.0f;
        break;
    }
}

uint32_t TreatmentHeatingControl_GetPidVersion(pid_debug_target_t target)
{
    switch (target)
    {
    case PID_DEBUG_TARGET_HEAT_LEFT:
        return s_heat_left_pid_version;

    case PID_DEBUG_TARGET_HEAT_RIGHT:
        return s_heat_right_pid_version;

    default:
        return 0U;
    }
}
