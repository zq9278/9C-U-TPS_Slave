#include "treatment_pressure_control.h"

#include <string.h>
#include "TreatmentActuators.h"

#define PRESS_PID_DEFAULT_DT_S 0.002f
#define PRESS_VENT_ZERO_KPA 0.50f
#define PRESS_VENT_MAX_MS 500U
#define PRESS_PUMP_MIN_PWM 10U
#define PRESS_PUMP_MAX_PWM 255U
#define PRESS_PUMP_RAW_MAX 255.0f

static float s_press_pid_rise_kp = 0.1f;
static float s_press_pid_rise_ki = 0.1f;
static float s_press_pid_rise_kd = 0.0f;
static float s_press_pid_hold_kp = 0.50;
static float s_press_pid_hold_ki = 0.1f;
static float s_press_pid_hold_kd = 0.00f;
static float s_press_pid_pulse_kp = 0.1f;
static float s_press_pid_pulse_ki = 0.000f;
static float s_press_pid_pulse_kd = 0.000f;
static uint32_t s_press_pid_profile_version = 0U;

static uint16_t TreatmentPressureControl_ClampPwm(float value, uint16_t max_value)
{
    float scaled_value;

    if (value <= 0.0f)
    {
        return 0U;
    }
    if (value >= (float)max_value)
    {
        return max_value;
    }

    if (max_value <= PRESS_PUMP_MIN_PWM)
    {
        return max_value;
    }

    scaled_value =
        (float)PRESS_PUMP_MIN_PWM +
        (value * (float)(max_value - PRESS_PUMP_MIN_PWM) / PRESS_PUMP_RAW_MAX);

    if (scaled_value <= (float)PRESS_PUMP_MIN_PWM)
    {
        return PRESS_PUMP_MIN_PWM;
    }

    if (scaled_value >= (float)max_value)
    {
        return max_value;
    }

    return (uint16_t)scaled_value;
}

static float TreatmentPressureControl_GetFeedbackKpa(const TreatmentAppController *controller,
                                                     const volatile sensor_data_t *sensor)
{
    if ((controller == NULL) || (sensor == NULL))
    {
        return 0.0f;
    }

    if ((controller->cfg.press_enable_L != 0U) && (controller->cfg.press_enable_R != 0U))
    {
        return (sensor->pressL > sensor->pressR) ? sensor->pressL : sensor->pressR;
    }

    if (controller->cfg.press_enable_R != 0U)
    {
        return sensor->pressR;
    }

    return sensor->pressL;
}

static void TreatmentPressureControl_UpdateHistory(TreatmentAppController *controller,
                                                   const TreatmentPressurePlan *plan)
{
    if ((controller == NULL) || (plan == NULL))
    {
        return;
    }

    controller->last_pressure_phase = plan->phase;
    controller->last_pressure_cycle_elapsed_ms = plan->cycle_elapsed_ms;
    controller->pressure_cycle_seen = 1U;
}

static void TreatmentPressureControl_ApplyVentOutputs(TreatmentAppController *controller,
                                                      const TreatmentPressurePlan *plan,
                                                      float feedback_kpa,
                                                      TreatmentAppRuntime *runtime)
{
    runtime->phase_name = "vent";
    runtime->phase_char = (uint8_t)'v';
    runtime->target_pressure_kpa = 0.0f;
    runtime->feedback_pressure_kpa = feedback_kpa;
    runtime->pump_pwm = 0U;
    runtime->running_outputs = 1U;
    TreatmentActuators_SetPressureVentAll();
    TreatmentActuators_SetWaveValve(0U);
    TreatmentActuators_SetPumpPwm(0U);
    TreatmentPressureControl_UpdateHistory(controller, plan);
}

static void TreatmentPressureControl_NormalizeStageMs(uint32_t *rise_ms,
                                                      uint32_t *hold_ms,
                                                      uint32_t *pulse_ms)
{
    uint32_t r = *rise_ms;
    uint32_t h = *hold_ms;
    uint32_t p = *pulse_ms;
    uint32_t total = r + h + p;

    if (total == 0U)
    {
        r = 20000U;
        h = 20000U;
        p = 20000U;
    }
    else
    {
        r = (uint32_t)(((uint64_t)r * TREATMENT_CYCLE_MS) / total);
        h = (uint32_t)(((uint64_t)h * TREATMENT_CYCLE_MS) / total);
        if ((r + h) >= TREATMENT_CYCLE_MS)
        {
            if (r >= TREATMENT_CYCLE_MS)
            {
                r = TREATMENT_CYCLE_MS;
                h = 0U;
            }
            else
            {
                h = TREATMENT_CYCLE_MS - r;
            }
            p = 0U;
        }
        else
        {
            p = TREATMENT_CYCLE_MS - r - h;
        }
    }

    *rise_ms = r;
    *hold_ms = h;
    *pulse_ms = p;
}

static uint32_t TreatmentPressureControl_GetPulsePeriodMs(const TreatmentAppController *controller)
{
    uint32_t pulse_on_ms;
    uint32_t pulse_off_ms;
    uint32_t pulse_period_ms;

    if (controller == NULL)
    {
        return TREATMENT_DEFAULT_PULSE_PERIOD_MS;
    }

    pulse_on_ms = (controller->cfg.pulse_on_ms > 0.0f) ? (uint32_t)controller->cfg.pulse_on_ms : 0U;
    pulse_off_ms = (controller->cfg.pulse_off_ms > 0.0f) ? (uint32_t)controller->cfg.pulse_off_ms : 0U;
    pulse_period_ms = pulse_on_ms + pulse_off_ms;

    return (pulse_period_ms > 0U) ? pulse_period_ms : TREATMENT_DEFAULT_PULSE_PERIOD_MS;
}

static uint32_t TreatmentPressureControl_GetPulseOnMs(const TreatmentAppController *controller,
                                                      uint32_t pulse_period_ms)
{
    uint32_t pulse_on_ms;

    if (controller == NULL)
    {
        return TREATMENT_DEFAULT_PULSE_ON_MS;
    }

    pulse_on_ms = (controller->cfg.pulse_on_ms > 0.0f) ? (uint32_t)controller->cfg.pulse_on_ms : 0U;
    if (pulse_on_ms == 0U)
    {
        pulse_on_ms = pulse_period_ms / 2U;
    }
    if (pulse_on_ms > pulse_period_ms)
    {
        pulse_on_ms = pulse_period_ms;
    }

    return pulse_on_ms;
}

static void TreatmentPressureControl_ApplyProfile(TreatmentAppController *controller,
                                                  TreatmentPressurePidStage stage)
{
    float kp;
    float ki;
    float kd;

    switch (stage)
    {
    case TREATMENT_PRESS_PID_STAGE_RISE:
        kp = s_press_pid_rise_kp;
        ki = s_press_pid_rise_ki;
        kd = s_press_pid_rise_kd;
        break;
    case TREATMENT_PRESS_PID_STAGE_HOLD:
        kp = s_press_pid_hold_kp;
        ki = s_press_pid_hold_ki;
        kd = s_press_pid_hold_kd;
        break;
    case TREATMENT_PRESS_PID_STAGE_PULSE:
    default:
        kp = s_press_pid_pulse_kp;
        ki = s_press_pid_pulse_ki;
        kd = s_press_pid_pulse_kd;
        break;
    }

    PidController_SetGains(&controller->pressure_pid, kp, ki, kd);
    PidController_Reset(&controller->pressure_pid);
    controller->active_pressure_stage = stage;
    controller->active_pressure_profile_version = s_press_pid_profile_version;
    controller->pressure_profile_loaded = 1U;
}

const char *TreatmentPressureControl_PhaseName(TreatmentPhase phase)
{
    switch (phase)
    {
    case TREATMENT_PHASE_RISE: return "rise";
    case TREATMENT_PHASE_HOLD: return "hold";
    case TREATMENT_PHASE_PULSE_ON: return "pulse_on";
    case TREATMENT_PHASE_PULSE_OFF: return "pulse_off";
    case TREATMENT_PHASE_PAUSE: return "pause";
    case TREATMENT_PHASE_IDLE:
    default: return "idle";
    }
}

void TreatmentPressureControl_InitPid(TreatmentAppController *controller)
{
    PidControllerConfig cfg;

    cfg.kp = s_press_pid_rise_kp;
    cfg.ki = s_press_pid_rise_ki;
    cfg.kd = s_press_pid_rise_kd;
    cfg.setpoint = 0.0f;
    cfg.output_min = 0.0f;
    cfg.output_max = 255.0f;
    cfg.integral_min = -200.0f;
    cfg.integral_max = 200.0f;
    cfg.default_dt_s = PRESS_PID_DEFAULT_DT_S;
    cfg.derivative_filter_alpha = 0.80f;
    cfg.derivative_mode = PID_CONTROLLER_DERIVATIVE_ON_MEASUREMENT;
    PidController_Init(&controller->pressure_pid, &cfg);
}

void TreatmentPressureControl_ResetPid(TreatmentAppController *controller)
{
    if (controller == NULL)
    {
        return;
    }

    controller->pressure_profile_loaded = 0U;
    controller->pressure_cycle_seen = 0U;
    controller->pressure_cycle_venting = 0U;
    controller->pressure_vent_start_tick = 0U;
    controller->last_pressure_cycle_elapsed_ms = 0U;
    controller->last_pressure_phase = TREATMENT_PHASE_IDLE;
    PidController_Reset(&controller->pressure_pid);
}

void TreatmentPressureControl_BuildPlan(const TreatmentAppController *controller,
                                        TickType_t now_tick,
                                        TreatmentPressurePlan *plan)
{
    uint32_t rise_ms;
    uint32_t hold_ms;
    uint32_t pulse_ms;
    uint32_t elapsed_total_ms;
    uint32_t cycle_elapsed_ms;

    (void)memset(plan, 0, sizeof(*plan));
    rise_ms = (uint32_t)(controller->cfg.t1_rise_s * 1000.0f);
    hold_ms = (uint32_t)(controller->cfg.t2_hold_s * 1000.0f);
    pulse_ms = (uint32_t)(controller->cfg.t3_pulse_s * 1000.0f);
    TreatmentPressureControl_NormalizeStageMs(&rise_ms, &hold_ms, &pulse_ms);

    elapsed_total_ms =
        (uint32_t)((now_tick - controller->wave_anchor_tick) * portTICK_PERIOD_MS);
    cycle_elapsed_ms = elapsed_total_ms % TREATMENT_CYCLE_MS;
    plan->cycle_elapsed_ms = cycle_elapsed_ms;

    if (cycle_elapsed_ms < rise_ms)
    {
        float ratio = (rise_ms > 0U) ? ((float)cycle_elapsed_ms / (float)rise_ms) : 1.0f;

        if (ratio > 1.0f)
        {
            ratio = 1.0f;
        }

        plan->phase = TREATMENT_PHASE_RISE;
        plan->pid_stage = TREATMENT_PRESS_PID_STAGE_RISE;
        plan->target_pressure_kpa = controller->cfg.press_target_max * ratio;
        plan->inflating = 1U;
    }
    else if (cycle_elapsed_ms < (rise_ms + hold_ms))
    {
        plan->phase = TREATMENT_PHASE_HOLD;
        plan->pid_stage = TREATMENT_PRESS_PID_STAGE_HOLD;
        plan->target_pressure_kpa = controller->cfg.press_target_max;
        plan->inflating = 1U;
    }
    else
    {
        uint32_t pulse_elapsed_ms = cycle_elapsed_ms - rise_ms - hold_ms;
        uint32_t pulse_period_ms = TreatmentPressureControl_GetPulsePeriodMs(controller);
        uint32_t pulse_on_ms = TreatmentPressureControl_GetPulseOnMs(controller, pulse_period_ms);
        uint8_t on_phase = (uint8_t)((pulse_elapsed_ms % pulse_period_ms) < pulse_on_ms);

        plan->phase = on_phase ? TREATMENT_PHASE_PULSE_ON : TREATMENT_PHASE_PULSE_OFF;
        plan->pid_stage = TREATMENT_PRESS_PID_STAGE_PULSE;
        plan->target_pressure_kpa = on_phase ? controller->cfg.press_target_max : 0.0f;
        plan->inflating = on_phase;
    }
}

void TreatmentPressureControl_ApplyPlan(TreatmentAppController *controller,
                                        const volatile sensor_data_t *sensor,
                                        float dt_s,
                                        const TreatmentPressurePlan *plan,
                                        TreatmentAppRuntime *runtime)
{
    float pressure_output;
    float feedback_kpa;
    uint8_t cycle_wrapped;
    uint8_t pulse_rise_started;

    if ((controller == NULL) || (sensor == NULL) || (plan == NULL) || (runtime == NULL))
    {
        return;
    }

    feedback_kpa = TreatmentPressureControl_GetFeedbackKpa(controller, sensor);
    cycle_wrapped = (uint8_t)((controller->pressure_cycle_seen != 0U) &&
                              (plan->cycle_elapsed_ms < controller->last_pressure_cycle_elapsed_ms));
    pulse_rise_started = (uint8_t)((plan->phase == TREATMENT_PHASE_PULSE_ON) &&
                                   (controller->last_pressure_phase != TREATMENT_PHASE_PULSE_ON));

    if ((cycle_wrapped != 0U) &&
        ((controller->last_pressure_phase == TREATMENT_PHASE_PULSE_ON) ||
         (controller->last_pressure_phase == TREATMENT_PHASE_PULSE_OFF)))
    {
        controller->pressure_cycle_venting = 1U;
        controller->pressure_vent_start_tick = xTaskGetTickCount();
        PidController_Reset(&controller->pressure_pid);
    }

    if ((controller->pressure_profile_loaded == 0U) ||
        (controller->active_pressure_stage != plan->pid_stage) ||
        (controller->active_pressure_profile_version != s_press_pid_profile_version))
    {
        TreatmentPressureControl_ApplyProfile(controller, plan->pid_stage);
    }

    runtime->phase = plan->phase;
    runtime->pid_stage = plan->pid_stage;
    runtime->phase_name = TreatmentPressureControl_PhaseName(plan->phase);
    runtime->phase_char = (uint8_t)runtime->phase_name[0];
    runtime->session_active = 1U;
    runtime->running_outputs = 1U;
    runtime->target_pressure_kpa = plan->target_pressure_kpa;
    runtime->feedback_pressure_kpa = feedback_kpa;
    runtime->cycle_elapsed_ms = plan->cycle_elapsed_ms;

    if (controller->pressure_cycle_venting != 0U)
    {
        uint32_t vent_elapsed_ms =
            (uint32_t)((xTaskGetTickCount() - controller->pressure_vent_start_tick) * portTICK_PERIOD_MS);

        if ((feedback_kpa <= PRESS_VENT_ZERO_KPA) || (vent_elapsed_ms >= PRESS_VENT_MAX_MS))
        {
            controller->pressure_cycle_venting = 0U;
            PidController_Reset(&controller->pressure_pid);
        }
        else
        {
            TreatmentPressureControl_ApplyVentOutputs(controller, plan, feedback_kpa, runtime);
            return;
        }
    }

    if (plan->phase == TREATMENT_PHASE_PULSE_OFF)
    {
        TreatmentPressureControl_ApplyVentOutputs(controller, plan, feedback_kpa, runtime);
        return;
    }

    TreatmentActuators_ApplyPressureRoute(controller->cfg.press_enable_L,
                                          controller->cfg.press_enable_R);
    if (pulse_rise_started != 0U)
    {
        PidController_Reset(&controller->pressure_pid);
    }

    if (plan->inflating != 0U)
    {
        TreatmentActuators_SetWaveValve(1U);
        PidController_SetSetpoint(&controller->pressure_pid, plan->target_pressure_kpa);
        pressure_output = PidController_ComputeDt(&controller->pressure_pid, feedback_kpa, dt_s);
        runtime->pump_pwm = TreatmentPressureControl_ClampPwm(pressure_output, PRESS_PUMP_MAX_PWM);
        controller->pressure_pid.debug.mapped_output = (float)runtime->pump_pwm;
        TreatmentActuators_SetPumpPwm(runtime->pump_pwm);
    }
    else
    {
        TreatmentActuators_SetWaveValve(0U);
        runtime->pump_pwm = 0U;
        controller->pressure_pid.debug.mapped_output = 0.0f;
        TreatmentActuators_SetPumpPwm(0U);
    }

    TreatmentPressureControl_UpdateHistory(controller, plan);
}

void TreatmentPressureControl_SetPidGains(TreatmentPressurePidStage stage, float kp, float ki, float kd)
{
    switch (stage)
    {
    case TREATMENT_PRESS_PID_STAGE_RISE:
        s_press_pid_rise_kp = kp;
        s_press_pid_rise_ki = ki;
        s_press_pid_rise_kd = kd;
        break;
    case TREATMENT_PRESS_PID_STAGE_HOLD:
        s_press_pid_hold_kp = kp;
        s_press_pid_hold_ki = ki;
        s_press_pid_hold_kd = kd;
        break;
    case TREATMENT_PRESS_PID_STAGE_PULSE:
    default:
        s_press_pid_pulse_kp = kp;
        s_press_pid_pulse_ki = ki;
        s_press_pid_pulse_kd = kd;
        break;
    }

    ++s_press_pid_profile_version;
}

void TreatmentPressureControl_GetPidGains(TreatmentPressurePidStage stage, float *kp, float *ki, float *kd)
{
    if ((kp == NULL) || (ki == NULL) || (kd == NULL))
    {
        return;
    }

    switch (stage)
    {
    case TREATMENT_PRESS_PID_STAGE_RISE:
        *kp = s_press_pid_rise_kp;
        *ki = s_press_pid_rise_ki;
        *kd = s_press_pid_rise_kd;
        break;
    case TREATMENT_PRESS_PID_STAGE_HOLD:
        *kp = s_press_pid_hold_kp;
        *ki = s_press_pid_hold_ki;
        *kd = s_press_pid_hold_kd;
        break;
    case TREATMENT_PRESS_PID_STAGE_PULSE:
    default:
        *kp = s_press_pid_pulse_kp;
        *ki = s_press_pid_pulse_ki;
        *kd = s_press_pid_pulse_kd;
        break;
    }
}

uint32_t TreatmentPressureControl_GetPidVersion(void)
{
    return s_press_pid_profile_version;
}
