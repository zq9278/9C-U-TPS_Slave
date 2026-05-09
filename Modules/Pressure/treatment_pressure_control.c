#include "treatment_pressure_control.h"

#include <string.h>
#include "BSP/Pwm/bsp_pwm.h"
#include "UserDrivers/ValveControl/valve_control.h"
#include "tim.h"

/* 压力 PID 缺省参数与执行器映射常量。 */
#define PRESS_PID_DEFAULT_DT_S 0.002f
#define PRESS_VENT_ZERO_KPA 0.50f
#define PRESS_VENT_MAX_MS 500U
#define PRESS_PUMP_MIN_PWM 10U
#define PRESS_PUMP_MAX_PWM 255U
#define PRESS_PUMP_RAW_MAX 255.0f

static BspPwmChannel s_pump_pwm;
static uint8_t s_pressure_hw_initialized = 0U;

/* 当前支持“单眼治疗”和“双眼治疗”两套独立压力 PID 参数。 */
typedef enum
{
    TREATMENT_PRESS_PROFILE_SINGLE_EYE = 0,
    TREATMENT_PRESS_PROFILE_DUAL_EYE = 1
} TreatmentPressureProfileKind;

typedef struct
{
    float kp;
    float ki;
    float kd;
} TreatmentPressurePidGains;

typedef struct
{
    TreatmentPressurePidGains rise;
    TreatmentPressurePidGains hold;
    TreatmentPressurePidGains pulse;
} TreatmentPressurePidProfile;

/* 单眼治疗 PID 参数组。 */
static TreatmentPressurePidProfile s_press_pid_single_eye = {
    {0.1f, 0.1f, 0.0f},
    {0.30f, 0.1f, 0.00f},
    {0.06f, 0.000f, 0.000f},
};

/* 双眼治疗 PID 参数组。 */
static TreatmentPressurePidProfile s_press_pid_dual_eye = {
    {0.1f, 0.1f, 0.0f},
    {0.50f, 0.1f, 0.00f},
    {0.1f, 0.000f, 0.000f},
};

/* 参数版本号，供运行中热切换 PID 配置时判断是否需要重载。 */
static uint32_t s_press_pid_profile_version = 0U;

/* 根据当前左右眼使能状态决定使用单眼还是双眼参数组。 */
static TreatmentPressureProfileKind TreatmentPressureControl_GetProfileKind(
    const TreatmentAppController *controller)
{
    if ((controller != NULL) &&
        (controller->cfg.press_enable_L != 0U) &&
        (controller->cfg.press_enable_R != 0U))
    {
        return TREATMENT_PRESS_PROFILE_DUAL_EYE;
    }

    return TREATMENT_PRESS_PROFILE_SINGLE_EYE;
}

/* 返回可写的 PID 参数组对象。 */
static TreatmentPressurePidProfile *TreatmentPressureControl_SelectProfileMutable(
    TreatmentPressureProfileKind kind)
{
    return (kind == TREATMENT_PRESS_PROFILE_DUAL_EYE) ?
           &s_press_pid_dual_eye :
           &s_press_pid_single_eye;
}

/* 返回只读的 PID 参数组对象。 */
static const TreatmentPressurePidProfile *TreatmentPressureControl_SelectProfile(
    TreatmentPressureProfileKind kind)
{
    return (kind == TREATMENT_PRESS_PROFILE_DUAL_EYE) ?
           &s_press_pid_dual_eye :
           &s_press_pid_single_eye;
}

/* 选择某个阶段对应的可写 PID 参数。 */
static TreatmentPressurePidGains *TreatmentPressureControl_SelectStageGainsMutable(
    TreatmentPressurePidProfile *profile,
    TreatmentPressurePidStage stage)
{
    switch (stage)
    {
    case TREATMENT_PRESS_PID_STAGE_RISE:
        return &profile->rise;
    case TREATMENT_PRESS_PID_STAGE_HOLD:
        return &profile->hold;
    case TREATMENT_PRESS_PID_STAGE_PULSE:
    default:
        return &profile->pulse;
    }
}

/* 选择某个阶段对应的只读 PID 参数。 */
static const TreatmentPressurePidGains *TreatmentPressureControl_SelectStageGains(
    const TreatmentPressurePidProfile *profile,
    TreatmentPressurePidStage stage)
{
    switch (stage)
    {
    case TREATMENT_PRESS_PID_STAGE_RISE:
        return &profile->rise;
    case TREATMENT_PRESS_PID_STAGE_HOLD:
        return &profile->hold;
    case TREATMENT_PRESS_PID_STAGE_PULSE:
    default:
        return &profile->pulse;
    }
}

/* 将 PID 输出归一化映射到泵 PWM 区间，并保留最小有效驱动。 */
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

static void TreatmentPressureControl_ApplyPressureRouteOutputs(uint8_t enable_left,
                                                               uint8_t enable_right)
{
    ValveControl_ApplyTreatmentRoute(enable_left, enable_right);
}

static void TreatmentPressureControl_SetPressureVentAllOutputs(void)
{
    ValveControl_SetVentAll();
}

static void TreatmentPressureControl_SetWaveValveOutput(uint8_t enabled)
{
    ValveControl_SetWave(enabled);
}

static void TreatmentPressureControl_SetPumpPwmOutput(uint16_t pwm)
{
    BspPwm_SetPulse(&s_pump_pwm, (pwm > PRESS_PUMP_MAX_PWM) ? PRESS_PUMP_MAX_PWM : pwm);
}

void TreatmentPressureControl_InitHardware(void)
{
    BspPwmChannelConfig cfg;

    if (s_pressure_hw_initialized != 0U)
    {
        return;
    }

    cfg.htim = &htim15;
    cfg.channel = TIM_CHANNEL_1;
    BspPwm_Init(&s_pump_pwm, &cfg);
    (void)BspPwm_Start(&s_pump_pwm);
    s_pressure_hw_initialized = 1U;
    TreatmentPressureControl_SetIdleOutputs();
}

void TreatmentPressureControl_SetIdleOutputs(void)
{
    TreatmentPressureControl_SetPumpPwmOutput(0U);
    TreatmentPressureControl_SetWaveValveOutput(0U);
    TreatmentPressureControl_SetPressureVentAllOutputs();
}

/*
 * 反馈压力选择规则：
 * - 双眼治疗时取左右眼较大值，保证两侧都不会低于目标过多；
 * - 单眼治疗时取启用侧压力。
 */
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

/* 记录上一周期阶段信息，供跨周期放气和阶段切换判断使用。 */
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

/* 放气阶段的统一输出：阀路切换到放气，泵关闭。 */
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
    TreatmentPressureControl_SetPressureVentAllOutputs();
    TreatmentPressureControl_SetWaveValveOutput(0U);
    TreatmentPressureControl_SetPumpPwmOutput(0U);
    TreatmentPressureControl_UpdateHistory(controller, plan);
}

/* 把 rise/hold/pulse 三段时长归一化映射到固定 60s 周期。 */
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

/* 计算当前脉冲周期总长度。 */
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

/* 计算当前脉冲周期内导通时长。 */
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

/* 将当前治疗模式和阶段对应的 PID 参数真正加载到控制器中。 */
static void TreatmentPressureControl_ApplyProfile(TreatmentAppController *controller,
                                                  TreatmentPressurePidStage stage)
{
    TreatmentPressureProfileKind profile_kind;
    const TreatmentPressurePidGains *gains;

    profile_kind = TreatmentPressureControl_GetProfileKind(controller);
    gains = TreatmentPressureControl_SelectStageGains(
        TreatmentPressureControl_SelectProfile(profile_kind), stage);

    PidController_SetGains(&controller->pressure_pid, gains->kp, gains->ki, gains->kd);
    PidController_Reset(&controller->pressure_pid);
    controller->active_pressure_stage = stage;
    controller->active_pressure_profile_kind = (uint8_t)profile_kind;
    controller->active_pressure_profile_version = s_press_pid_profile_version;
    controller->pressure_profile_loaded = 1U;
}

/* 阶段名字符串主要用于日志和遥测。 */
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
    const TreatmentPressurePidGains *gains;

    /* 初始化时先加载当前模式下的 rise 段 PID，后续再按阶段切换。 */
    gains = TreatmentPressureControl_SelectStageGains(
        TreatmentPressureControl_SelectProfile(TreatmentPressureControl_GetProfileKind(controller)),
        TREATMENT_PRESS_PID_STAGE_RISE);
    cfg.kp = gains->kp;
    cfg.ki = gains->ki;
    cfg.kd = gains->kd;
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

    /* 清空运行时状态，但不动静态参数组。 */
    controller->pressure_profile_loaded = 0U;
    controller->pressure_cycle_seen = 0U;
    controller->pressure_cycle_venting = 0U;
    controller->pressure_vent_start_tick = 0U;
    controller->last_pressure_cycle_elapsed_ms = 0U;
    controller->last_pressure_phase = TREATMENT_PHASE_IDLE;
    controller->active_pressure_profile_kind = (uint8_t)TREATMENT_PRESS_PROFILE_SINGLE_EYE;
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

    /* 先清零计划结构，再按当前相位计算目标值与阶段。 */
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
        /* 升压段：目标压力线性爬升。 */
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
        /* 保压段：目标压力保持峰值。 */
        plan->phase = TREATMENT_PHASE_HOLD;
        plan->pid_stage = TREATMENT_PRESS_PID_STAGE_HOLD;
        plan->target_pressure_kpa = controller->cfg.press_target_max;
        plan->inflating = 1U;
    }
    else
    {
        /* 脉冲段：依据 pulse_on / pulse_off 交替导通与放气。 */
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
    TreatmentPressureProfileKind profile_kind;

    if ((controller == NULL) || (sensor == NULL) || (plan == NULL) || (runtime == NULL))
    {
        return;
    }

    /* 先选定本周期使用的反馈压力与参数组。 */
    feedback_kpa = TreatmentPressureControl_GetFeedbackKpa(controller, sensor);
    profile_kind = TreatmentPressureControl_GetProfileKind(controller);
    cycle_wrapped = (uint8_t)((controller->pressure_cycle_seen != 0U) &&
                              (plan->cycle_elapsed_ms < controller->last_pressure_cycle_elapsed_ms));
    pulse_rise_started = (uint8_t)((plan->phase == TREATMENT_PHASE_PULSE_ON) &&
                                   (controller->last_pressure_phase != TREATMENT_PHASE_PULSE_ON));

    if ((cycle_wrapped != 0U) &&
        ((controller->last_pressure_phase == TREATMENT_PHASE_PULSE_ON) ||
         (controller->last_pressure_phase == TREATMENT_PHASE_PULSE_OFF)))
    {
        /* 一个治疗大周期结束后，先强制进入短放气，避免残压带入下一轮。 */
        controller->pressure_cycle_venting = 1U;
        controller->pressure_vent_start_tick = xTaskGetTickCount();
        PidController_Reset(&controller->pressure_pid);
    }

    if ((controller->pressure_profile_loaded == 0U) ||
        (controller->active_pressure_stage != plan->pid_stage) ||
        (controller->active_pressure_profile_kind != (uint8_t)profile_kind) ||
        (controller->active_pressure_profile_version != s_press_pid_profile_version))
    {
        /* 运行中阶段切换、模式切换或参数热更新时，都需要重新装载 PID。 */
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
        /* 脉冲关闭窗口直接执行放气，不参与 PID。 */
        TreatmentPressureControl_ApplyVentOutputs(controller, plan, feedback_kpa, runtime);
        return;
    }

    /* 切换气路到当前治疗眼别。 */
    TreatmentPressureControl_ApplyPressureRouteOutputs(controller->cfg.press_enable_L,
                                                       controller->cfg.press_enable_R);
    if (pulse_rise_started != 0U)
    {
        PidController_Reset(&controller->pressure_pid);
    }

    if (plan->inflating != 0U)
    {
        /* 需要加压时打开波形阀并执行压力 PID。 */
        TreatmentPressureControl_SetWaveValveOutput(1U);
        PidController_SetSetpoint(&controller->pressure_pid, plan->target_pressure_kpa);
        pressure_output = PidController_ComputeDt(&controller->pressure_pid, feedback_kpa, dt_s);
        runtime->pump_pwm = TreatmentPressureControl_ClampPwm(pressure_output, PRESS_PUMP_MAX_PWM);
        controller->pressure_pid.debug.mapped_output = (float)runtime->pump_pwm;
        TreatmentPressureControl_SetPumpPwmOutput(runtime->pump_pwm);
    }
    else
    {
        /* 这里理论上目前只会用于某些扩展场景，保守关闭输出。 */
        TreatmentPressureControl_SetWaveValveOutput(0U);
        runtime->pump_pwm = 0U;
        controller->pressure_pid.debug.mapped_output = 0.0f;
        TreatmentPressureControl_SetPumpPwmOutput(0U);
    }

    TreatmentPressureControl_UpdateHistory(controller, plan);
}

/* 只修改当前治疗模式对应的 PID 参数组。 */
void TreatmentPressureControl_SetPidGainsForController(const TreatmentAppController *controller,
                                                       TreatmentPressurePidStage stage,
                                                       float kp,
                                                       float ki,
                                                       float kd)
{
    TreatmentPressurePidProfile *profile;
    TreatmentPressurePidGains *gains;

    profile = TreatmentPressureControl_SelectProfileMutable(
        TreatmentPressureControl_GetProfileKind(controller));
    gains = TreatmentPressureControl_SelectStageGainsMutable(profile, stage);
    gains->kp = kp;
    gains->ki = ki;
    gains->kd = kd;
    ++s_press_pid_profile_version;
}

/* 读取当前治疗模式对应的 PID 参数组。 */
void TreatmentPressureControl_GetPidGainsForController(const TreatmentAppController *controller,
                                                       TreatmentPressurePidStage stage,
                                                       float *kp,
                                                       float *ki,
                                                       float *kd)
{
    const TreatmentPressurePidGains *gains;

    if ((kp == NULL) || (ki == NULL) || (kd == NULL))
    {
        return;
    }

    gains = TreatmentPressureControl_SelectStageGains(
        TreatmentPressureControl_SelectProfile(TreatmentPressureControl_GetProfileKind(controller)),
        stage);
    *kp = gains->kp;
    *ki = gains->ki;
    *kd = gains->kd;
}

/* 同步修改单双眼两套参数，适用于“全局默认参数”写入。 */
void TreatmentPressureControl_SetPidGains(TreatmentPressurePidStage stage, float kp, float ki, float kd)
{
    TreatmentPressurePidGains *single_gains;
    TreatmentPressurePidGains *dual_gains;

    single_gains = TreatmentPressureControl_SelectStageGainsMutable(&s_press_pid_single_eye, stage);
    dual_gains = TreatmentPressureControl_SelectStageGainsMutable(&s_press_pid_dual_eye, stage);

    single_gains->kp = kp;
    single_gains->ki = ki;
    single_gains->kd = kd;
    dual_gains->kp = kp;
    dual_gains->ki = ki;
    dual_gains->kd = kd;

    ++s_press_pid_profile_version;
}

/* 默认读取双眼参数组，兼容旧接口。 */
void TreatmentPressureControl_GetPidGains(TreatmentPressurePidStage stage, float *kp, float *ki, float *kd)
{
    const TreatmentPressurePidGains *gains;

    if ((kp == NULL) || (ki == NULL) || (kd == NULL))
    {
        return;
    }

    gains = TreatmentPressureControl_SelectStageGains(&s_press_pid_dual_eye, stage);
    *kp = gains->kp;
    *ki = gains->ki;
    *kd = gains->kd;
}

uint32_t TreatmentPressureControl_GetPidVersion(void)
{
    return s_press_pid_profile_version;
}
