#include "treatment_heating_control.h"

#include "BSP/Pwm/bsp_pwm.h"
#include "FreeRTOS.h"
#include "main.h"
#define MODULE_LOG_ENABLED MODULE_LOG_ACTUATOR_ENABLE
#include "Modules/Log/module_log.h"
#include "task.h"
#include "tim.h"

#define HEAT_PID_DEFAULT_DT_S 0.002f
#define HEAT_PID_INTEGRAL_MAX 600.0f
#define HEAT_OTP_NORMAL_LEVEL  1U
#define HEAT_TARGET_OFFSET_LEFT_C  0.0f
#define HEAT_TARGET_OFFSET_RIGHT_C 0.0f

static float s_heat_left_kp = 0.700f;
static float s_heat_left_ki = 0.300f;
static float s_heat_left_kd = 0.0000f;
static float s_heat_right_kp = 0.700f;
static float s_heat_right_ki = 0.3000f;
static float s_heat_right_kd = 0.0000f;
static uint32_t s_heat_left_pid_version = 0U;
static uint32_t s_heat_right_pid_version = 0U;

static BspPwmChannel s_heat_left_pwm;
static BspPwmChannel s_heat_right_pwm;
static uint8_t s_heat_hw_initialized = 0U;
static uint8_t s_heat_left_enabled = 0U;
static uint8_t s_heat_right_enabled = 0U;
static uint16_t s_heat_left_last_pwm = 0U;
static uint16_t s_heat_right_last_pwm = 0U;
static uint8_t s_heat_left_otp_reference = 0U;
static uint8_t s_heat_right_otp_reference = 0U;
static uint8_t s_heat_left_otp_reference_valid = 0U;
static uint8_t s_heat_right_otp_reference_valid = 0U;

static float TreatmentHeatingControl_GetEffectiveTargetC(TreatmentSide side,
                                                         const TreatmentAppController *controller)
{
    float base_target = 0.0f;

    if (controller != NULL)
    {
        base_target = controller->cfg.temp_target;
    }

    return base_target +
           ((side == TREATMENT_SIDE_LEFT) ? HEAT_TARGET_OFFSET_LEFT_C
                                          : HEAT_TARGET_OFFSET_RIGHT_C);
}

static uint32_t TreatmentHeatingControl_PinToIndex(uint16_t pin)
{
    uint32_t index;

    for (index = 0U; index < 16U; ++index)
    {
        if (pin == (uint16_t)(1U << index))
        {
            return index;
        }
    }

    return 0U;
}

static void TreatmentHeatingControl_GpioSetModeInput(GPIO_TypeDef *port, uint16_t pin)
{
    uint32_t shift = TreatmentHeatingControl_PinToIndex(pin) * 2U;

    port->MODER &= ~(0x3UL << shift);
}

static void TreatmentHeatingControl_GpioSetModeOutput(GPIO_TypeDef *port, uint16_t pin)
{
    uint32_t shift = TreatmentHeatingControl_PinToIndex(pin) * 2U;

    port->MODER &= ~(0x3UL << shift);
    port->MODER |= (0x1UL << shift);
}

static void TreatmentHeatingControl_DelayMs(uint32_t delay_ms)
{
    if (xTaskGetSchedulerState() == taskSCHEDULER_RUNNING)
    {
        vTaskDelay(pdMS_TO_TICKS(delay_ms));
    }
    else
    {
        HAL_Delay(delay_ms);
    }
}

static void TreatmentHeatingControl_ForceOtpHighAll(void)
{
#if (HEAT_OTP_FAULT_REPORT_ENABLE == 0U)
    TreatmentHeatingControl_GpioSetModeOutput(OTP1_RESET_GPIO_Port, OTP1_RESET_Pin);
    TreatmentHeatingControl_GpioSetModeOutput(OTP2_RESET_GPIO_Port, OTP2_RESET_Pin);
    HAL_GPIO_WritePin(OTP1_RESET_GPIO_Port, OTP1_RESET_Pin, GPIO_PIN_SET);
    HAL_GPIO_WritePin(OTP2_RESET_GPIO_Port, OTP2_RESET_Pin, GPIO_PIN_SET);
#endif
}

static uint16_t TreatmentHeatingControl_ClampU16(uint16_t value, uint16_t max_value)
{
    return (value > max_value) ? max_value : value;
}

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

static void TreatmentHeatingControl_SetPower(TreatmentSide side, uint8_t enabled)
{
    /* 板级映射：左眼走 Heat2/OTP2，右眼走 Heat1/OTP1。 */
    if (side == TREATMENT_SIDE_LEFT)
    {
        if (enabled != 0U)
        {
            if (BspPwm_Start(&s_heat_right_pwm) == 0U)
            {
                LOG_E("heater L pwm start failed");
            }
            if (s_heat_left_enabled == 0U)
            {
                LOG_I("heater L power on");
            }
            s_heat_left_enabled = 1U;
        }
        else
        {
            BspPwm_SetPulse(&s_heat_right_pwm, 0U);
            if (BspPwm_Stop(&s_heat_right_pwm) == 0U)
            {
                LOG_E("heater L pwm stop failed");
            }
#if (HEAT_OTP_FAULT_REPORT_ENABLE != 0U)
            TreatmentHeatingControl_GpioSetModeOutput(OTP2_RESET_GPIO_Port, OTP2_RESET_Pin);
            HAL_GPIO_WritePin(OTP2_RESET_GPIO_Port, OTP2_RESET_Pin, GPIO_PIN_RESET);
#else
            TreatmentHeatingControl_ForceOtpHighAll();
#endif
            if (s_heat_left_enabled != 0U)
            {
                LOG_I("heater L power off");
            }
            s_heat_left_enabled = 0U;
            s_heat_left_otp_reference_valid = 0U;
        }
    }
    else
    {
        if (enabled != 0U)
        {
            if (BspPwm_Start(&s_heat_left_pwm) == 0U)
            {
                LOG_E("heater R pwm start failed");
            }
            if (s_heat_right_enabled == 0U)
            {
                LOG_I("heater R power on");
            }
            s_heat_right_enabled = 1U;
        }
        else
        {
            BspPwm_SetPulse(&s_heat_left_pwm, 0U);
            if (BspPwm_Stop(&s_heat_left_pwm) == 0U)
            {
                LOG_E("heater R pwm stop failed");
            }
#if (HEAT_OTP_FAULT_REPORT_ENABLE != 0U)
            TreatmentHeatingControl_GpioSetModeOutput(OTP1_RESET_GPIO_Port, OTP1_RESET_Pin);
            HAL_GPIO_WritePin(OTP1_RESET_GPIO_Port, OTP1_RESET_Pin, GPIO_PIN_RESET);
#else
            TreatmentHeatingControl_ForceOtpHighAll();
#endif
            if (s_heat_right_enabled != 0U)
            {
                LOG_I("heater R power off");
            }
            s_heat_right_enabled = 0U;
            s_heat_right_otp_reference_valid = 0U;
        }
    }
}

static void TreatmentHeatingControl_SetPwm(TreatmentSide side, uint16_t pwm)
{
    uint16_t clamped_pwm = TreatmentHeatingControl_ClampU16(pwm, 1999U);

    if (side == TREATMENT_SIDE_LEFT)
    {
        if ((clamped_pwm != 0U) && (BspPwm_Start(&s_heat_right_pwm) == 0U))
        {
            LOG_E("heater L pwm start failed while set pwm=%u", clamped_pwm);
        }
        BspPwm_SetPulse(&s_heat_right_pwm, clamped_pwm);
        if (((s_heat_left_last_pwm == 0U) && (clamped_pwm != 0U)) ||
            ((s_heat_left_last_pwm != 0U) && (clamped_pwm == 0U)))
        {
            LOG_I("heater L pwm=%u", clamped_pwm);
        }
        s_heat_left_last_pwm = clamped_pwm;
    }
    else
    {
        if ((clamped_pwm != 0U) && (BspPwm_Start(&s_heat_left_pwm) == 0U))
        {
            LOG_E("heater R pwm start failed while set pwm=%u", clamped_pwm);
        }
        BspPwm_SetPulse(&s_heat_left_pwm, clamped_pwm);
        if (((s_heat_right_last_pwm == 0U) && (clamped_pwm != 0U)) ||
            ((s_heat_right_last_pwm != 0U) && (clamped_pwm == 0U)))
        {
            LOG_I("heater R pwm=%u", clamped_pwm);
        }
        s_heat_right_last_pwm = clamped_pwm;
    }
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
                                  TreatmentHeatingControl_GetEffectiveTargetC(TREATMENT_SIDE_LEFT,
                                                                              controller) * 100.0f);
        controller->active_heat_left_profile_version = s_heat_left_pid_version;
        break;

    case PID_DEBUG_TARGET_HEAT_RIGHT:
        PidController_SetGains(&controller->heat_right_pid,
                               s_heat_right_kp,
                               s_heat_right_ki,
                               s_heat_right_kd);
        PidController_Reset(&controller->heat_right_pid);
        PidController_SetSetpoint(&controller->heat_right_pid,
                                  TreatmentHeatingControl_GetEffectiveTargetC(TREATMENT_SIDE_RIGHT,
                                                                              controller) * 100.0f);
        controller->active_heat_right_profile_version = s_heat_right_pid_version;
        break;

    default:
        break;
    }
}

void TreatmentHeatingControl_InitHardware(void)
{
    BspPwmChannelConfig cfg;

    if (s_heat_hw_initialized != 0U)
    {
        return;
    }

    cfg.htim = &htim14;
    cfg.channel = TIM_CHANNEL_1;
    BspPwm_Init(&s_heat_left_pwm, &cfg);

    cfg.htim = &htim17;
    cfg.channel = TIM_CHANNEL_1;
    BspPwm_Init(&s_heat_right_pwm, &cfg);

    (void)BspPwm_Start(&s_heat_left_pwm);
    (void)BspPwm_Start(&s_heat_right_pwm);

    TreatmentHeatingControl_GpioSetModeOutput(OTP1_RESET_GPIO_Port, OTP1_RESET_Pin);
    TreatmentHeatingControl_GpioSetModeOutput(OTP2_RESET_GPIO_Port, OTP2_RESET_Pin);
#if (HEAT_OTP_FAULT_REPORT_ENABLE == 0U)
    HAL_GPIO_WritePin(OTP1_RESET_GPIO_Port, OTP1_RESET_Pin, GPIO_PIN_SET);
    HAL_GPIO_WritePin(OTP2_RESET_GPIO_Port, OTP2_RESET_Pin, GPIO_PIN_SET);
#else
    HAL_GPIO_WritePin(OTP1_RESET_GPIO_Port, OTP1_RESET_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(OTP2_RESET_GPIO_Port, OTP2_RESET_Pin, GPIO_PIN_RESET);
#endif

    s_heat_hw_initialized = 1U;
    s_heat_left_otp_reference_valid = 0U;
    s_heat_right_otp_reference_valid = 0U;
    TreatmentHeatingControl_SetIdleOutputs();
}

void TreatmentHeatingControl_SetIdleOutputs(void)
{
    TreatmentHeatingControl_DisableSide(TREATMENT_SIDE_LEFT);
    TreatmentHeatingControl_DisableSide(TREATMENT_SIDE_RIGHT);
}

void TreatmentHeatingControl_DisableSide(TreatmentSide side)
{
    TreatmentHeatingControl_SetPwm(side, 0U);
    TreatmentHeatingControl_SetPower(side, 0U);
}

void TreatmentHeatingControl_ResetOtp(TreatmentSide side)
{
#if (HEAT_OTP_FAULT_REPORT_ENABLE == 0U)
    (void)side;
    TreatmentHeatingControl_ForceOtpHighAll();
    s_heat_left_otp_reference = HEAT_OTP_NORMAL_LEVEL;
    s_heat_right_otp_reference = HEAT_OTP_NORMAL_LEVEL;
    s_heat_left_otp_reference_valid = 0U;
    s_heat_right_otp_reference_valid = 0U;
    return;
#else
    GPIO_TypeDef *port = (side == TREATMENT_SIDE_LEFT) ? OTP2_RESET_GPIO_Port : OTP1_RESET_GPIO_Port;
    uint16_t pin = (side == TREATMENT_SIDE_LEFT) ? OTP2_RESET_Pin : OTP1_RESET_Pin;

    LOG_I("heater %c otp reset", (side == TREATMENT_SIDE_LEFT) ? 'L' : 'R');
    TreatmentHeatingControl_GpioSetModeOutput(port, pin);
    HAL_GPIO_WritePin(port, pin, GPIO_PIN_SET);
    TreatmentHeatingControl_DelayMs(50U);
    TreatmentHeatingControl_GpioSetModeInput(port, pin);
    if (side == TREATMENT_SIDE_LEFT)
    {
        s_heat_left_otp_reference = HEAT_OTP_NORMAL_LEVEL;
        s_heat_left_otp_reference_valid = 1U;
    }
    else
    {
        s_heat_right_otp_reference = HEAT_OTP_NORMAL_LEVEL;
        s_heat_right_otp_reference_valid = 1U;
    }
    LOG_I("heater %c otp ready pin=%u",
          (side == TREATMENT_SIDE_LEFT) ? 'L' : 'R',
          (unsigned int)((HAL_GPIO_ReadPin(port, pin) == GPIO_PIN_SET) ? 1U : 0U));
#endif
}

void TreatmentHeatingControl_GetOtpFaultFlags(uint8_t *left_fault, uint8_t *right_fault)
{
    uint8_t left = 0U;
    uint8_t right = 0U;

#if (HEAT_OTP_FAULT_REPORT_ENABLE == 0U)
    TreatmentHeatingControl_ForceOtpHighAll();
#else
    if ((s_heat_left_enabled != 0U) &&
        (s_heat_left_otp_reference_valid != 0U))
    {
        left = (uint8_t)(((HAL_GPIO_ReadPin(OTP2_RESET_GPIO_Port, OTP2_RESET_Pin) == GPIO_PIN_SET) ? 1U : 0U) !=
                         HEAT_OTP_NORMAL_LEVEL);
    }

    if ((s_heat_right_enabled != 0U) &&
        (s_heat_right_otp_reference_valid != 0U))
    {
        right = (uint8_t)(((HAL_GPIO_ReadPin(OTP1_RESET_GPIO_Port, OTP1_RESET_Pin) == GPIO_PIN_SET) ? 1U : 0U) !=
                          HEAT_OTP_NORMAL_LEVEL);
    }
#endif

    if (left_fault != NULL)
    {
        *left_fault = left;
    }
    if (right_fault != NULL)
    {
        *right_fault = right;
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
    cfg.integral_max = HEAT_PID_INTEGRAL_MAX;
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

    setpoint = TreatmentHeatingControl_GetEffectiveTargetC(TREATMENT_SIDE_LEFT, controller) * 100.0f;
    PidController_SetSetpoint(&controller->heat_left_pid, setpoint);
    setpoint = TreatmentHeatingControl_GetEffectiveTargetC(TREATMENT_SIDE_RIGHT, controller) * 100.0f;
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

#if (HEAT_OTP_FAULT_REPORT_ENABLE == 0U)
    TreatmentHeatingControl_ForceOtpHighAll();
#endif

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
        if (s_heat_left_otp_reference_valid == 0U)
        {
            TreatmentHeatingControl_ResetOtp(TREATMENT_SIDE_LEFT);
        }
        TreatmentHeatingControl_SetPower(TREATMENT_SIDE_LEFT, 1U);
        heat_output = PidController_ComputeDt(&controller->heat_left_pid,
                                              sensor->tempL * 100.0f,
                                              dt_s);
        runtime->heat_left_pwm = TreatmentHeatingControl_ClampPwm(heat_output, 1999U);
        controller->heat_left_pid.debug.mapped_output = (float)runtime->heat_left_pwm;
        TreatmentHeatingControl_SetPwm(TREATMENT_SIDE_LEFT, runtime->heat_left_pwm);
    }
    else
    {
        TreatmentHeatingControl_SetPwm(TREATMENT_SIDE_LEFT, 0U);
        controller->heat_left_pid.debug.mapped_output = 0.0f;
        TreatmentHeatingControl_SetPower(TREATMENT_SIDE_LEFT, 0U);
    }

    if (controller->cfg.press_enable_R != 0U)
    {
        if (s_heat_right_otp_reference_valid == 0U)
        {
            TreatmentHeatingControl_ResetOtp(TREATMENT_SIDE_RIGHT);
        }
        TreatmentHeatingControl_SetPower(TREATMENT_SIDE_RIGHT, 1U);
        heat_output = PidController_ComputeDt(&controller->heat_right_pid,
                                              sensor->tempR * 100.0f,
                                              dt_s);
        runtime->heat_right_pwm = TreatmentHeatingControl_ClampPwm(heat_output, 1999U);
        controller->heat_right_pid.debug.mapped_output = (float)runtime->heat_right_pwm;
        TreatmentHeatingControl_SetPwm(TREATMENT_SIDE_RIGHT, runtime->heat_right_pwm);
    }
    else
    {
        TreatmentHeatingControl_SetPwm(TREATMENT_SIDE_RIGHT, 0U);
        controller->heat_right_pid.debug.mapped_output = 0.0f;
        TreatmentHeatingControl_SetPower(TREATMENT_SIDE_RIGHT, 0U);
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
