#include "TreatmentActuators.h"

#include "BSP/Pwm/bsp_pwm.h"
#define MODULE_LOG_ENABLED MODULE_LOG_ACTUATOR_ENABLE
#include "Modules/Log/module_log.h"
#include "UserDrivers/ValveControl/valve_control.h"
#include "main.h"
#include "FreeRTOS.h"
#include "task.h"
#include "tim.h"

static BspPwmChannel s_pump_pwm;
static BspPwmChannel s_heat_left_pwm;
static BspPwmChannel s_heat_right_pwm;
static uint8_t s_initialized = 0U;
static uint8_t s_heat_left_enabled = 0U;
static uint8_t s_heat_right_enabled = 0U;
static uint16_t s_heat_left_last_pwm = 0U;
static uint16_t s_heat_right_last_pwm = 0U;

static uint32_t pin_to_index(uint16_t pin)
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

static void gpio_set_mode_input(GPIO_TypeDef *port, uint16_t pin)
{
    uint32_t shift = pin_to_index(pin) * 2U;
    port->MODER &= ~(0x3UL << shift);
}

static void gpio_set_mode_output(GPIO_TypeDef *port, uint16_t pin)
{
    uint32_t shift = pin_to_index(pin) * 2U;
    port->MODER &= ~(0x3UL << shift);
    port->MODER |= (0x1UL << shift);
}

static void actuator_delay_ms(uint32_t delay_ms)
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

static uint16_t clamp_u16(uint16_t value, uint16_t max_value)
{
    return (value > max_value) ? max_value : value;
}

void TreatmentActuators_Init(void)
{
    BspPwmChannelConfig cfg;

    if (s_initialized != 0U)
    {
        return;
    }

    cfg.htim = &htim15;
    cfg.channel = TIM_CHANNEL_1;
    BspPwm_Init(&s_pump_pwm, &cfg);

    cfg.htim = &htim14;
    cfg.channel = TIM_CHANNEL_1;
    BspPwm_Init(&s_heat_left_pwm, &cfg);

    cfg.htim = &htim17;
    cfg.channel = TIM_CHANNEL_1;
    BspPwm_Init(&s_heat_right_pwm, &cfg);

    (void)BspPwm_Start(&s_pump_pwm);
    (void)BspPwm_Start(&s_heat_left_pwm);
    (void)BspPwm_Start(&s_heat_right_pwm);

    HAL_GPIO_WritePin(OTP1_RESET_GPIO_Port, OTP1_RESET_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(OTP2_RESET_GPIO_Port, OTP2_RESET_Pin, GPIO_PIN_RESET);
    gpio_set_mode_output(OTP1_RESET_GPIO_Port, OTP1_RESET_Pin);
    gpio_set_mode_output(OTP2_RESET_GPIO_Port, OTP2_RESET_Pin);

    s_initialized = 1U;
    TreatmentActuators_SetIdle();
}

void TreatmentActuators_SetIdle(void)
{
    TreatmentActuators_SetPumpPwm(0U);
    TreatmentActuators_SetHeaterPwm(TREATMENT_SIDE_LEFT, 0U);
    TreatmentActuators_SetHeaterPwm(TREATMENT_SIDE_RIGHT, 0U);
    TreatmentActuators_SetHeaterPower(TREATMENT_SIDE_LEFT, 0U);
    TreatmentActuators_SetHeaterPower(TREATMENT_SIDE_RIGHT, 0U);
    ValveControl_SetIdle();
}

void TreatmentActuators_ApplyPressureRoute(uint8_t enable_left, uint8_t enable_right)
{
    ValveControl_ApplyTreatmentRoute(enable_left, enable_right);
}

void TreatmentActuators_SetPressureVentAll(void)
{
    ValveControl_SetVentAll();
}

void TreatmentActuators_SetWaveValve(uint8_t enabled)
{
    ValveControl_SetWave(enabled);
}

void TreatmentActuators_SetPumpPwm(uint16_t pwm)
{
    BspPwm_SetPulse(&s_pump_pwm, clamp_u16(pwm, 255U));
}

void TreatmentActuators_SetHeaterPower(TreatmentSide side, uint8_t enabled)
{
    if (side == TREATMENT_SIDE_LEFT)
    {
        if (enabled != 0U)
        {
            if (BspPwm_Start(&s_heat_left_pwm) == 0U)
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
            BspPwm_SetPulse(&s_heat_left_pwm, 0U);
            if (BspPwm_Stop(&s_heat_left_pwm) == 0U)
            {
                LOG_E("heater L pwm stop failed");
            }
            gpio_set_mode_output(OTP1_RESET_GPIO_Port, OTP1_RESET_Pin);
            HAL_GPIO_WritePin(OTP1_RESET_GPIO_Port, OTP1_RESET_Pin, GPIO_PIN_RESET);
            if (s_heat_left_enabled != 0U)
            {
                LOG_I("heater L power off");
            }
            s_heat_left_enabled = 0U;
        }
    }
    else
    {
        if (enabled != 0U)
        {
            if (BspPwm_Start(&s_heat_right_pwm) == 0U)
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
            BspPwm_SetPulse(&s_heat_right_pwm, 0U);
            if (BspPwm_Stop(&s_heat_right_pwm) == 0U)
            {
                LOG_E("heater R pwm stop failed");
            }
            gpio_set_mode_output(OTP2_RESET_GPIO_Port, OTP2_RESET_Pin);
            HAL_GPIO_WritePin(OTP2_RESET_GPIO_Port, OTP2_RESET_Pin, GPIO_PIN_RESET);
            if (s_heat_right_enabled != 0U)
            {
                LOG_I("heater R power off");
            }
            s_heat_right_enabled = 0U;
        }
    }
}

void TreatmentActuators_SetHeaterPwm(TreatmentSide side, uint16_t pwm)
{
    uint16_t clamped_pwm = clamp_u16(pwm, 1999U);

    if (side == TREATMENT_SIDE_LEFT)
    {
        if (clamped_pwm != 0U)
        {
            if (BspPwm_Start(&s_heat_left_pwm) == 0U)
            {
                LOG_E("heater L pwm start failed while set pwm=%u", clamped_pwm);
            }
        }
        BspPwm_SetPulse(&s_heat_left_pwm, clamped_pwm);
        if (((s_heat_left_last_pwm == 0U) && (clamped_pwm != 0U)) ||
            ((s_heat_left_last_pwm != 0U) && (clamped_pwm == 0U)))
        {
            LOG_I("heater L pwm=%u", clamped_pwm);
        }
        s_heat_left_last_pwm = clamped_pwm;
    }
    else
    {
        if (clamped_pwm != 0U)
        {
            if (BspPwm_Start(&s_heat_right_pwm) == 0U)
            {
                LOG_E("heater R pwm start failed while set pwm=%u", clamped_pwm);
            }
        }
        BspPwm_SetPulse(&s_heat_right_pwm, clamped_pwm);
        if (((s_heat_right_last_pwm == 0U) && (clamped_pwm != 0U)) ||
            ((s_heat_right_last_pwm != 0U) && (clamped_pwm == 0U)))
        {
            LOG_I("heater R pwm=%u", clamped_pwm);
        }
        s_heat_right_last_pwm = clamped_pwm;
    }
}

void TreatmentActuators_ResetHeaterOtp(TreatmentSide side)
{
    GPIO_TypeDef *port = (side == TREATMENT_SIDE_LEFT) ? OTP1_RESET_GPIO_Port : OTP2_RESET_GPIO_Port;
    uint16_t pin = (side == TREATMENT_SIDE_LEFT) ? OTP1_RESET_Pin : OTP2_RESET_Pin;

    LOG_I("heater %c otp reset", (side == TREATMENT_SIDE_LEFT) ? 'L' : 'R');
    gpio_set_mode_output(port, pin);
    HAL_GPIO_WritePin(port, pin, GPIO_PIN_SET);
    actuator_delay_ms(50U);
    gpio_set_mode_input(port, pin);
}
