#include "pid_controller.h"

#include <float.h>
#include <stddef.h>
#include <string.h>

static float PidClamp(float value, float min_value, float max_value)
{
    if (value < min_value)
    {
        return min_value;
    }
    if (value > max_value)
    {
        return max_value;
    }
    return value;
}

static float PidResolveDt(const PidController *pid, float dt_s)
{
    if (dt_s > 0.0f)
    {
        return dt_s;
    }
    if ((pid != NULL) && (pid->config.default_dt_s > 0.0f))
    {
        return pid->config.default_dt_s;
    }
    return 1.0f;
}

static void PidNormalizeConfig(PidControllerConfig *config)
{
    float tmp;

    if (config->output_min > config->output_max)
    {
        tmp = config->output_min;
        config->output_min = config->output_max;
        config->output_max = tmp;
    }
    if (config->integral_min > config->integral_max)
    {
        tmp = config->integral_min;
        config->integral_min = config->integral_max;
        config->integral_max = tmp;
    }
    if (config->default_dt_s <= 0.0f)
    {
        config->default_dt_s = 1.0f;
    }
    config->derivative_filter_alpha = PidClamp(config->derivative_filter_alpha, 0.0f, 0.999f);
}

void PidController_Init(PidController *pid, const PidControllerConfig *config)
{
    PidControllerConfig local_config;

    if (pid == NULL)
    {
        return;
    }

    if (config != NULL)
    {
        local_config = *config;
    }
    else
    {
        local_config.kp = 0.0f;
        local_config.ki = 0.0f;
        local_config.kd = 0.0f;
        local_config.setpoint = 0.0f;
        local_config.output_min = -FLT_MAX;
        local_config.output_max = FLT_MAX;
        local_config.integral_min = -FLT_MAX;
        local_config.integral_max = FLT_MAX;
        local_config.default_dt_s = 1.0f;
        local_config.derivative_filter_alpha = 0.0f;
        local_config.derivative_mode = PID_CONTROLLER_DERIVATIVE_ON_MEASUREMENT;
    }

    PidNormalizeConfig(&local_config);
    (void)memset(pid, 0, sizeof(*pid));
    pid->config = local_config;
    pid->debug.setpoint = local_config.setpoint;
}

void PidController_InitDefaults(PidController *pid)
{
    PidController_Init(pid, NULL);
}

void PidController_Reset(PidController *pid)
{
    PidControllerConfig config;

    if (pid == NULL)
    {
        return;
    }

    config = pid->config;
    (void)memset(pid, 0, sizeof(*pid));
    pid->config = config;
    pid->debug.setpoint = config.setpoint;
}

void PidController_SetGains(PidController *pid, float kp, float ki, float kd)
{
    if (pid == NULL)
    {
        return;
    }

    pid->config.kp = kp;
    pid->config.ki = ki;
    pid->config.kd = kd;
}

void PidController_SetSetpoint(PidController *pid, float setpoint)
{
    if (pid == NULL)
    {
        return;
    }

    pid->config.setpoint = setpoint;
    pid->debug.setpoint = setpoint;
}

void PidController_SetOutputLimits(PidController *pid, float min_value, float max_value)
{
    if (pid == NULL)
    {
        return;
    }

    pid->config.output_min = min_value;
    pid->config.output_max = max_value;
    PidNormalizeConfig(&pid->config);
}

void PidController_SetIntegralLimits(PidController *pid, float min_value, float max_value)
{
    if (pid == NULL)
    {
        return;
    }

    pid->config.integral_min = min_value;
    pid->config.integral_max = max_value;
    PidNormalizeConfig(&pid->config);
    pid->integral = PidClamp(pid->integral, pid->config.integral_min, pid->config.integral_max);
}

void PidController_SetDefaultDt(PidController *pid, float default_dt_s)
{
    if (pid == NULL)
    {
        return;
    }

    pid->config.default_dt_s = default_dt_s;
    PidNormalizeConfig(&pid->config);
}

void PidController_SetDerivativeFilterAlpha(PidController *pid, float alpha)
{
    if (pid == NULL)
    {
        return;
    }

    pid->config.derivative_filter_alpha = alpha;
    PidNormalizeConfig(&pid->config);
}

void PidController_SetDerivativeMode(PidController *pid, PidControllerDerivativeMode mode)
{
    if (pid == NULL)
    {
        return;
    }

    pid->config.derivative_mode = mode;
}

float PidController_Compute(PidController *pid, float measurement)
{
    if (pid == NULL)
    {
        return 0.0f;
    }

    return PidController_ComputeDt(pid, measurement, pid->config.default_dt_s);
}

float PidController_ComputeDt(PidController *pid, float measurement, float dt_s)
{
    float dt;
    float error;
    float derivative_raw = 0.0f;
    float output;
    uint8_t limited = 0U;

    if (pid == NULL)
    {
        return 0.0f;
    }

    dt = PidResolveDt(pid, dt_s);
    error = pid->config.setpoint - measurement;

    pid->integral += error * dt;
    pid->integral = PidClamp(pid->integral, pid->config.integral_min, pid->config.integral_max);

    if (pid->has_previous != 0U)
    {
        if (pid->config.derivative_mode == PID_CONTROLLER_DERIVATIVE_ON_ERROR)
        {
            derivative_raw = (error - pid->previous_error) / dt;
        }
        else
        {
            derivative_raw = -(measurement - pid->previous_measurement) / dt;
        }
    }

    pid->derivative_filtered =
        (pid->config.derivative_filter_alpha * pid->derivative_filtered) +
        ((1.0f - pid->config.derivative_filter_alpha) * derivative_raw);

    pid->debug.p = pid->config.kp * error;
    pid->debug.i = pid->config.ki * pid->integral;
    pid->debug.d = pid->config.kd * pid->derivative_filtered;
    output = pid->debug.p + pid->debug.i + pid->debug.d;

    if (output > pid->config.output_max)
    {
        output = pid->config.output_max;
        limited = 1U;
    }
    else if (output < pid->config.output_min)
    {
        output = pid->config.output_min;
        limited = 1U;
    }

    pid->previous_error = error;
    pid->previous_measurement = measurement;
    pid->has_previous = 1U;

    pid->debug.error = error;
    pid->debug.integral = pid->integral;
    pid->debug.derivative = pid->derivative_filtered;
    pid->debug.setpoint = pid->config.setpoint;
    pid->debug.measurement = measurement;
    pid->debug.output = output;
    pid->debug.dt_s = dt;
    pid->debug.output_limited = limited;

    return output;
}
