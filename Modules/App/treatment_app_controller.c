#include "treatment_app_controller.h"

#include <stddef.h>
#include <string.h>

#define MODULE_LOG_ENABLED MODULE_LOG_APP_CONTROLLER_ENABLE
#include "Modules/Heat/treatment_heating_control.h"
#include "Modules/Log/module_log.h"
#include "Modules/Pressure/treatment_pressure_control.h"

void TreatmentAppController_SetSystemIdleOutputs(void)
{
    TreatmentPressureControl_SetIdleOutputs();
    TreatmentHeatingControl_SetIdleOutputs();
}

void TreatmentAppController_Init(TreatmentAppController *controller)
{
    if (controller == NULL)
    {
        return;
    }

    (void)memset(controller, 0, sizeof(*controller));
    TreatmentPressureControl_InitHardware();
    TreatmentHeatingControl_InitHardware();
    TreatmentPressureControl_InitPid(controller);
    TreatmentHeatingControl_InitPid(controller);
    TreatmentAppController_SetSystemIdleOutputs();
}

void TreatmentAppController_HandleCommand(TreatmentAppController *controller,
                                          const ctrl_cmd_t *command,
                                          TickType_t now_tick)
{
    if ((controller == NULL) || (command == NULL))
    {
        return;
    }

    switch (command->id)
    {
    case CTRL_CMD_STOP:
        LOG_I("treat ctrl stop");
        controller->cfg.running = 0U;
        controller->emergency_stop = 0U;
        controller->paused = 0U;
        controller->pause_log_emitted = 0U;
        controller->paused_elapsed_ticks = 0U;
        TreatmentPressureControl_ResetPid(controller);
        TreatmentHeatingControl_ResetPid(controller);
        TreatmentAppController_SetSystemIdleOutputs();
        break;

    case CTRL_CMD_START:
        controller->cfg = command->cfg;
        controller->cfg.running = 1U;
        controller->emergency_stop = 0U;
        controller->paused = 0U;
        controller->pause_log_emitted = 0U;
        controller->paused_elapsed_ticks = 0U;
        controller->wave_anchor_tick = now_tick;
        TreatmentPressureControl_ResetPid(controller);
        TreatmentHeatingControl_ResetPid(controller);
        TreatmentHeatingControl_UpdateSetpoint(controller);
        LOG_I("treat ctrl start tick=%lu temp_sp=%d.%02d L_en=%u R_en=%u",
              (unsigned long)now_tick,
              (int)controller->cfg.temp_target,
              (int)((controller->cfg.temp_target - (float)((int)controller->cfg.temp_target)) * 100.0f),
              controller->cfg.press_enable_L,
              controller->cfg.press_enable_R);
        if (controller->cfg.press_enable_L != 0U)
        {
            TreatmentHeatingControl_ResetOtp(TREATMENT_SIDE_LEFT);
        }
        if (controller->cfg.press_enable_R != 0U)
        {
            TreatmentHeatingControl_ResetOtp(TREATMENT_SIDE_RIGHT);
        }
        break;

    case CTRL_CMD_PAUSE:
        controller->paused = 1U;
        controller->pause_log_emitted = 0U;
        controller->paused_elapsed_ticks = now_tick - controller->wave_anchor_tick;
        LOG_I("treat ctrl pause tick=%lu", (unsigned long)now_tick);
        TreatmentAppController_SetSystemIdleOutputs();
        break;

    case CTRL_CMD_RESUME:
        if (controller->paused != 0U)
        {
            controller->wave_anchor_tick = now_tick - controller->paused_elapsed_ticks;
            controller->paused = 0U;
            controller->pause_log_emitted = 0U;
            LOG_I("treat ctrl resume tick=%lu", (unsigned long)now_tick);
        }
        break;

    case CTRL_CMD_UPDATE_CFG:
        if ((controller->cfg.press_enable_L != command->cfg.press_enable_L) &&
            (command->cfg.press_enable_L != 0U))
        {
            TreatmentHeatingControl_ResetOtp(TREATMENT_SIDE_LEFT);
        }
        if ((controller->cfg.press_enable_R != command->cfg.press_enable_R) &&
            (command->cfg.press_enable_R != 0U))
        {
            TreatmentHeatingControl_ResetOtp(TREATMENT_SIDE_RIGHT);
        }
        controller->cfg = command->cfg;
        TreatmentHeatingControl_UpdateSetpoint(controller);
        break;

    case CTRL_CMD_PID_SET_GAINS:
        switch (command->pid_target)
        {
        case PID_DEBUG_TARGET_PRESS_RISE:
        case PID_DEBUG_TARGET_PRESS_HOLD:
        case PID_DEBUG_TARGET_PRESS_PULSE:
            TreatmentPressureControl_SetPidGainsForController(controller,
                                                              (TreatmentPressurePidStage)command->pid_target,
                                                              command->kp,
                                                              command->ki,
                                                              command->kd);
            LOG_I("pid set target=%u profile=%s kp=%d.%04d ki=%d.%04d kd=%d.%04d",
                  (unsigned int)command->pid_target,
                  ((controller->cfg.press_enable_L != 0U) &&
                   (controller->cfg.press_enable_R != 0U)) ? "dual" : "single",
                  (int)command->kp,
                  (int)((command->kp - (float)((int)command->kp)) * 10000.0f),
                  (int)command->ki,
                  (int)((command->ki - (float)((int)command->ki)) * 10000.0f),
                  (int)command->kd,
                  (int)((command->kd - (float)((int)command->kd)) * 10000.0f));
            break;

        case PID_DEBUG_TARGET_HEAT_LEFT:
        case PID_DEBUG_TARGET_HEAT_RIGHT:
            TreatmentHeatingControl_SetPidGains(command->pid_target,
                                                command->kp,
                                                command->ki,
                                                command->kd);
            LOG_I("pid set target=%u kp=%d.%04d ki=%d.%04d kd=%d.%04d",
                  (unsigned int)command->pid_target,
                  (int)command->kp,
                  (int)((command->kp - (float)((int)command->kp)) * 10000.0f),
                  (int)command->ki,
                  (int)((command->ki - (float)((int)command->ki)) * 10000.0f),
                  (int)command->kd,
                  (int)((command->kd - (float)((int)command->kd)) * 10000.0f));
            break;

        default:
            break;
        }
        break;

    case CTRL_CMD_PID_STREAM_ENABLE:
        controller->pid_debug_stream_enabled = (command->enabled != 0U) ? 1U : 0U;
        LOG_I("pid stream=%u", controller->pid_debug_stream_enabled);
        break;

    case CTRL_CMD_NONE:
    default:
        break;
    }
}

void TreatmentAppController_Run(TreatmentAppController *controller,
                                const volatile sensor_data_t *sensor,
                                TickType_t now_tick,
                                float dt_s,
                                TreatmentAppRuntime *runtime)
{
    TreatmentPressurePlan plan;
    uint8_t should_run;

    if ((controller == NULL) || (sensor == NULL) || (runtime == NULL))
    {
        return;
    }

    (void)memset(runtime, 0, sizeof(*runtime));
    runtime->phase = TREATMENT_PHASE_IDLE;
    runtime->phase_name = TreatmentPressureControl_PhaseName(TREATMENT_PHASE_IDLE);
    runtime->phase_char = (uint8_t)'i';
    runtime->session_active = controller->cfg.running;

    if (controller->paused != 0U)
    {
        if (controller->pause_log_emitted == 0U)
        {
            LOG_W("treat blocked: paused");
            controller->pause_log_emitted = 1U;
        }
        runtime->phase = TREATMENT_PHASE_PAUSE;
        runtime->phase_name = TreatmentPressureControl_PhaseName(TREATMENT_PHASE_PAUSE);
        runtime->phase_char = (uint8_t)'p';
        runtime->session_active = 1U;
        TreatmentAppController_SetSystemIdleOutputs();
        return;
    }

    should_run = (uint8_t)((controller->cfg.running != 0U) &&
                           (controller->emergency_stop == 0U) &&
                           ((controller->cfg.press_enable_L != 0U) ||
                            (controller->cfg.press_enable_R != 0U)));
    if (should_run == 0U)
    {
        static TickType_t s_next_block_log = 0U;

        if ((controller->cfg.running != 0U) &&
            ((int32_t)(now_tick - s_next_block_log) >= 0))
        {
            s_next_block_log = now_tick + pdMS_TO_TICKS(1000U);
            LOG_W("treat blocked run=%u estop=%u L_en=%u R_en=%u",
                  controller->cfg.running,
                  controller->emergency_stop,
                  controller->cfg.press_enable_L,
                  controller->cfg.press_enable_R);
        }
        TreatmentAppController_SetSystemIdleOutputs();
        return;
    }

    TreatmentPressureControl_BuildPlan(controller, now_tick, &plan);
    TreatmentPressureControl_ApplyPlan(controller, sensor, dt_s, &plan, runtime);
    TreatmentHeatingControl_ApplyOutputs(controller, sensor, dt_s, runtime);
}

void TreatmentAppController_SetEmergencyStop(TreatmentAppController *controller, uint8_t enabled)
{
    if (controller == NULL)
    {
        return;
    }

    controller->emergency_stop = (enabled != 0U) ? 1U : 0U;
    if (controller->emergency_stop != 0U)
    {
        TreatmentAppController_SetSystemIdleOutputs();
    }
}

const control_config_t *TreatmentAppController_GetConfig(const TreatmentAppController *controller)
{
    return (controller != NULL) ? &controller->cfg : NULL;
}

const PidControllerDebug *TreatmentAppController_GetPressureDebug(const TreatmentAppController *controller)
{
    return (controller != NULL) ? &controller->pressure_pid.debug : NULL;
}

const PidControllerDebug *TreatmentAppController_GetHeatLeftDebug(const TreatmentAppController *controller)
{
    return (controller != NULL) ? &controller->heat_left_pid.debug : NULL;
}

const PidControllerDebug *TreatmentAppController_GetHeatRightDebug(const TreatmentAppController *controller)
{
    return (controller != NULL) ? &controller->heat_right_pid.debug : NULL;
}

void TreatmentAppController_SetPressurePidGains(TreatmentPressurePidStage stage,
                                                float kp,
                                                float ki,
                                                float kd)
{
    TreatmentPressureControl_SetPidGains(stage, kp, ki, kd);
}

void TreatmentAppController_GetPressurePidGains(TreatmentPressurePidStage stage,
                                                float *kp,
                                                float *ki,
                                                float *kd)
{
    TreatmentPressureControl_GetPidGains(stage, kp, ki, kd);
}

uint32_t TreatmentAppController_GetPressurePidVersion(void)
{
    return TreatmentPressureControl_GetPidVersion();
}

void TreatmentAppController_SetHeatPidGains(pid_debug_target_t target,
                                            float kp,
                                            float ki,
                                            float kd)
{
    TreatmentHeatingControl_SetPidGains(target, kp, ki, kd);
}

void TreatmentAppController_GetHeatPidGains(pid_debug_target_t target,
                                            float *kp,
                                            float *ki,
                                            float *kd)
{
    TreatmentHeatingControl_GetPidGains(target, kp, ki, kd);
}

uint32_t TreatmentAppController_GetHeatPidVersion(pid_debug_target_t target)
{
    return TreatmentHeatingControl_GetPidVersion(target);
}
