#ifndef MODULES_APP_TREATMENT_APP_CONTROLLER_H
#define MODULES_APP_TREATMENT_APP_CONTROLLER_H

#include <stdint.h>
#include "FreeRTOS.h"
#include "pid_controller.h"
#include "system_app.h"

#ifdef __cplusplus
extern "C" {
#endif

#define TREATMENT_CYCLE_MS 60000U
#define TREATMENT_DEFAULT_PULSE_PERIOD_MS 2000U
#define TREATMENT_DEFAULT_PULSE_ON_MS (TREATMENT_DEFAULT_PULSE_PERIOD_MS / 2U)
#define TREATMENT_DEFAULT_RISE_S 15.0f
#define TREATMENT_DEFAULT_HOLD_S 15.0f
#define TREATMENT_DEFAULT_PULSE_S 30.0f

typedef enum
{
    TREATMENT_PHASE_IDLE = 0,
    TREATMENT_PHASE_RISE,
    TREATMENT_PHASE_HOLD,
    TREATMENT_PHASE_PULSE_ON,
    TREATMENT_PHASE_PULSE_OFF,
    TREATMENT_PHASE_PAUSE
} TreatmentPhase;

typedef enum
{
    TREATMENT_PRESS_PID_STAGE_RISE = 0,
    TREATMENT_PRESS_PID_STAGE_HOLD,
    TREATMENT_PRESS_PID_STAGE_PULSE
} TreatmentPressurePidStage;

typedef struct
{
    TreatmentPhase phase;
    TreatmentPressurePidStage pid_stage;
    const char *phase_name;
    uint8_t phase_char;
    uint8_t session_active;
    uint8_t running_outputs;
    float target_pressure_kpa;
    float feedback_pressure_kpa;
    uint16_t pump_pwm;
    uint16_t heat_left_pwm;
    uint16_t heat_right_pwm;
    uint32_t cycle_elapsed_ms;
} TreatmentAppRuntime;

typedef struct
{
    control_config_t cfg;
    PidController pressure_pid;
    PidController heat_left_pid;
    PidController heat_right_pid;
    TreatmentPressurePidStage active_pressure_stage;
    uint8_t active_pressure_profile_kind;
    uint32_t active_pressure_profile_version;
    uint32_t active_heat_left_profile_version;
    uint32_t active_heat_right_profile_version;
    TickType_t wave_anchor_tick;
    TickType_t paused_elapsed_ticks;
    TickType_t pressure_vent_start_tick;
    uint32_t last_pressure_cycle_elapsed_ms;
    uint8_t paused;
    uint8_t pause_log_emitted;
    uint8_t pressure_profile_loaded;
    uint8_t pressure_cycle_seen;
    uint8_t pressure_cycle_venting;
    uint8_t emergency_stop;
    uint8_t pid_debug_stream_enabled;
    TreatmentPhase last_pressure_phase;
} TreatmentAppController;

void TreatmentAppController_Init(TreatmentAppController *controller);
void TreatmentAppController_HandleCommand(TreatmentAppController *controller,
                                          const ctrl_cmd_t *command,
                                          TickType_t now_tick);
void TreatmentAppController_Run(TreatmentAppController *controller,
                                const volatile sensor_data_t *sensor,
                                TickType_t now_tick,
                                float dt_s,
                                TreatmentAppRuntime *runtime);
void TreatmentAppController_SetEmergencyStop(TreatmentAppController *controller, uint8_t enabled);
const control_config_t *TreatmentAppController_GetConfig(const TreatmentAppController *controller);
const PidControllerDebug *TreatmentAppController_GetPressureDebug(const TreatmentAppController *controller);
const PidControllerDebug *TreatmentAppController_GetHeatLeftDebug(const TreatmentAppController *controller);
const PidControllerDebug *TreatmentAppController_GetHeatRightDebug(const TreatmentAppController *controller);
void TreatmentAppController_SetPressurePidGains(TreatmentPressurePidStage stage,
                                                float kp,
                                                float ki,
                                                float kd);
void TreatmentAppController_GetPressurePidGains(TreatmentPressurePidStage stage,
                                                float *kp,
                                                float *ki,
                                                float *kd);
uint32_t TreatmentAppController_GetPressurePidVersion(void);
void TreatmentAppController_SetHeatPidGains(pid_debug_target_t target,
                                            float kp,
                                            float ki,
                                            float kd);
void TreatmentAppController_GetHeatPidGains(pid_debug_target_t target,
                                            float *kp,
                                            float *ki,
                                            float *kd);
uint32_t TreatmentAppController_GetHeatPidVersion(pid_debug_target_t target);

#ifdef __cplusplus
}
#endif

#endif
