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
//151530

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

/* 将全部治疗输出切回系统级安全空闲态。 */
void TreatmentAppController_SetSystemIdleOutputs(void);
/* 控制器初始化。 */
void TreatmentAppController_Init(TreatmentAppController *controller);
/* 处理来自 AppTask 的控制命令。 */
void TreatmentAppController_HandleCommand(TreatmentAppController *controller,
                                          const ctrl_cmd_t *command,
                                          TickType_t now_tick);
/* 执行一次控制循环。 */
void TreatmentAppController_Run(TreatmentAppController *controller,
                                const volatile sensor_data_t *sensor,
                                TickType_t now_tick,
                                float dt_s,
                                TreatmentAppRuntime *runtime);
/* 设置紧急停机标志。 */
void TreatmentAppController_SetEmergencyStop(TreatmentAppController *controller, uint8_t enabled);
/* 获取当前控制配置。 */
const control_config_t *TreatmentAppController_GetConfig(const TreatmentAppController *controller);
/* 获取压力 PID 调试快照。 */
const PidControllerDebug *TreatmentAppController_GetPressureDebug(const TreatmentAppController *controller);
/* 获取左温度 PID 调试快照。 */
const PidControllerDebug *TreatmentAppController_GetHeatLeftDebug(const TreatmentAppController *controller);
/* 获取右温度 PID 调试快照。 */
const PidControllerDebug *TreatmentAppController_GetHeatRightDebug(const TreatmentAppController *controller);
/* 同时修改单双眼两套压力 PID 某一阶段参数。 */
void TreatmentAppController_SetPressurePidGains(TreatmentPressurePidStage stage,
                                                float kp,
                                                float ki,
                                                float kd);
/* 读取默认压力 PID 参数。 */
void TreatmentAppController_GetPressurePidGains(TreatmentPressurePidStage stage,
                                                float *kp,
                                                float *ki,
                                                float *kd);
uint32_t TreatmentAppController_GetPressurePidVersion(void);
/* 修改温度 PID 参数。 */
void TreatmentAppController_SetHeatPidGains(pid_debug_target_t target,
                                            float kp,
                                            float ki,
                                            float kd);
/* 读取温度 PID 参数。 */
void TreatmentAppController_GetHeatPidGains(pid_debug_target_t target,
                                            float *kp,
                                            float *ki,
                                            float *kd);
uint32_t TreatmentAppController_GetHeatPidVersion(pid_debug_target_t target);

#ifdef __cplusplus
}
#endif

#endif
