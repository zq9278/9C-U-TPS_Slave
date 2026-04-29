#ifndef WAVE_CONTROL_H
#define WAVE_CONTROL_H

#include <stdbool.h>
#include <stdint.h>
#include "FreeRTOS.h"
#include "task.h"
#include "pid.h"
#include "system_app.h"

/*
 * 固定 60 秒循环的波形阶段定义。
 * ControlTask 只关心当前属于哪个阶段，以及该阶段对应的目标压力。
 */
typedef enum {
    WAVE_CONTROL_PHASE_IDLE = 0,
    WAVE_CONTROL_PHASE_RISE,
    WAVE_CONTROL_PHASE_HOLD,
    WAVE_CONTROL_PHASE_PULSE_ON,
    WAVE_CONTROL_PHASE_PULSE_OFF,
    WAVE_CONTROL_PHASE_PAUSE,
} wave_control_phase_t;

/*
 * 压力 PID 只按三段治疗阶段切换：
 * RISE / HOLD / PULSE。
 * PULSE_ON 和 PULSE_OFF 都属于同一个 PULSE PID 阶段，避免脉动开关时反复清积分。
 */
typedef enum {
    WAVE_CONTROL_PID_STAGE_RISE = 0,
    WAVE_CONTROL_PID_STAGE_HOLD,
    WAVE_CONTROL_PID_STAGE_PULSE,
} wave_control_pid_stage_t;

/*
 * 波形计算结果：
 * - phase：当前阶段
 * - target_pressure_kpa：当前统一目标压力
 * - inflating_phase：当前是否属于需要补压的阶段
 * - cycle_elapsed_ms：当前在 60 秒循环内已经走过的时间
 * - rise/hold/pulse_ms：三个阶段映射到 60 秒后的实际时长
 */
typedef struct {
    wave_control_phase_t phase;
    float target_pressure_kpa;
    bool inflating_phase;
    uint32_t cycle_elapsed_ms;
    uint32_t rise_ms;
    uint32_t hold_ms;
    uint32_t pulse_ms;
} wave_control_snapshot_t;

/*
 * 压力控制计划：
 * 把当前波形阶段对应的控压动作整理成一份统一结果，
 * 供 ControlTask 直接执行，避免在任务里再分散写一套阶段判断。
 */
typedef struct {
    wave_control_snapshot_t snapshot;
    wave_control_pid_stage_t pid_stage;
    bool open_wave_valve;
    bool pump_enabled;
} wave_control_pressure_plan_t;

/* 一个完整波形循环固定 60 秒。 */
#define WAVE_CONTROL_CYCLE_MS 60000U
/* 脉动挤压频率：改这里即可切换成 2 秒一次或 3 秒一次。 */
#define WAVE_CONTROL_PULSE_PERIOD_MS 2000U
/* 脉动挤压的高压持续时间，默认取半周期。 */
#define WAVE_CONTROL_PULSE_ON_MS (WAVE_CONTROL_PULSE_PERIOD_MS / 2U)

/*
 * 内部波形三段时长（单位：秒），由软件固定，不开放给上位机设置。
 * ControlTask 会将三者按占比归一化到 WAVE_CONTROL_CYCLE_MS 窗口内。
 */
/*
 * 保持重构前的默认波形比例不变。
 * 本次修改的目标只是把压力控制逻辑归拢到 WaveControl，
 * 不在这里顺带修改原有治疗节拍。
 */
#define WAVE_CONTROL_RISE_S   2.0f
#define WAVE_CONTROL_HOLD_S   2.0f
#define WAVE_CONTROL_PULSE_S  56.0f

void WaveControl_NormalizeStageMs(uint32_t *rise_ms, uint32_t *hold_ms, uint32_t *pulse_ms);
void WaveControl_ComputeSnapshot(const control_config_t *cfg,
                                 TickType_t wave_anchor_tick,
                                 wave_control_snapshot_t *snapshot);
void WaveControl_BuildPressurePlan(const control_config_t *cfg,
                                   TickType_t wave_anchor_tick,
                                   wave_control_pressure_plan_t *plan);
void WaveControl_ApplyPressurePidProfile(PID_TypeDef *pid, wave_control_pid_stage_t stage);
void WaveControl_SetPressurePidGains(wave_control_pid_stage_t stage,
                                     float kp,
                                     float ki,
                                     float kd);
void WaveControl_GetPressurePidGains(wave_control_pid_stage_t stage,
                                     float *kp,
                                     float *ki,
                                     float *kd);
uint32_t WaveControl_GetPressurePidVersion(void);
uint16_t WaveControl_ComputePumpPwm(PID_TypeDef *pid,
                                    float target_pressure_kpa,
                                    float feedback_pressure_kpa,
                                    bool pump_enabled);
const char *WaveControl_PhaseName(wave_control_phase_t phase);

#endif /* WAVE_CONTROL_H */
