#ifndef WAVE_CONTROL_H
#define WAVE_CONTROL_H

#include <stdbool.h>
#include <stdint.h>
#include "FreeRTOS.h"
#include "task.h"
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
#define WAVE_CONTROL_RISE_S   1.0f
#define WAVE_CONTROL_HOLD_S   58.0f
#define WAVE_CONTROL_PULSE_S  1.0f

void WaveControl_NormalizeStageMs(uint32_t *rise_ms, uint32_t *hold_ms, uint32_t *pulse_ms);
void WaveControl_ComputeSnapshot(const control_config_t *cfg,
                                 TickType_t wave_anchor_tick,
                                 wave_control_snapshot_t *snapshot);
const char *WaveControl_PhaseName(wave_control_phase_t phase);

#endif /* WAVE_CONTROL_H */
