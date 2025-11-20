#ifndef MODE_CURVES_H
#define MODE_CURVES_H

#include <stdint.h>
#include <stdbool.h>
#include "Config/config.h"

typedef struct {
    float target_kpa;    // 目标压力 kPa
    float t1_rise_s;     // 缓慢上升时间 s
    float t2_hold_s;     // 恒压保持时间 s
    float t3_pulse_s;    // 脉动阶段总时长 s
    float pulse_on_ms;   // 脉动 ON 时间 ms
    float pulse_off_ms;  // 脉动 OFF 时间 ms
} ModeCurve_t;

// 只读: 四组内置曲线（常量）
extern const ModeCurve_t gModeCurves[4]; // 模式1..4

// 兼容接口（只读环境下均为 no-op/false）
void ModeCurves_InitFromSettings(const SystemSettings_t *s);
bool ModeCurves_SetFromCSV(uint8_t mode_index_1_based, const char *csv, uint16_t len);

#endif // MODE_CURVES_H
