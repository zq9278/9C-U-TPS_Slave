#include "mode_curves.h"
#include <string.h>
#include <stdlib.h>

// Built-in default curves (read-only).
// Format: { target_kpa, t1_rise_s, t2_hold_s, t3_pulse_s, pulse_on_ms, pulse_off_ms }
const ModeCurve_t gModeCurves[4] = { 
    // Mode 1: t1=25s, t2=20s, t3=15s -> total 60s,柔和脉动
    { 0.0f, 5.0f, 5.0f, 50.0f, 2000.0f, 1000.0f },
    // Mode 2: t1=20s, t2=20s, t3=20s -> total 60s,均衡脉动
    { 0.0f, 20.0f, 20.0f, 20.0f, 400.0f, 400.0f },
    // Mode 3: t1=15s, t2=25s, t3=20s -> total 60s,更长保持
    { 0.0f, 15.0f, 25.0f, 20.0f, 350.0f, 350.0f },
    // Mode 4: t1=10s, t2=30s, t3=20s -> total 60s,快速上升
    { 0.0f, 10.0f, 30.0f, 20.0f, 300.0f, 300.0f },
};

void ModeCurves_InitFromSettings(const SystemSettings_t *s)
{
    (void)s; // Read-only: no-op
}

bool ModeCurves_SetFromCSV(uint8_t mode_index_1_based, const char *csv, uint16_t len)
{
    (void)mode_index_1_based; (void)csv; (void)len;
    // Read-only: reject updates
    return false;
}
