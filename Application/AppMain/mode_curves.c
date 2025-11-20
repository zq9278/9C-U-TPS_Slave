#include "mode_curves.h"
#include <string.h>
#include <stdlib.h>

// Built-in default curves (read-only).
// Format: { target_kpa, t1_rise_s, t2_hold_s, t3_pulse_s, pulse_on_ms, pulse_off_ms }
const ModeCurve_t gModeCurves[4] = {
    // Mode 1: slow rise 1.5s -> hold 2s -> pulse 4s (300/300ms)
    { 25.0f, 1.5f, 2.0f, 4.0f, 300.0f, 300.0f },
    // Mode 2: slow rise 2.0s -> hold 3s -> pulse 5s (250/250ms)
    { 30.0f, 2.0f, 3.0f, 5.0f, 250.0f, 250.0f },
    // Mode 3: default same as Mode 2 (customize if needed)
    { 30.0f, 2.0f, 3.0f, 5.0f, 250.0f, 250.0f },
    // Mode 4: default same as Mode 2 (customize if needed)
    { 30.0f, 2.0f, 3.0f, 5.0f, 250.0f, 250.0f },
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
