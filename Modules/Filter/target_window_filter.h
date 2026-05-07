#ifndef MODULES_TARGET_WINDOW_FILTER_H
#define MODULES_TARGET_WINDOW_FILTER_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef struct
{
    float target;
    uint16_t sample_count_limit;
    uint32_t period_ms;
} TargetWindowFilterConfig;

typedef struct
{
    float target;
    float best_value;
    float best_error_abs;
    float last_value;
    uint16_t sample_count;
    uint16_t sample_count_limit;
    uint32_t period_ms;
    uint32_t window_start_ms;
    uint8_t has_sample;
    uint8_t ready;
} TargetWindowFilter;

void TargetWindowFilter_Init(TargetWindowFilter *filter, const TargetWindowFilterConfig *config);
void TargetWindowFilter_Reset(TargetWindowFilter *filter, uint32_t now_ms);
void TargetWindowFilter_SetTarget(TargetWindowFilter *filter, float target);
void TargetWindowFilter_SetWindow(TargetWindowFilter *filter, uint16_t sample_count_limit, uint32_t period_ms);

uint8_t TargetWindowFilter_Push(TargetWindowFilter *filter,
                                float sample,
                                uint32_t now_ms,
                                float *output);
uint8_t TargetWindowFilter_IsReady(const TargetWindowFilter *filter);
uint8_t TargetWindowFilter_GetOutput(TargetWindowFilter *filter, float *output);
float TargetWindowFilter_GetBest(const TargetWindowFilter *filter);

#ifdef __cplusplus
}
#endif

#endif
