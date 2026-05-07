#include "target_window_filter.h"

#include <float.h>
#include <stddef.h>
#include <string.h>

static float FilterAbs(float value)
{
    return (value < 0.0f) ? -value : value;
}

static uint8_t FilterWindowExpired(const TargetWindowFilter *filter, uint32_t now_ms)
{
    if ((filter == NULL) || (filter->has_sample == 0U))
    {
        return 0U;
    }

    if ((filter->sample_count_limit > 0U) && (filter->sample_count >= filter->sample_count_limit))
    {
        return 1U;
    }

    if ((filter->period_ms > 0U) && ((uint32_t)(now_ms - filter->window_start_ms) >= filter->period_ms))
    {
        return 1U;
    }

    return 0U;
}

void TargetWindowFilter_Init(TargetWindowFilter *filter, const TargetWindowFilterConfig *config)
{
    if (filter == NULL)
    {
        return;
    }

    (void)memset(filter, 0, sizeof(*filter));
    filter->best_error_abs = FLT_MAX;

    if (config != NULL)
    {
        filter->target = config->target;
        filter->sample_count_limit = config->sample_count_limit;
        filter->period_ms = config->period_ms;
    }
}

void TargetWindowFilter_Reset(TargetWindowFilter *filter, uint32_t now_ms)
{
    float target;
    uint16_t sample_count_limit;
    uint32_t period_ms;

    if (filter == NULL)
    {
        return;
    }

    target = filter->target;
    sample_count_limit = filter->sample_count_limit;
    period_ms = filter->period_ms;
    (void)memset(filter, 0, sizeof(*filter));
    filter->target = target;
    filter->sample_count_limit = sample_count_limit;
    filter->period_ms = period_ms;
    filter->window_start_ms = now_ms;
    filter->best_error_abs = FLT_MAX;
}

void TargetWindowFilter_SetTarget(TargetWindowFilter *filter, float target)
{
    if (filter != NULL)
    {
        filter->target = target;
    }
}

void TargetWindowFilter_SetWindow(TargetWindowFilter *filter, uint16_t sample_count_limit, uint32_t period_ms)
{
    if (filter == NULL)
    {
        return;
    }

    filter->sample_count_limit = sample_count_limit;
    filter->period_ms = period_ms;
}

uint8_t TargetWindowFilter_Push(TargetWindowFilter *filter,
                                float sample,
                                uint32_t now_ms,
                                float *output)
{
    float error_abs;

    if (filter == NULL)
    {
        return 0U;
    }

    if ((filter->has_sample == 0U) && (filter->sample_count == 0U))
    {
        filter->window_start_ms = now_ms;
    }

    filter->last_value = sample;
    error_abs = FilterAbs(sample - filter->target);
    if ((filter->has_sample == 0U) || (error_abs < filter->best_error_abs))
    {
        filter->best_value = sample;
        filter->best_error_abs = error_abs;
    }

    filter->has_sample = 1U;
    if (filter->sample_count < UINT16_MAX)
    {
        ++filter->sample_count;
    }

    if (FilterWindowExpired(filter, now_ms) == 0U)
    {
        return 0U;
    }

    filter->ready = 1U;
    if (output != NULL)
    {
        *output = filter->best_value;
    }
    return 1U;
}

uint8_t TargetWindowFilter_IsReady(const TargetWindowFilter *filter)
{
    return (uint8_t)((filter != NULL) && (filter->ready != 0U));
}

uint8_t TargetWindowFilter_GetOutput(TargetWindowFilter *filter, float *output)
{
    if ((filter == NULL) || (output == NULL) || (filter->ready == 0U))
    {
        return 0U;
    }

    *output = filter->best_value;
    filter->ready = 0U;
    TargetWindowFilter_Reset(filter, filter->window_start_ms + filter->period_ms);
    return 1U;
}

float TargetWindowFilter_GetBest(const TargetWindowFilter *filter)
{
    if ((filter == NULL) || (filter->has_sample == 0U))
    {
        return 0.0f;
    }

    return filter->best_value;
}
