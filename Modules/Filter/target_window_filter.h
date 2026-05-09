#ifndef MODULES_TARGET_WINDOW_FILTER_H
#define MODULES_TARGET_WINDOW_FILTER_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* 目标窗口滤波器初始化参数。 */
typedef struct
{
    float target;
    uint16_t sample_count_limit;
    uint32_t period_ms;
} TargetWindowFilterConfig;

/*
 * 该滤波器不做平均，而是在一个时间窗/样本窗内选择
 * “距离 target 最近”的样本，适合做带目标约束的平滑输出。
 */
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

/* 初始化滤波器。 */
void TargetWindowFilter_Init(TargetWindowFilter *filter, const TargetWindowFilterConfig *config);
/* 重置窗口状态但保留配置。 */
void TargetWindowFilter_Reset(TargetWindowFilter *filter, uint32_t now_ms);
/* 设置目标值。 */
void TargetWindowFilter_SetTarget(TargetWindowFilter *filter, float target);
/* 设置窗口长度。 */
void TargetWindowFilter_SetWindow(TargetWindowFilter *filter, uint16_t sample_count_limit, uint32_t period_ms);

/* 推入一个样本，若窗口到期则返回 1。 */
uint8_t TargetWindowFilter_Push(TargetWindowFilter *filter,
                                float sample,
                                uint32_t now_ms,
                                float *output);
/* 判断是否已有待取输出。 */
uint8_t TargetWindowFilter_IsReady(const TargetWindowFilter *filter);
/* 取出窗口输出。 */
uint8_t TargetWindowFilter_GetOutput(TargetWindowFilter *filter, float *output);
/* 获取当前窗口内最佳样本。 */
float TargetWindowFilter_GetBest(const TargetWindowFilter *filter);

#ifdef __cplusplus
}
#endif

#endif
