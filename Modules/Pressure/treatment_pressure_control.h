#ifndef MODULES_PRESSURE_TREATMENT_PRESSURE_CONTROL_H
#define MODULES_PRESSURE_TREATMENT_PRESSURE_CONTROL_H

#include "treatment_app_controller.h"

#ifdef __cplusplus
extern "C" {
#endif

/* 初始化气泵 PWM 与阀路执行器。 */
void TreatmentPressureControl_InitHardware(void);
/* 将压力通道恢复到放气且气泵关闭的安全态。 */
void TreatmentPressureControl_SetIdleOutputs(void);

/**
 * @brief 单个控制周期内的压力执行计划。
 *
 * BuildPlan 负责计算该结构，ApplyPlan 负责执行该结构。
 */
typedef struct
{
    TreatmentPhase phase;
    TreatmentPressurePidStage pid_stage;
    uint8_t inflating;
    float target_pressure_kpa;
    uint32_t cycle_elapsed_ms;
} TreatmentPressurePlan;

/* 初始化压力 PID 默认配置。 */
void TreatmentPressureControl_InitPid(TreatmentAppController *controller);
/* 复位压力控制器运行状态。 */
void TreatmentPressureControl_ResetPid(TreatmentAppController *controller);
/* 根据当前配置与当前时间计算本周期的压力阶段计划。 */
void TreatmentPressureControl_BuildPlan(const TreatmentAppController *controller,
                                        TickType_t now_tick,
                                        TreatmentPressurePlan *plan);
/* 执行本周期压力计划并更新 runtime。 */
void TreatmentPressureControl_ApplyPlan(TreatmentAppController *controller,
                                        const volatile sensor_data_t *sensor,
                                        float dt_s,
                                        const TreatmentPressurePlan *plan,
                                        TreatmentAppRuntime *runtime);
/* 将阶段枚举转换为便于上位机识别的字符串。 */
const char *TreatmentPressureControl_PhaseName(TreatmentPhase phase);
/* 按当前单双眼治疗模式修改对应压力 PID 参数。 */
void TreatmentPressureControl_SetPidGainsForController(const TreatmentAppController *controller,
                                                       TreatmentPressurePidStage stage,
                                                       float kp,
                                                       float ki,
                                                       float kd);
/* 按当前单双眼治疗模式读取对应压力 PID 参数。 */
void TreatmentPressureControl_GetPidGainsForController(const TreatmentAppController *controller,
                                                       TreatmentPressurePidStage stage,
                                                       float *kp,
                                                       float *ki,
                                                       float *kd);
/* 同时修改单双眼两套压力 PID 参数。 */
void TreatmentPressureControl_SetPidGains(TreatmentPressurePidStage stage, float kp, float ki, float kd);
/* 读取默认压力 PID 参数。 */
void TreatmentPressureControl_GetPidGains(TreatmentPressurePidStage stage, float *kp, float *ki, float *kd);
uint32_t TreatmentPressureControl_GetPidVersion(void);

#ifdef __cplusplus
}
#endif

#endif
