#ifndef MODULES_HEAT_TREATMENT_HEATING_CONTROL_H
#define MODULES_HEAT_TREATMENT_HEATING_CONTROL_H

#include "treatment_app_controller.h"

#ifdef __cplusplus
extern "C" {
#endif

#ifndef HEAT_OTP_FAULT_REPORT_ENABLE
#define HEAT_OTP_FAULT_REPORT_ENABLE 0U
#endif

typedef enum
{
    TREATMENT_SIDE_LEFT = 0,
    TREATMENT_SIDE_RIGHT = 1
} TreatmentSide;

/* 初始化左右加热 PWM 与 OTP 复位硬件。 */
void TreatmentHeatingControl_InitHardware(void);
/* 关闭左右加热输出，恢复空闲安全态。 */
void TreatmentHeatingControl_SetIdleOutputs(void);
/* 关闭单侧加热输出，供眼罩保护等逻辑复用。 */
void TreatmentHeatingControl_DisableSide(TreatmentSide side);
/* 外部安全逻辑可临时抑制全部加热输出。 */
void TreatmentHeatingControl_SetExternalInhibit(uint8_t enabled);
/* 复位单侧加热 OTP 锁存。 */
void TreatmentHeatingControl_ResetOtp(TreatmentSide side);
/* 检查左右眼 OTP 状态线是否相对复位后的基线发生变化。 */
void TreatmentHeatingControl_GetOtpFaultFlags(uint8_t *left_fault, uint8_t *right_fault);

/* 初始化左右加热 PID。 */
void TreatmentHeatingControl_InitPid(TreatmentAppController *controller);
/* 复位左右加热 PID 运行状态。 */
void TreatmentHeatingControl_ResetPid(TreatmentAppController *controller);
/* 根据当前配置刷新温度设定值。 */
void TreatmentHeatingControl_UpdateSetpoint(TreatmentAppController *controller);
/* 根据当前温度采样执行一次左右加热输出。 */
void TreatmentHeatingControl_ApplyOutputs(TreatmentAppController *controller,
                                          const volatile sensor_data_t *sensor,
                                          float dt_s,
                                          TreatmentAppRuntime *runtime);
/* 修改单侧温度 PID 参数。 */
void TreatmentHeatingControl_SetPidGains(pid_debug_target_t target, float kp, float ki, float kd);
/* 读取单侧温度 PID 参数。 */
void TreatmentHeatingControl_GetPidGains(pid_debug_target_t target, float *kp, float *ki, float *kd);
uint32_t TreatmentHeatingControl_GetPidVersion(pid_debug_target_t target);

#ifdef __cplusplus
}
#endif

#endif
