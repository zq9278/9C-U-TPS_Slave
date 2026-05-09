#ifndef MODULES_EYESHIELD_EYE_SHIELD_STATUS_H
#define MODULES_EYESHIELD_EYE_SHIELD_STATUS_H

#include <stdint.h>
#include "system_app.h"

#ifdef __cplusplus
extern "C" {
#endif

/* 调试宏：为 1 时忽略保险丝故障强停，仅保留日志提示。 */
#ifndef EYE_SHIELD_IGNORE_FUSE_PROTECTION_DEBUG
#define EYE_SHIELD_IGNORE_FUSE_PROTECTION_DEBUG 1
#endif

/* 在位/保险丝状态去抖阈值。 */
#define EYE_SHIELD_PRESENT_DEBOUNCE_COUNT 5U
#define EYE_SHIELD_FUSE_DEBOUNCE_COUNT    3U

/* 初始化眼罩状态检测模块。 */
void EyeShieldStatus_Init(void);
/* 处理保险丝击穿脉冲等短时动作。 */void EyeShieldStatus_Service(void);
/* 采样在位/保险丝/保护器 ADC 状态，必要时修改 cfg 并请求停机。 */
uint8_t EyeShieldStatus_Process(control_config_t *cfg, stop_reason_t *stop_reason);
/* 请求触发左/右侧保险丝熔断脉冲。 */
void EyeShieldStatus_RequestFuseBlow(uint8_t blow_left, uint8_t blow_right);

#ifdef __cplusplus
}
#endif

#endif
