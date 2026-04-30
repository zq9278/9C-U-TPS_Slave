#ifndef HEATER_SHIELD_STATUS_H
#define HEATER_SHIELD_STATUS_H

#include <stdint.h>
#include "system_app.h"

#ifdef __cplusplus
extern "C" {
#endif

/*
 * 调试开关：
 * 1 = 忽略熔断后的自动关停，只上报状态，方便联调
 * 0 = 熔断时执行正常保护动作
 */
#ifndef HEATER_SHIELD_IGNORE_FUSE_PROTECTION_DEBUG
#define HEATER_SHIELD_IGNORE_FUSE_PROTECTION_DEBUG 1
#endif

/*
 * 消抖次数：
 * 控制循环约 2ms 一次，当前模块约 100ms 调一次；
 * 连续 3 次状态一致后，才更新最终稳定值。
 */
#define HEATER_SHIELD_STATUS_DEBOUNCE_COUNT 3U

void HeaterShieldStatus_Init(void);
void HeaterShieldStatus_Service(void);
void HeaterShieldStatus_Process(control_config_t *cfg);
void HeaterShieldStatus_RequestFuseBlow(uint8_t blow_left, uint8_t blow_right);

#ifdef __cplusplus
}
#endif

#endif /* HEATER_SHIELD_STATUS_H */
