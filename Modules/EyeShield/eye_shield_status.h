#ifndef MODULES_EYESHIELD_EYE_SHIELD_STATUS_H
#define MODULES_EYESHIELD_EYE_SHIELD_STATUS_H

#include <stdint.h>
#include "system_app.h"

#ifdef __cplusplus
extern "C" {
#endif

#ifndef EYE_SHIELD_IGNORE_FUSE_PROTECTION_DEBUG
#define EYE_SHIELD_IGNORE_FUSE_PROTECTION_DEBUG 1
#endif

#define EYE_SHIELD_STATUS_DEBOUNCE_COUNT 3U

void EyeShieldStatus_Init(void);
void EyeShieldStatus_Service(void);
uint8_t EyeShieldStatus_Process(control_config_t *cfg);
void EyeShieldStatus_RequestFuseBlow(uint8_t blow_left, uint8_t blow_right);

#ifdef __cplusplus
}
#endif

#endif
