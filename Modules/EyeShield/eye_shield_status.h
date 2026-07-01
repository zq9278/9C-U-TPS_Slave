#ifndef MODULES_EYESHIELD_EYE_SHIELD_STATUS_H
#define MODULES_EYESHIELD_EYE_SHIELD_STATUS_H

#include <stdint.h>
#include "App/System/app_safety_config.h"
#include "system_app.h"

#ifdef __cplusplus
extern "C" {
#endif

/* Debounce thresholds for eye-shield present and fuse status inputs. */
#define EYE_SHIELD_PRESENT_DEBOUNCE_COUNT 5U
#define EYE_SHIELD_FUSE_DEBOUNCE_COUNT    3U

void EyeShieldStatus_Init(void);
void EyeShieldStatus_Service(void);
uint8_t EyeShieldStatus_Process(control_config_t *cfg, stop_reason_t *stop_reason);
void EyeShieldStatus_RequestFuseBlow(uint8_t blow_left, uint8_t blow_right);

#ifdef __cplusplus
}
#endif

#endif
