#include "upgrade.h"
#include "Config/product_config.h"
/* Binary-scanned, fixed 64-byte record. The packer rejects version overrides
 * that differ from this compiled identity, avoiding false post-update success. */
typedef struct {
    char magic[8];
    uint32_t version;
    uint16_t hardware, reserved;
    uint32_t target, app_base, min_boot, rtt_enabled;
    char version_text[32];
} FirmwareIdentity;
_Static_assert(sizeof(FirmwareIdentity)==64,"identity size");
__attribute__((used,section(".firmware_identity")))
const FirmwareIdentity tps_firmware_identity = {
    {'T','P','S','I','D','0','0','1'}, PRODUCT_FIRMWARE_VERSION_CODE,
    UP_HW_REV,0,UP_TARGET,UP_APP_BASE,UP_BOOT_VERSION,TPS_RTT,
    PRODUCT_FIRMWARE_VERSION_STRING
};
