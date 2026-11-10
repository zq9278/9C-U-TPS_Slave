#ifndef TPS_BOOT_ENGINE_H
#define TPS_BOOT_ENGINE_H
#include "Shared/Upgrade/upgrade.h"
typedef struct {
    const uint8_t *(*read)(uint32_t address);
    int (*erase)(uint32_t address, uint32_t pages);
    int (*write)(uint32_t address, const uint8_t *data, uint32_t size);
    /* Optional latched ECC/read fault check. */
    int (*healthy)(void);
} BootFlash;
typedef struct {
    BootFlash flash;
    uint32_t session, offset;
    uint16_t last_seq;
    uint8_t state, app_valid, reset_requested;
    uint8_t header[64], last_request[UP_MAX_FRAME], last_response[UP_MAX_FRAME];
    size_t last_request_size, last_response_size;
    uint8_t hello_request[4], hello_response[64];
    uint16_t hello_seq;
    size_t hello_size;
    uint8_t response[UP_MAX_FRAME];
} BootEngine;
int Boot_ValidApp(const BootFlash *flash);
void Boot_Init(BootEngine *b, BootFlash flash);
size_t Boot_Handle(BootEngine *b, const UpFrame *f);
#endif
