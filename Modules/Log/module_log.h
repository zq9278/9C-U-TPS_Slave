#ifndef MODULES_LOG_MODULE_LOG_H
#define MODULES_LOG_MODULE_LOG_H

#include <stdio.h>
#include "Modules/communication/communication.h"

#ifdef __cplusplus
extern "C" {
#endif

void ModuleLog_Printf(const char *level, const char *fmt, ...);

#define LOG_I(...) ModuleLog_Printf("I", __VA_ARGS__)
#define LOG_W(...) ModuleLog_Printf("W", __VA_ARGS__)
#define LOG_E(...) ModuleLog_Printf("E", __VA_ARGS__)

#ifdef __cplusplus
}
#endif

#endif
