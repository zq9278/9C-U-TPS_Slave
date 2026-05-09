#ifndef MODULES_LOG_MODULE_LOG_H
#define MODULES_LOG_MODULE_LOG_H

#include <stdio.h>
#include "Modules/Log/module_log_config.h"
#include "Modules/communication/communication.h"

#ifdef __cplusplus
extern "C" {
#endif

/* 模块化日志打印入口，底层默认经 UART1 输出。 */
void ModuleLog_Printf(const char *level, const char *fmt, ...);

#ifndef MODULE_LOG_ENABLED
#define MODULE_LOG_ENABLED MODULE_LOG_GLOBAL_ENABLE
#endif

/* 日志宏按“全局开关 && 当前模块开关”双重控制。 */
#if ((MODULE_LOG_GLOBAL_ENABLE != 0U) && (MODULE_LOG_ENABLED != 0U))
#define LOG_I(...) ModuleLog_Printf("I", __VA_ARGS__)
#define LOG_W(...) ModuleLog_Printf("W", __VA_ARGS__)
#define LOG_E(...) ModuleLog_Printf("E", __VA_ARGS__)
#else
#define LOG_I(...) ((void)0)
#define LOG_W(...) ((void)0)
#define LOG_E(...) ((void)0)
#endif

#ifdef __cplusplus
}
#endif

#endif
