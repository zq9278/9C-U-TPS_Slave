#include "app_task.h"

#include "FreeRTOS.h"
#include "queue.h"
#include "task.h"
#include "TreatmentApp.hpp"

#define RTOS_TASK_STATS_LOG_ENABLE 0

#if RTOS_TASK_STATS_LOG_ENABLE
#include "LOG.h"

static const char *taskStateName(eTaskState state)
{
    switch (state) {
        case eRunning:   return "Running";
        case eReady:     return "Ready";
        case eBlocked:   return "Blocked";
        case eSuspended: return "Suspended";
        case eDeleted:   return "Deleted";
        case eInvalid:
        default:         return "Invalid";
    }
}

static void logFreertosTaskStats()
{
    constexpr TickType_t kLogPeriod = pdMS_TO_TICKS(2000);
    constexpr UBaseType_t kMaxCount = 16;
    static TickType_t nextLogTick = 0;
    TaskStatus_t taskStatus[kMaxCount];

    const TickType_t now = xTaskGetTickCount();
    if (static_cast<int32_t>(now - nextLogTick) < 0) {
        return;
    }
    nextLogTick = now + kLogPeriod;

    const UBaseType_t taskCount = uxTaskGetSystemState(taskStatus, kMaxCount, nullptr);
    LOG_I("[RTOS] tasks=%lu free_heap=%lu min_free_heap=%lu",
          static_cast<unsigned long>(taskCount),
          static_cast<unsigned long>(xPortGetFreeHeapSize()),
          static_cast<unsigned long>(xPortGetMinimumEverFreeHeapSize()));
    for (UBaseType_t i = 0; i < taskCount; ++i) {
        LOG_I("[RTOS] %-16s %-10s prio=%lu stack=%lu no=%lu",
              taskStatus[i].pcTaskName,
              taskStateName(taskStatus[i].eCurrentState),
              static_cast<unsigned long>(taskStatus[i].uxCurrentPriority),
              static_cast<unsigned long>(taskStatus[i].usStackHighWaterMark),
              static_cast<unsigned long>(taskStatus[i].xTaskNumber));
    }
}
#else
static void logFreertosTaskStats()
{
}
#endif

extern "C" void AppTask(void *argument)
{
    (void)argument;

    TreatmentApp app;
    app.init();

    for (;;) {
        app_event_t event = {};
        if (xQueueReceive(gAppEventQueue, &event, pdMS_TO_TICKS(10)) == pdPASS) {
            app.handleEvent(event);
        }

        app.service();
        logFreertosTaskStats();
    }
}
