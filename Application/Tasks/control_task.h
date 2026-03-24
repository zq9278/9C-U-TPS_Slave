#ifndef CONTROL_TASK_H
#define CONTROL_TASK_H

#include "system_app.h"

void ControlTask(void *argument);

extern volatile uint16_t gPumpPwmDebug;

#endif // CONTROL_TASK_H
