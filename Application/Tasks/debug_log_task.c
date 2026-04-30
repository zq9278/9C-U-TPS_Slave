#include "FreeRTOS.h"
#include "task.h"
#include "system_app.h"
#include "uart_driver.h"
#include "LOG.h"
#include "WaveControl/wave_control.h"
#include <stdlib.h>

extern UartPort_t debug_uart_port;

static const char *skip_spaces(const char *s)
{
    while ((*s == ' ') || (*s == '\t') || (*s == ',') || (*s == ':')) {
        s++;
    }
    return s;
}

static bool parse_float_arg(const char **cursor, float *value)
{
    char *end = NULL;
    const char *p = skip_spaces(*cursor);

    *value = strtof(p, &end);
    if (end == p) {
        return false;
    }

    *cursor = end;
    return true;
}

static bool debug_pid_stage_from_char(char ch, wave_control_pid_stage_t *stage, const char **name)
{
    switch (ch) {
    case 'R':
    case 'r':
        *stage = WAVE_CONTROL_PID_STAGE_RISE;
        *name = "RISE";
        return true;
    case 'H':
    case 'h':
        *stage = WAVE_CONTROL_PID_STAGE_HOLD;
        *name = "HOLD";
        return true;
    case 'P':
    case 'p':
        *stage = WAVE_CONTROL_PID_STAGE_PULSE;
        *name = "PULSE";
        return true;
    default:
        return false;
    }
}

static void handle_debug_ascii_command(const uint8_t *data, uint16_t length)
{
    char line[ASCII_CMD_MAX_LEN];
    const char *cursor;
    wave_control_pid_stage_t stage;
    const char *stage_name = NULL;
    float kp = 0.0f;
    float ki = 0.0f;
    float kd = 0.0f;
    uint16_t copy_len = length;

    if (copy_len >= ASCII_CMD_MAX_LEN) {
        copy_len = ASCII_CMD_MAX_LEN - 1U;
    }
    for (uint16_t i = 0; i < copy_len; ++i) {
        char ch = (char)data[i];
        line[i] = ((ch == '\r') || (ch == '\n')) ? '\0' : ch;
    }
    line[copy_len] = '\0';

    cursor = skip_spaces(line);
    if (!debug_pid_stage_from_char(*cursor, &stage, &stage_name)) {
        LOG_W("[PID] cmd format: R/H/P kp ki kd");
        return;
    }
    cursor++;

    if (!parse_float_arg(&cursor, &kp) ||
        !parse_float_arg(&cursor, &ki) ||
        !parse_float_arg(&cursor, &kd)) {
        LOG_W("[PID] parse failed, use: %c kp ki kd", stage_name[0]);
        return;
    }

    WaveControl_SetPressurePidGains(stage, kp, ki, kd);
    LOG_I("[PID] %s set Kp=%.3f Ki=%.3f Kd=%.3f", stage_name, (double)kp, (double)ki, (double)kd);
}

void DebugLogTask(void *argument)
{
    UartRxMessage_t rx_msg;

    (void)argument;

    for (;;)
    {
        while (xQueueReceive(debug_uart_port.rx_queue, &rx_msg, 0) == pdPASS) {
            handle_debug_ascii_command(rx_msg.data, rx_msg.length);
        }

        vTaskDelay(pdMS_TO_TICKS(20));
    }
}
