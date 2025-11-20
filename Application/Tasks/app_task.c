#include <string.h>
#include "app_task.h"
#include "LOG.h"
#include "config.h"
#include "Uart_Communicate.h"
#include "AppMain/mode_curves.h"

extern SystemSettings_t g_settings;
// Runtime flags for per-side pressure enable (not persisted across reboots)
uint8_t g_press_enable_L = 1;
uint8_t g_press_enable_R = 1;
// Runtime flags for per-side temperature enable (not persisted)
uint8_t g_temp_enable_L  = 1;
uint8_t g_temp_enable_R  = 1;
// Runtime squeeze mode (0 normal, 1 alternate, 2 sync)
static uint8_t g_squeeze_mode = 0;
// Single-mode runtime curve (start from built-in mode 1)
static ModeCurve_t gCurveRT;

static void send_control_cfg(uint8_t mode, uint8_t running)
{
    ctrl_cmd_t c = {0};
    c.id = running ? CTRL_CMD_START : CTRL_CMD_STOP;
    (void)mode; c.cfg.mode = 1; // single fixed mode
    c.cfg.running = running;
    // Common temp target
    c.cfg.temp_target = g_settings.use_common_temp ? g_settings.left_temp_c : g_settings.left_temp_c;
    // Use runtime curve (overrides allowed)
    c.cfg.press_target_max = gCurveRT.target_kpa;
    c.cfg.t1_rise_s        = gCurveRT.t1_rise_s;
    c.cfg.t2_hold_s        = gCurveRT.t2_hold_s;
    c.cfg.t3_pulse_s       = gCurveRT.t3_pulse_s;
    c.cfg.pulse_on_ms      = gCurveRT.pulse_on_ms;
    c.cfg.pulse_off_ms     = gCurveRT.pulse_off_ms;
    c.cfg.squeeze_mode     = g_squeeze_mode;
    // Pressure side enable flags come from local runtime state
    c.cfg.press_enable_L   = g_press_enable_L;
    c.cfg.press_enable_R   = g_press_enable_R;
    (void)xQueueSend(gCtrlCmdQueue, &c, 0);
}

void AppTask(void *argument)
{
    (void)argument;
    app_cmd_t cmd;
    uint32_t save_due_tick = 0;
    uint8_t current_mode = 1; // single fixed mode
    // init runtime curve from built-in
    gCurveRT = gModeCurves[0];
    gAppState = APP_STATE_IDLE;
    // g_press_enable_L/R already defined at file scope

    for(;;)
    {
        // Handle incoming commands
        if (xQueueReceive(gCmdQueue, &cmd, pdMS_TO_TICKS(10)) == pdPASS)
        {
            switch (cmd.id)
            {
                case APP_CMD_MODE_SELECT:
                    // Ignore (single mode). Keep ready state unless running.
                    current_mode = 1; g_settings.mode_select = 1;
                    break;

                case APP_CMD_START:
                    gAppState = (current_mode==1)?APP_STATE_RUN_MODE1:APP_STATE_RUN_MODE2;
                    send_control_cfg(current_mode, 1);
                    break;

                case APP_CMD_STOP:
                    send_control_cfg(current_mode, 0);
                    gAppState = APP_STATE_READY;
                    break;

                case APP_CMD_SET_TEMP_LEFT:
                    g_settings.left_temp_c = cmd.v.f32;
                    if (g_settings.use_common_temp) g_settings.right_temp_c = g_settings.left_temp_c;
                    save_due_tick = xTaskGetTickCount() + pdMS_TO_TICKS(3000);
                    break;

                case APP_CMD_SET_TEMP_RIGHT:
                    g_settings.right_temp_c = cmd.v.f32;
                    save_due_tick = xTaskGetTickCount() + pdMS_TO_TICKS(3000);
                    break;

                case APP_CMD_SET_USE_COMMON_TEMP:
                    g_settings.use_common_temp = cmd.v.u8 ? 1 : 0;
                    if (g_settings.use_common_temp) g_settings.right_temp_c = g_settings.left_temp_c;
                    save_due_tick = xTaskGetTickCount() + pdMS_TO_TICKS(3000);
                    break;

                case APP_CMD_SET_PRESSURE_KPA:
                    // Update target pressure for next start/update
                    gCurveRT.target_kpa = cmd.v.f32;
                    if (gAppState==APP_STATE_RUN_MODE1 || gAppState==APP_STATE_RUN_MODE2) {
                        ctrl_cmd_t c = {0}; c.id = CTRL_CMD_UPDATE_CFG;
                        c.cfg.mode = 1; c.cfg.running = 1;
                        c.cfg.temp_target = g_settings.use_common_temp ? g_settings.left_temp_c : g_settings.left_temp_c;
                        c.cfg.press_target_max = gCurveRT.target_kpa;
                        c.cfg.t1_rise_s = gCurveRT.t1_rise_s;
                        c.cfg.t2_hold_s = gCurveRT.t2_hold_s;
                        c.cfg.t3_pulse_s = gCurveRT.t3_pulse_s;
                        c.cfg.pulse_on_ms = gCurveRT.pulse_on_ms;
                        c.cfg.pulse_off_ms = gCurveRT.pulse_off_ms;
                        c.cfg.squeeze_mode = g_squeeze_mode;
                        c.cfg.press_enable_L = g_press_enable_L; c.cfg.press_enable_R = g_press_enable_R;
                        (void)xQueueSend(gCtrlCmdQueue, &c, 0);
                    }
                    break;

                case APP_CMD_SET_MODE1_PARAM:
                case APP_CMD_SET_MODE2_PARAM:
                {
                    // Special keys for runtime controls that are not persisted in settings
                    if (cmd.key == 100) { // left temp enable
                        g_temp_enable_L = cmd.v.u8 ? 1 : 0; break; }
                    if (cmd.key == 101) { // right temp enable
                        g_temp_enable_R = cmd.v.u8 ? 1 : 0; break; }

                    // Update single mode timings/pulse and squeeze mode
                    if (cmd.key == 6) { g_squeeze_mode = cmd.v.u8; break; }
                    {
                        switch (cmd.key) {
                            case 1: gCurveRT.t1_rise_s   = cmd.v.u32 / 1000.0f; break;
                            case 2: gCurveRT.t2_hold_s   = cmd.v.u32 / 1000.0f; break;
                            case 3: gCurveRT.t3_pulse_s  = cmd.v.u32 / 1000.0f; break;
                            case 4: gCurveRT.pulse_on_ms = (float)cmd.v.u16; break;
                            case 5: gCurveRT.pulse_off_ms= (float)cmd.v.u16; break;
                            case 0: gCurveRT.target_kpa  = cmd.v.f32; break;
                            default: break;
                        }
                        // If running, push live update
                        if (gAppState==APP_STATE_RUN_MODE1 || gAppState==APP_STATE_RUN_MODE2) {
                            ctrl_cmd_t c = {0}; c.id = CTRL_CMD_UPDATE_CFG;
                            c.cfg.mode = 1; c.cfg.running = 1;
                            c.cfg.temp_target = g_settings.use_common_temp ? g_settings.left_temp_c : g_settings.left_temp_c;
                            c.cfg.press_target_max = gCurveRT.target_kpa;
                            c.cfg.t1_rise_s = gCurveRT.t1_rise_s;
                            c.cfg.t2_hold_s = gCurveRT.t2_hold_s;
                            c.cfg.t3_pulse_s = gCurveRT.t3_pulse_s;
                            c.cfg.pulse_on_ms = gCurveRT.pulse_on_ms;
                            c.cfg.pulse_off_ms = gCurveRT.pulse_off_ms;
                            c.cfg.squeeze_mode = g_squeeze_mode;
                            c.cfg.press_enable_L = g_press_enable_L; c.cfg.press_enable_R = g_press_enable_R;
                            (void)xQueueSend(gCtrlCmdQueue, &c, 0);
                        }
                    }
                    break;
                }

                case APP_CMD_SET_LEFT_PRESSURE_ENABLE:
                {
                    g_press_enable_L = cmd.v.u8 ? 1 : 0;
                    // If controller is running, push live update
                    ctrl_cmd_t c = {0};
                    c.id = CTRL_CMD_UPDATE_CFG;
                    // keep current config snapshot (single mode)
                    c.cfg.mode = current_mode;
                    c.cfg.running = (gAppState==APP_STATE_RUN_MODE1 || gAppState==APP_STATE_RUN_MODE2) ? 1 : 0;
                    c.cfg.temp_target = g_settings.use_common_temp ? g_settings.left_temp_c : g_settings.left_temp_c;
                    c.cfg.press_target_max = gCurveRT.target_kpa;
                    c.cfg.t1_rise_s = gCurveRT.t1_rise_s;
                    c.cfg.t2_hold_s = gCurveRT.t2_hold_s;
                    c.cfg.t3_pulse_s = gCurveRT.t3_pulse_s;
                    c.cfg.pulse_on_ms = gCurveRT.pulse_on_ms;
                    c.cfg.pulse_off_ms = gCurveRT.pulse_off_ms;
                    c.cfg.squeeze_mode = g_squeeze_mode;
                    c.cfg.press_enable_L = g_press_enable_L;
                    c.cfg.press_enable_R = g_press_enable_R;
                    (void)xQueueSend(gCtrlCmdQueue, &c, 0);
                    break;
                }

                case APP_CMD_SET_RIGHT_PRESSURE_ENABLE:
                {
                    g_press_enable_R = cmd.v.u8 ? 1 : 0;
                    ctrl_cmd_t c = {0};
                    c.id = CTRL_CMD_UPDATE_CFG;
                    // keep current config snapshot (single mode)
                    c.cfg.mode = current_mode;
                    c.cfg.running = (gAppState==APP_STATE_RUN_MODE1 || gAppState==APP_STATE_RUN_MODE2) ? 1 : 0;
                    c.cfg.temp_target = g_settings.use_common_temp ? g_settings.left_temp_c : g_settings.left_temp_c;
                    c.cfg.press_target_max = gCurveRT.target_kpa;
                    c.cfg.t1_rise_s = gCurveRT.t1_rise_s;
                    c.cfg.t2_hold_s = gCurveRT.t2_hold_s;
                    c.cfg.t3_pulse_s = gCurveRT.t3_pulse_s;
                    c.cfg.pulse_on_ms = gCurveRT.pulse_on_ms;
                    c.cfg.pulse_off_ms = gCurveRT.pulse_off_ms;
                    c.cfg.squeeze_mode = g_squeeze_mode;
                    c.cfg.press_enable_L = g_press_enable_L;
                    c.cfg.press_enable_R = g_press_enable_R;
                    (void)xQueueSend(gCtrlCmdQueue, &c, 0);
                    break;
                }

                case APP_CMD_READ_PARAM:
                    Settings_Broadcast();
                    break;

                case APP_CMD_SAVE_PARAM:
                    { storage_cmd_t s = STORAGE_CMD_SAVE_PARAM; xQueueSend(gStorageQueue, &s, 0); }
                    break;

                default:
                    break;
            }
        }

        // Save debounce
        if (save_due_tick != 0 && (int32_t)(xTaskGetTickCount() - save_due_tick) >= 0) {
            storage_cmd_t s = STORAGE_CMD_SAVE_PARAM;
            xQueueSend(gStorageQueue, &s, 0);
            save_due_tick = 0;
        }

        // Safety events
        uint8_t fault;
        if (xQueueReceive(gSafetyQueue, &fault, 0) == pdPASS && fault) {
            // Emergency stop
            send_control_cfg(current_mode, 0);
            gAppState = APP_STATE_ALARM;
        }
    }
}
