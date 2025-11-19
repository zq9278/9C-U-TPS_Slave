#include <string.h>
#include "app_task.h"
#include "LOG.h"
#include "config.h"
#include "Uart_Communicate.h"

extern SystemSettings_t g_settings;

static void send_control_cfg(uint8_t mode, uint8_t running)
{
    ctrl_cmd_t c = {0};
    c.id = running ? CTRL_CMD_START : CTRL_CMD_STOP;
    c.cfg.mode = mode;
    c.cfg.running = running;
    // Common temp target
    c.cfg.temp_target = g_settings.use_common_temp ? g_settings.left_temp_c : g_settings.left_temp_c;
    // Select profile by mode
    const PressureProfile_t *pf = &g_settings.mode[(mode<=1)?0:1];
    c.cfg.press_target_max = pf->target_kpa;
    c.cfg.t1_rise_s        = pf->t_rise_ms / 1000.0f;
    c.cfg.t2_hold_s        = pf->t_hold_ms / 1000.0f;
    c.cfg.t3_pulse_s       = pf->t_pulse_total_ms / 1000.0f;
    c.cfg.pulse_on_ms      = pf->t_pulse_on_ms;
    c.cfg.pulse_off_ms     = pf->t_pulse_off_ms;
    c.cfg.squeeze_mode     = pf->squeeze_mode;
    (void)xQueueSend(gCtrlCmdQueue, &c, 0);
}

void AppTask(void *argument)
{
    (void)argument;
    app_cmd_t cmd;
    uint32_t save_due_tick = 0;
    uint8_t current_mode = (g_settings.mode_select<=1)?1:2;
    gAppState = APP_STATE_IDLE;

    for(;;)
    {
        // Handle incoming commands
        if (xQueueReceive(gCmdQueue, &cmd, pdMS_TO_TICKS(10)) == pdPASS)
        {
            switch (cmd.id)
            {
                case APP_CMD_MODE_SELECT:
                    current_mode = (cmd.v.u8<1)?1:((cmd.v.u8>2)?2:cmd.v.u8);
                    g_settings.mode_select = current_mode;
                    gAppState = (gAppState==APP_STATE_RUN_MODE1||gAppState==APP_STATE_RUN_MODE2) ? (current_mode==1?APP_STATE_RUN_MODE1:APP_STATE_RUN_MODE2) : APP_STATE_READY;
                    save_due_tick = xTaskGetTickCount() + pdMS_TO_TICKS(3000);
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
                    g_settings.mode[(current_mode<=1)?0:1].target_kpa = cmd.v.f32;
                    save_due_tick = xTaskGetTickCount() + pdMS_TO_TICKS(3000);
                    break;

                case APP_CMD_SET_MODE1_PARAM:
                case APP_CMD_SET_MODE2_PARAM:
                {
                    uint8_t idx = (cmd.id==APP_CMD_SET_MODE1_PARAM) ? 0 : 1;
                    PressureProfile_t *pf = &g_settings.mode[idx];
                    switch (cmd.key)
                    {
                        case 1: pf->t_rise_ms = cmd.v.u32; break;
                        case 2: pf->t_hold_ms = cmd.v.u32; break;
                        case 3: pf->t_pulse_total_ms = cmd.v.u32; break;
                        case 4: pf->t_pulse_on_ms = cmd.v.u16; break;
                        case 5: pf->t_pulse_off_ms = cmd.v.u16; break;
                        case 6: pf->squeeze_mode = cmd.v.u8; break;
                        default: break;
                    }
                    save_due_tick = xTaskGetTickCount() + pdMS_TO_TICKS(3000);
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

