#include <string.h>
#include "app_task.h"
#include "LOG.h"
#include "config.h"
#include "Uart_Communicate.h"
#include "AppMain/mode_curves.h"

extern SystemSettings_t g_settings;

static uint8_t current_mode = 1;
static uint8_t control_active = 0;
static uint8_t run_request = 0;
static uint8_t left_enable = 0;
static uint8_t right_enable = 0;
static float   target_temp_c = 38.0f;
static ModeCurve_t gCurveRT;

static uint8_t storage_slot_for_mode(uint8_t mode)
{
    return (mode <= 1) ? 0 : 1;
}

static void fill_control_cfg(control_config_t *cfg, uint8_t running)
{
    cfg->mode            = current_mode;
    cfg->running         = running;
    cfg->temp_target     = target_temp_c;
    cfg->press_target_max= gCurveRT.target_kpa;
    cfg->t1_rise_s       = gCurveRT.t1_rise_s;
    cfg->t2_hold_s       = gCurveRT.t2_hold_s;
    cfg->t3_pulse_s      = gCurveRT.t3_pulse_s;
    cfg->pulse_on_ms     = gCurveRT.pulse_on_ms;
    cfg->pulse_off_ms    = gCurveRT.pulse_off_ms;
    cfg->squeeze_mode    = 0;
    cfg->press_enable_L  = left_enable;
    cfg->press_enable_R  = right_enable;
}

static void post_control_cmd(ctrl_cmd_id_t id, uint8_t running)
{
    ctrl_cmd_t c = {0};
    c.id = id;
    fill_control_cfg(&c.cfg, running);
    (void)xQueueSend(gCtrlCmdQueue, &c, 0);
}

static void update_control_state(void)
{
    uint8_t should_run = (run_request && (left_enable || right_enable));
    if (should_run) {
        gAppState = APP_STATE_RUN_MODE1;
        if (!control_active) {
            control_active = 1;
            post_control_cmd(CTRL_CMD_START, 1);
        } else {
            post_control_cmd(CTRL_CMD_UPDATE_CFG, 1);
        }
    } else {
        if (control_active) {
            control_active = 0;
            post_control_cmd(CTRL_CMD_STOP, 0);
        }
        gAppState = APP_STATE_READY;
    }
}

static void load_mode_curve(uint8_t mode)
{
    if (mode < 1) mode = 1;
    if (mode > 4) mode = 4;
    current_mode = mode;
    gCurveRT = gModeCurves[current_mode - 1];
    float stored = g_settings.mode[storage_slot_for_mode(current_mode)].target_kpa;
    if (stored > 0.0f) gCurveRT.target_kpa = stored;
}

void AppTask(void *argument)
{
    (void)argument;
    app_cmd_t cmd;
    uint32_t save_due_tick = 0;

    load_mode_curve((g_settings.mode_select >= 1 && g_settings.mode_select <= 4) ? g_settings.mode_select : 1);
    target_temp_c = g_settings.left_temp_c;
    left_enable = 0;
    right_enable = 0;
    run_request = 0;
    control_active = 0;
    gAppState = APP_STATE_IDLE;

    for(;;)
    {
        if (xQueueReceive(gCmdQueue, &cmd, pdMS_TO_TICKS(10)) == pdPASS)
        {
            switch (cmd.id)
            {
                case APP_CMD_MODE_SELECT:
                    g_settings.mode_select = cmd.v.u8;
                    load_mode_curve(cmd.v.u8);
                    update_control_state();
                    break;

                case APP_CMD_SET_TEMP:
                    target_temp_c = cmd.v.f32;
                    g_settings.left_temp_c  = target_temp_c;
                    g_settings.right_temp_c = target_temp_c;
                    save_due_tick = xTaskGetTickCount() + pdMS_TO_TICKS(3000);
                    update_control_state();
                    break;

                case APP_CMD_SET_PRESSURE_KPA:
                    gCurveRT.target_kpa = cmd.v.f32;
                    g_settings.mode[storage_slot_for_mode(current_mode)].target_kpa = cmd.v.f32;
                    save_due_tick = xTaskGetTickCount() + pdMS_TO_TICKS(3000);
                    update_control_state();
                    break;

                case APP_CMD_LEFT_ENABLE:
                    left_enable = cmd.v.u8 ? 1 : 0;
                    update_control_state();
                    break;

                case APP_CMD_RIGHT_ENABLE:
                    right_enable = cmd.v.u8 ? 1 : 0;
                    update_control_state();
                    break;

                case APP_CMD_START:
                    run_request = 1;
                    update_control_state();
                    break;

                case APP_CMD_STOP:
                    run_request = 0;
                    update_control_state();
                    break;

                case APP_CMD_READ_PARAM:
                    Settings_Broadcast();
                    break;

                case APP_CMD_SAVE_PARAM:
                {
                    storage_cmd_t s = STORAGE_CMD_SAVE_PARAM;
                    xQueueSend(gStorageQueue, &s, 0);
                    break;
                }

                default:
                    break;
            }
        }

        if (save_due_tick != 0 && (int32_t)(xTaskGetTickCount() - save_due_tick) >= 0) {
            storage_cmd_t s = STORAGE_CMD_SAVE_PARAM;
            xQueueSend(gStorageQueue, &s, 0);
            save_due_tick = 0;
        }

        uint8_t fault;
        if (xQueueReceive(gSafetyQueue, &fault, 0) == pdPASS && fault) {
            run_request = 0;
            left_enable = 0;
            right_enable = 0;
            update_control_state();
            gAppState = APP_STATE_ALARM;
        }
    }
}

