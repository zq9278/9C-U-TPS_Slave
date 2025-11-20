#include <string.h>
#include "app_task.h"
#include "LOG.h"
#include "config.h"
#include "Uart_Communicate.h"
#include "AppMain/mode_curves.h"

extern SystemSettings_t g_settings;
// Runtime flags (compatibility, always 1 in current protocol)
uint8_t g_press_enable_L = 1;
uint8_t g_press_enable_R = 1;
uint8_t g_temp_enable_L  = 1;
uint8_t g_temp_enable_R  = 1;

static uint8_t current_mode = 1;
static float   g_mode_pressure[4];

static inline uint8_t storage_slot_for_mode(uint8_t mode)
{
    return (mode <= 1) ? 0 : 1;
}

static void fill_control_cfg(control_config_t *cfg, uint8_t running)
{
    if (current_mode < 1) current_mode = 1;
    if (current_mode > 4) current_mode = 4;
    const ModeCurve_t *cv = &gModeCurves[current_mode - 1];
    cfg->mode           = current_mode;
    cfg->running        = running;
    cfg->temp_target    = g_settings.left_temp_c;
    cfg->press_target_max = g_mode_pressure[current_mode - 1];
    cfg->t1_rise_s      = cv->t1_rise_s;
    cfg->t2_hold_s      = cv->t2_hold_s;
    cfg->t3_pulse_s     = cv->t3_pulse_s;
    cfg->pulse_on_ms    = cv->pulse_on_ms;
    cfg->pulse_off_ms   = cv->pulse_off_ms;
    cfg->squeeze_mode   = 0;
    cfg->press_enable_L = g_press_enable_L;
    cfg->press_enable_R = g_press_enable_R;
}

static void dispatch_ctrl_cmd(ctrl_cmd_id_t id, uint8_t running)
{
    ctrl_cmd_t c = {0};
    c.id = id;
    fill_control_cfg(&c.cfg, running);
    (void)xQueueSend(gCtrlCmdQueue, &c, 0);
}

static inline void update_if_running(void)
{
    if (gAppState == APP_STATE_RUN_MODE1 || gAppState == APP_STATE_RUN_MODE2) {
        dispatch_ctrl_cmd(CTRL_CMD_UPDATE_CFG, 1);
    }
}

void AppTask(void *argument)
{
    (void)argument;
    app_cmd_t cmd;
    uint32_t save_due_tick = 0;

    // 初始化模式、压力目标
    g_mode_pressure[0] = (g_settings.mode[0].target_kpa > 0.0f) ? g_settings.mode[0].target_kpa : gModeCurves[0].target_kpa;
    g_mode_pressure[1] = (g_settings.mode[1].target_kpa > 0.0f) ? g_settings.mode[1].target_kpa : gModeCurves[1].target_kpa;
    g_mode_pressure[2] = g_mode_pressure[1];
    g_mode_pressure[3] = g_mode_pressure[1];

    current_mode = (g_settings.mode_select >= 1 && g_settings.mode_select <= 4) ? g_settings.mode_select : 1;
    gAppState = APP_STATE_IDLE;

    for(;;)
    {
        if (xQueueReceive(gCmdQueue, &cmd, pdMS_TO_TICKS(10)) == pdPASS)
        {
            switch (cmd.id)
            {
                case APP_CMD_MODE_SELECT:
                    current_mode = (cmd.v.u8 < 1) ? 1 : ((cmd.v.u8 > 4) ? 4 : cmd.v.u8);
                    g_settings.mode_select = current_mode;
                    update_if_running();
                    break;

                case APP_CMD_START:
                    gAppState = APP_STATE_RUN_MODE1;
                    dispatch_ctrl_cmd(CTRL_CMD_START, 1);
                    break;

                case APP_CMD_STOP:
                    dispatch_ctrl_cmd(CTRL_CMD_STOP, 0);
                    gAppState = APP_STATE_READY;
                    break;

                case APP_CMD_SET_TEMP:
                    g_settings.left_temp_c  = cmd.v.f32;
                    g_settings.right_temp_c = cmd.v.f32;
                    save_due_tick = xTaskGetTickCount() + pdMS_TO_TICKS(3000);
                    update_if_running();
                    break;

                case APP_CMD_SET_PRESSURE_KPA:
                {
                    g_mode_pressure[current_mode - 1] = cmd.v.f32;
                    uint8_t slot = storage_slot_for_mode(current_mode);
                    g_settings.mode[slot].target_kpa = cmd.v.f32;
                    save_due_tick = xTaskGetTickCount() + pdMS_TO_TICKS(3000);
                    update_if_running();
                    break;
                }

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
            dispatch_ctrl_cmd(CTRL_CMD_STOP, 0);
            gAppState = APP_STATE_ALARM;
        }
    }
}
