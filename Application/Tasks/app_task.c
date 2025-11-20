#include <string.h>
#include "app_task.h"
#include "LOG.h"
#include "config.h"
#include "Uart_Communicate.h"

extern SystemSettings_t g_settings;

static uint8_t current_mode = 1;
static uint8_t control_active = 0;
static uint8_t run_request = 0;
static uint8_t left_enable = 0;
static uint8_t right_enable = 0;
static float target_temp_c = 38.0f;
static float target_pressure_kpa = 25.0f;

// 根据模式号映射到 settings.mode[] 存储槽
static uint8_t storage_slot_for_mode(uint8_t mode)
{
    return (mode <= 1) ? 0 : 1;
}

// 构建控制层配置结构体（温度/压力/左右开关）
static void fill_control_cfg(control_config_t *cfg, uint8_t running)
{
    cfg->mode           = current_mode;
    cfg->running        = running;
    cfg->temp_target    = target_temp_c;
    cfg->press_target_max = target_pressure_kpa;
    cfg->t1_rise_s      = 0.0f;
    cfg->t2_hold_s      = 0.0f;
    cfg->t3_pulse_s     = 0.0f;
    cfg->pulse_on_ms    = 0.0f;
    cfg->pulse_off_ms   = 0.0f;
    cfg->squeeze_mode   = 0;
    cfg->press_enable_L = left_enable;
    cfg->press_enable_R = right_enable;
}

// 发布控制命令到 ControlTask
static void post_control_cmd(ctrl_cmd_id_t id, uint8_t running)
{
    ctrl_cmd_t c = {0};
    c.id = id;
    fill_control_cfg(&c.cfg, running);
    (void)xQueueSend(gCtrlCmdQueue, &c, 0);
}

// 根据 run_request 与左右开关决定是否需要控制输出
static void update_control_state(void)
{
    uint8_t should_run = (run_request && (left_enable || right_enable));
    if (should_run) {
        if (!control_active) {
            control_active = 1;
            gAppState = APP_STATE_RUN_MODE1;
            post_control_cmd(CTRL_CMD_START, 1);      // 首次启动
        } else {
            post_control_cmd(CTRL_CMD_UPDATE_CFG, 1); // 运行中更新
        }
    } else {
        if (control_active) {
            control_active = 0;
            post_control_cmd(CTRL_CMD_STOP, 0);       // 停止输出
        }
    }
}

void AppTask(void *argument)
{
    (void)argument;
    app_cmd_t cmd;
    uint32_t save_due_tick = 0;

    current_mode = (g_settings.mode_select >= 1 && g_settings.mode_select <= 4) ? g_settings.mode_select : 1;
    target_temp_c = g_settings.left_temp_c;
    float stored_pressure = g_settings.mode[storage_slot_for_mode(current_mode)].target_kpa;
    target_pressure_kpa = (stored_pressure > 0.0f) ? stored_pressure : 25.0f;
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
                case APP_CMD_MODE_SELECT: // 选择曲线模式
                    current_mode = (cmd.v.u8 < 1) ? 1 : ((cmd.v.u8 > 4) ? 4 : cmd.v.u8);
                    g_settings.mode_select = current_mode;
                    target_pressure_kpa = g_settings.mode[storage_slot_for_mode(current_mode)].target_kpa;
                    if (target_pressure_kpa <= 0.0f) target_pressure_kpa = 25.0f;
                    update_control_state();
                    break;


                case APP_CMD_SET_TEMP: // 温度设定
                    target_temp_c = cmd.v.f32;
                    g_settings.left_temp_c  = target_temp_c;
                    g_settings.right_temp_c = target_temp_c;
                    save_due_tick = xTaskGetTickCount() + pdMS_TO_TICKS(3000);
                    update_control_state();
                    break;

                case APP_CMD_SET_PRESSURE_KPA: // 压力设定
                    target_pressure_kpa = cmd.v.f32;
                    g_settings.mode[storage_slot_for_mode(current_mode)].target_kpa = target_pressure_kpa;
                    save_due_tick = xTaskGetTickCount() + pdMS_TO_TICKS(3000);
                    update_control_state();
                    break;

                case APP_CMD_LEFT_ENABLE: // 左眼开关
                    left_enable = cmd.v.u8 ? 1 : 0;
                    update_control_state();
                    break;

                case APP_CMD_RIGHT_ENABLE: // 右眼开关
                    right_enable = cmd.v.u8 ? 1 : 0;
                    update_control_state();
                    break;

                case APP_CMD_READ_PARAM: // 请求回传参数
                    Settings_Broadcast();
                    break;

                case APP_CMD_SAVE_PARAM: // 请求保存参数
                {
                    storage_cmd_t s = STORAGE_CMD_SAVE_PARAM;
                    xQueueSend(gStorageQueue, &s, 0);
                    break;
                }

                default: // 未知命令
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
