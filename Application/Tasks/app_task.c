/**
 * @file app_task.c
 * @brief 负责应用层调度逻辑：接收App命令、管理运行模式、更新控制器配置并处理安全/存储事件。
 *
 * 该文件是TPS从机应用的“大脑”，通过队列与各模块通信：
 *  - gCmdQueue: 由通信任务/UART发来的上位机指令
 *  - gCtrlCmdQueue: 下发到控制任务的启动/停止/参数更新指令
 *  - gStorageQueue: 触发参数保存
 *  - gSafetyQueue: 安全任务上报的故障
 */
#include <string.h>
#include "app_task.h"
#include "LOG.h"
#include "config.h"
#include "Uart_Communicate.h"
#include "AppMain/mode_curves.h"

extern SystemSettings_t g_settings;

/** 当前运行模式编号（1~4，对应不同压控曲线） */
static uint8_t current_mode = 1;
/** 控制任务实际运行状态（1=已下发START且未STOP） */
static uint8_t control_active = 0;
/** 用户是否请求运行（START/STOP控制此标志） */
static uint8_t run_request = 0;
/** 左右两侧气袋使能标志 */
static uint8_t left_enable = 0;
static uint8_t right_enable = 0;
/** 目标温度（°C） */
static float   target_temp_c = 38.0f;
/** 当前模式实时使用的压力/时间曲线（含上次保存的自定义参数） */
static ModeCurve_t gCurveRT;
/** ???????????5????1??????? */
static uint16_t treatment_minutes = 5;
/** ?????? */
static TickType_t session_end_tick = 0;
static uint8_t session_timer_active = 0;

static uint8_t storage_slot_for_mode(uint8_t mode)
{
    /* 模式1~2写入槽0，其余写入槽1；用于映射全局设置结构 */
    return (mode <= 1) ? 0 : 1;
}

static void fill_control_cfg(control_config_t *cfg, uint8_t running)
{
    /* 将当前应用层配置打包成控制任务能理解的结构，running用于保持控制器状态 */
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
    cfg->treatment_minutes = (treatment_minutes == 0) ? 1 : treatment_minutes;
}

static void post_control_cmd(ctrl_cmd_id_t id, uint8_t running)
{
    /* 构造控制命令并立即投递到控制任务队列 */
    ctrl_cmd_t c = {0};
    c.id = id;
    fill_control_cfg(&c.cfg, running);
    if (xQueueSend(gCtrlCmdQueue, &c, 0) != pdPASS) {
        //LOG_W("post_control_cmd id=%d failed", id);
    } else {
        //LOG_I("post_control_cmd id=%d running=%u L_en=%u R_en=%u", id, running, c.cfg.press_enable_L, c.cfg.press_enable_R);
    }
}

static uint32_t curve_cycle_duration_ms(void)
{
    float total_s = gCurveRT.t1_rise_s + gCurveRT.t2_hold_s + gCurveRT.t3_pulse_s;
    if (total_s <= 0.0f) total_s = 60.0f;
    return (uint32_t)(total_s * 1000.0f);
}

static void arm_session_timer(void)
{
    uint32_t minutes = (treatment_minutes == 0) ? 1U : (uint32_t)treatment_minutes;
    TickType_t now = xTaskGetTickCount();
    uint32_t total_ms = curve_cycle_duration_ms() * minutes;
    session_end_tick = now + pdMS_TO_TICKS(total_ms);
    session_timer_active = 1;
}

static void update_control_state(void)
{
    /* 根据run_request与左右使能状态决定是否要驱动控制任务运行 */
    uint8_t should_run = (run_request && (left_enable || right_enable));
    if (should_run) {
        gAppState = APP_STATE_RUN_MODE1;
        if (!control_active) {
            control_active = 1;
            //LOG_I("update_control_state: START should_run=1 run_req=%u L_en=%u R_en=%u", run_request, left_enable, right_enable);
            post_control_cmd(CTRL_CMD_START, 1);
            arm_session_timer();
        } else {
            //LOG_I("update_control_state: UPDATE_CFG should_run=1 run_req=%u L_en=%u R_en=%u", run_request, left_enable, right_enable);
            post_control_cmd(CTRL_CMD_UPDATE_CFG, 1);
            if (!session_timer_active) {
                arm_session_timer();
            }
        }
    } else {
        if (control_active) {
            control_active = 0;
            //LOG_W("update_control_state: STOP should_run=0 run_req=%u L_en=%u R_en=%u", run_request, left_enable, right_enable);
            post_control_cmd(CTRL_CMD_STOP, 0);
        }
        gAppState = APP_STATE_READY;
        session_timer_active = 0;
    }
}

static void load_mode_curve(uint8_t mode)
{
    /* 装载指定模式曲线并应用用户自定义的目标压力 */
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

    /* 上电初始化：根据持久化设置恢复模式/温度/目标压力 */
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
                    /* 切换模式，立刻加载对应曲线并刷新控制状态 */
                    g_settings.mode_select = cmd.v.u8;
                    load_mode_curve(cmd.v.u8);
                    update_control_state();
                    break;

                case APP_CMD_SET_TEMP:
                    /* 上位机设置目标温度，延时写入Flash（防止频繁擦写） */
                    target_temp_c = cmd.v.f32;
                    g_settings.left_temp_c  = target_temp_c;
                    g_settings.right_temp_c = target_temp_c;
                    save_due_tick = xTaskGetTickCount() + pdMS_TO_TICKS(3000);
                    update_control_state();
                    break;

                case APP_CMD_SET_PRESSURE_KPA:
                    /* 上位机传入单位为 mmHg，内部统一使用 mmHg（字段名沿用） */
                    gCurveRT.target_kpa = cmd.v.f32;
                    g_settings.mode[storage_slot_for_mode(current_mode)].target_kpa = cmd.v.f32;
                    save_due_tick = xTaskGetTickCount() + pdMS_TO_TICKS(3000);
                    update_control_state();
                    break;

                case APP_CMD_LEFT_ENABLE:
                    //LOG_I("AppTask LEFT_EN: %u", cmd.v.u8 ? 1 : 0);
                    left_enable = cmd.v.u8 ? 1 : 0; 
                    update_control_state();
                    break;

                case APP_CMD_RIGHT_ENABLE:
                    //LOG_I("AppTask RIGHT_EN: %u", cmd.v.u8 ? 1 : 0);
                    right_enable = cmd.v.u8 ? 1 : 0;
                    update_control_state();
                    break;

                case APP_CMD_SET_TREATMENT_TIME:
                    treatment_minutes = (cmd.v.u16 == 0) ? 1 : cmd.v.u16;
                    if (control_active) {
                        arm_session_timer();
                        update_control_state();
                    }
                    break;

                case APP_CMD_START:
                    /* ?????????????????????????? */
                    if (left_enable == 0 && right_enable == 0) {
                        left_enable = 1;
                        right_enable = 1;
                    }
                    run_request = 1;
                    //LOG_I("AppTask START: run_request=%u L_en=%u R_en=%u", run_request, left_enable, right_enable);
                    update_control_state();
                    break;

                case APP_CMD_STOP:
                    //LOG_I("AppTask STOP: run_request=%u L_en=%u R_en=%u", run_request, left_enable, right_enable);
                    run_request = 0;
                    //LOG_I("AppTask STOP: run_request=%u L_en=%u R_en=%u", run_request, left_enable, right_enable);
                    update_control_state();
                    break;
                case APP_CMD_READ_PARAM:
                    Settings_Broadcast();
                    break;

                case APP_CMD_SAVE_PARAM:
                {
                    /* 立即触发存储任务写入Flash（高优先级请求） */
                    storage_cmd_t s = STORAGE_CMD_SAVE_PARAM;
                    xQueueSend(gStorageQueue, &s, 0);
                    break;
                }

                default:
                    break;
            }
        }

        if (save_due_tick != 0 && (int32_t)(xTaskGetTickCount() - save_due_tick) >= 0) {
            /* 延迟保存到期：写入Flash并清除计时 */
            storage_cmd_t s = STORAGE_CMD_SAVE_PARAM;
            xQueueSend(gStorageQueue, &s, 0);
            save_due_tick = 0;
        }

        uint8_t fault;
        if (xQueueReceive(gSafetyQueue, &fault, 0) == pdPASS && fault) {
            /* ????????????????????????????????????????? */
            run_request = 0;
            left_enable = 0;
            right_enable = 0;
            //LOG_W("AppTask SAFETY fault=%u -> stop, L_en=%u R_en=%u", fault, left_enable, right_enable);
            update_control_state();
            gAppState = APP_STATE_ALARM;
        }

        if (control_active && session_timer_active) {
            TickType_t now = xTaskGetTickCount();
            if ((int32_t)(now - session_end_tick) >= 0) {
                run_request = 0;
                session_timer_active = 0;
                update_control_state();
                tx_frame_t tx = {0};
                tx.type = TX_DATA_UINT8;
                tx.frame_id = U8_STOP_TREATMENT;
                tx.v.u8 = 1;
                xQueueSend(gTxQueue, &tx, 0);
            }
        }
    }
}
