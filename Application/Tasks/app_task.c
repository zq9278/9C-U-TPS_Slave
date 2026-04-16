/**
 * @file app_task.c
 * @brief Application-level scheduling logic: receives App commands, manages
 *        run modes, updates controller configuration, and handles safety and
 *        storage events.
 *
 * This file is the main application "brain" of the TPS slave and communicates
 * with other modules via queues:
 *  - gCmdQueue: host commands from the comm/UART task
 *  - gCtrlCmdQueue: start/stop/update commands to ControlTask
 *  - gStorageQueue: trigger parameter persistence
 *  - gSafetyQueue: faults reported by SafetyTask
 */
#include <string.h>
#include "FreeRTOS.h"
#include "task.h"
#include "app_task.h"
#include "LOG.h"
#include "config.h"
#include "Uart_Communicate.h"
#include "wave_control.h"

extern SystemSettings_t g_settings;

/*
 * FreeRTOS 官方任务统计日志：
 * - 使用 uxTaskGetSystemState() 获取系统任务快照
 * - 输出任务状态、优先级、栈高水位以及系统堆剩余量
 */
#define RTOS_TASK_STATS_LOG_ENABLE    0
#define RTOS_TASK_STATS_LOG_PERIOD_MS 2000U
#define RTOS_TASK_STATS_MAX_COUNT     16U

#if RTOS_TASK_STATS_LOG_ENABLE
static const char *task_state_name(eTaskState state)
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

static void log_freertos_task_stats(void)
{
    static TickType_t next_log_tick = 0;
    TickType_t now = xTaskGetTickCount();
    TaskStatus_t task_status[RTOS_TASK_STATS_MAX_COUNT];
    UBaseType_t task_count;

    if ((int32_t)(now - next_log_tick) < 0) {
        return;
    }
    next_log_tick = now + pdMS_TO_TICKS(RTOS_TASK_STATS_LOG_PERIOD_MS);

    task_count = uxTaskGetSystemState(task_status, RTOS_TASK_STATS_MAX_COUNT, NULL);
    if (task_count == 0) {
        LOG_W("[RTOS] uxTaskGetSystemState returned 0");
        return;
    }

    LOG_I("[RTOS] ===== Task Snapshot =====");
    LOG_I("[RTOS] tasks=%lu free_heap=%lu min_free_heap=%lu",
          (unsigned long)task_count,
          (unsigned long)xPortGetFreeHeapSize(),
          (unsigned long)xPortGetMinimumEverFreeHeapSize());
    LOG_I("[RTOS] %-16s %-10s %-6s %-10s %-8s",
          "Name", "State", "Prio", "StackHW", "TaskNo");
    LOG_I("[RTOS] --------------------------------------------------------");

    for (UBaseType_t i = 0; i < task_count; ++i) {
        LOG_I("[RTOS] %-16s %-10s %-6lu %-10lu %-8lu",
              task_status[i].pcTaskName,
              task_state_name(task_status[i].eCurrentState),
              (unsigned long)task_status[i].uxCurrentPriority,
              (unsigned long)task_status[i].usStackHighWaterMark,
              (unsigned long)task_status[i].xTaskNumber);
    }
}
#else
static void log_freertos_task_stats(void)
{
}
#endif

/** Current mode index is kept only for host/UI compatibility. */
static uint8_t current_mode = 1;
/** Control task actual running state (1=started, 0=stopped). */
static uint8_t control_active = 0;
/** User run request flag (controlled by START/STOP). */
static uint8_t run_request = 0;
/** Pause request flag (0=running normally, 1=paused waiting for resume). */
static uint8_t pause_request = 0;
/** Track whether ControlTask is currently in pause hold state. */
static uint8_t control_paused = 0;
/** Left/right channel enable flags. */
static uint8_t left_enable = 0;
static uint8_t right_enable = 0;
/** Target temperature (C). */
static float   target_temp_c = 38.0f;
#define TEMP_CONTROL_COMP_SUB_C  (2.0f)
/** Current pressure target used by the fixed 60-second waveform. */
static float current_pressure_target_kpa = 25.0f;
/** Treatment duration in minutes (default 5). */
static uint16_t treatment_minutes = 5;
/** Session end time tracking. */
static TickType_t session_end_tick = 0;
static uint8_t session_timer_active = 0;
/** Remaining countdown ticks captured when treatment is paused. */
static TickType_t session_remaining_ticks = 0;

/* Waveform timing rule:
 * - one treatment cycle is always 60 seconds (WAVE_CONTROL_CYCLE_MS)
 * - total treatment time is an integer multiple of that 60-second cycle
 * The host currently sends U16_TREAT_TIME_MIN; the existing field name is kept
 * for protocol compatibility, but here it is treated as "number of 60s cycles".
 * Wave shape constants (rise/hold/pulse) are defined in wave_control.h.
 */

static uint8_t storage_slot_for_mode(uint8_t mode)
{
    /* Map a mode index to the persistent settings slot. */
    /* Mode 1 writes to slot 0, other modes to slot 1. */
    return (mode <= 1) ? 0 : 1;
}

static void fill_control_cfg(control_config_t *cfg, uint8_t running)
{
    /* Build the control-layer configuration from current app state. */
    float control_temp_c = target_temp_c - TEMP_CONTROL_COMP_SUB_C;
    if (control_temp_c < 0.0f) control_temp_c = 0.0f;
    /* Pack app settings into ControlTask config; running preserves state. */
    cfg->mode            = current_mode;
    cfg->running         = running;
    cfg->temp_target     = control_temp_c;
    /* Fixed waveform configuration:
     * t1/t2/t3 are now purely internal software parameters and no longer come
     * from mode_curves.c.
     */
    cfg->press_target_max= current_pressure_target_kpa;
    cfg->t1_rise_s       = WAVE_CONTROL_RISE_S;
    cfg->t2_hold_s       = WAVE_CONTROL_HOLD_S;
    cfg->t3_pulse_s      = WAVE_CONTROL_PULSE_S;
    cfg->pulse_on_ms     = 0.0f;
    cfg->pulse_off_ms    = 0.0f;
    cfg->squeeze_mode    = 0;
    cfg->press_enable_L  = left_enable;
    cfg->press_enable_R  = right_enable;
    cfg->treatment_minutes = (treatment_minutes == 0) ? 1 : treatment_minutes;
}

static void post_control_cmd(ctrl_cmd_id_t id, uint8_t running)
{
    /* Push a control command with the current config to ControlTask. */
    /* Build control command and post to ControlTask. */
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
    /* One waveform cycle is fixed to 60 seconds. The internal rise / hold /
     * pulse durations only define how that 60-second window is split.
     */
    return WAVE_CONTROL_CYCLE_MS;
}

static void arm_session_timer(void)
{
    /* Start/refresh the session end timer.
     * treatment_minutes is preserved as the protocol field name, but it now
     * means "number of 60-second waveform cycles".
     */
    uint32_t minutes = (treatment_minutes == 0) ? 1U : (uint32_t)treatment_minutes;
    TickType_t now = xTaskGetTickCount();
    uint32_t total_ms = curve_cycle_duration_ms() * minutes;
    session_end_tick = now + pdMS_TO_TICKS(total_ms);
    session_timer_active = 1;
}

static void freeze_session_timer(void)
{
    /* Capture the remaining countdown so pause/resume can continue from the
     * same remaining treatment time instead of restarting the whole session.
     */
    TickType_t now = xTaskGetTickCount();
    if (session_timer_active && (session_end_tick > now)) {
        session_remaining_ticks = session_end_tick - now;
    } else {
        session_remaining_ticks = 0;
    }
    session_timer_active = 0;
}

static void resume_session_timer(void)
{
    /* Restore the frozen countdown when treatment resumes. */
    if (session_remaining_ticks > 0) {
        session_end_tick = xTaskGetTickCount() + session_remaining_ticks;
        session_timer_active = 1;
    }
}

static void update_control_state(void)
{
    /* Decide start/stop/update actions for ControlTask based on app state. */
    /* Decide whether ControlTask should run based on run_request and enables.
     * Pause is handled as a separate hold state: the session still exists, but
     * heating / squeezing outputs are forced off until a resume command arrives.
     */
    uint8_t should_run = (run_request && (left_enable || right_enable));
    if (should_run) {
        if (pause_request) {
            gTreatmentRunning = 0;
            gAppState = APP_STATE_READY;
            if (control_active && !control_paused) {
                control_paused = 1;
                LOG_W("update_control_state: PAUSE run_req=%u L_en=%u R_en=%u",
                      run_request, left_enable, right_enable);
                post_control_cmd(CTRL_CMD_PAUSE, 1);
            }
            return;
        }

        gTreatmentRunning = 1;
        gAppState = APP_STATE_RUN_MODE1;
        if (!control_active) {
            control_active = 1;
            control_paused = 0;
            LOG_I("update_control_state: START should_run=1 run_req=%u L_en=%u R_en=%u", run_request, left_enable, right_enable);
            post_control_cmd(CTRL_CMD_START, 1);
            if (!session_timer_active) {
                if (session_remaining_ticks > 0) {
                    resume_session_timer();
                } else {
                    arm_session_timer();
                }
            }
        } else if (control_paused) {
            control_paused = 0;
            LOG_I("update_control_state: RESUME run_req=%u L_en=%u R_en=%u", run_request, left_enable, right_enable);
            post_control_cmd(CTRL_CMD_RESUME, 1);
            if (!session_timer_active) {
                resume_session_timer();
            }
        } else {
            LOG_I("update_control_state: UPDATE_CFG should_run=1 run_req=%u L_en=%u R_en=%u", run_request, left_enable, right_enable);
            post_control_cmd(CTRL_CMD_UPDATE_CFG, 1);
            if (!session_timer_active) {
                arm_session_timer();
            }
        }
    } else {
        gTreatmentRunning = 0;
        if (control_active || control_paused) {
            control_active = 0;
            control_paused = 0;
            LOG_W("update_control_state: STOP should_run=0 run_req=%u L_en=%u R_en=%u", run_request, left_enable, right_enable);
            post_control_cmd(CTRL_CMD_STOP, 0);
        }
        gAppState = APP_STATE_READY;
        session_timer_active = 0;
        session_remaining_ticks = 0;
    }
}

static void load_pressure_target_for_mode(uint8_t mode)
{
    /* Mode select is still accepted from the host, but it only selects which
     * stored target pressure slot to use. Waveform timing no longer comes from
     * mode_curves.c.
     */
    if (mode < 1) mode = 1;
    if (mode > 4) mode = 4;
    current_mode = mode;
    float stored = g_settings.mode[storage_slot_for_mode(current_mode)].target_kpa;
    current_pressure_target_kpa = (stored > 0.0f) ? stored : 25.0f;
}

void AppTask(void *argument)
{
    /* App task main loop: consume commands, update state, and dispatch control/storage actions. */
    (void)argument;
    app_cmd_t cmd;
    uint32_t save_due_tick = 0;

    /* Power-on init: restore mode/temp/pressure from persisted settings. */
    load_pressure_target_for_mode((g_settings.mode_select >= 1 && g_settings.mode_select <= 4) ? g_settings.mode_select : 1);
    target_temp_c = g_settings.left_temp_c;
    left_enable = 0;
    right_enable = 0;
    run_request = 0;
    pause_request = 0;
    control_paused = 0;
    control_active = 0;
    gAppState = APP_STATE_IDLE;

    for(;;)
    {
        if (xQueueReceive(gCmdQueue, &cmd, pdMS_TO_TICKS(10)) == pdPASS)
        {
            switch (cmd.id)
            {
                case APP_CMD_MODE_SELECT:
                    /* Switch mode and load the corresponding stored pressure
                     * target. Waveform timing itself stays fixed in software.
                     */
                    g_settings.mode_select = cmd.v.u8;
                    load_pressure_target_for_mode(cmd.v.u8);
                    update_control_state();
                    break;

                case APP_CMD_SET_TEMP:
                    /* Host sets target temperature; defer flash save. */
                    target_temp_c = cmd.v.f32;
                    g_settings.left_temp_c  = target_temp_c;
                    g_settings.right_temp_c = target_temp_c;
                    save_due_tick = xTaskGetTickCount() + pdMS_TO_TICKS(3000);
                    update_control_state();
                    break;

                case APP_CMD_SET_PRESSURE_KPA:
                    /* Host pressure setpoint stored internally. */
                    current_pressure_target_kpa = cmd.v.f32;
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
                    /* The host value is treated as the number of 60-second
                     * cycles. 0 is coerced to one cycle.
                     */
                    treatment_minutes = (cmd.v.u16 == 0) ? 1 : cmd.v.u16;
                    if (control_active) {
                        arm_session_timer();
                        update_control_state();
                    }
                    break;

                case APP_CMD_START:
                    /* If both sides disabled, enable both before starting. */
                    if (left_enable == 0 && right_enable == 0) {
                        left_enable = 1;
                        right_enable = 1;
                    }
                    pause_request = 0;
                    run_request = 1;
                    //LOG_I("AppTask START: run_request=%u L_en=%u R_en=%u", run_request, left_enable, right_enable);
                    update_control_state();
                    break;

                case APP_CMD_STOP:
                    //LOG_I("AppTask STOP: run_request=%u L_en=%u R_en=%u", run_request, left_enable, right_enable);
                    pause_request = 0;
                    run_request = 0;
                    //LOG_I("AppTask STOP: run_request=%u L_en=%u R_en=%u", run_request, left_enable, right_enable);
                    update_control_state();
                    break;

                case APP_CMD_PAUSE_RESUME:
                    /* Host sends 0=pause, 1=resume.
                     * Pause freezes the countdown and stops control outputs;
                     * resume continues the same session from the remaining time.
                     */
                    if (cmd.v.u8 == 0U) {
                        if (run_request && !pause_request) {
                            pause_request = 1;
                            freeze_session_timer();
                            update_control_state();
                        }
                    } else {
                        if (run_request && pause_request) {
                            pause_request = 0;
                            update_control_state();
                        }
                    }
                    break;

                case APP_CMD_READ_PARAM:
                    Settings_Broadcast();
                    break;

                case APP_CMD_SAVE_PARAM:
                {
                    /* Immediately trigger storage task to write to flash. */
                    storage_cmd_t s = STORAGE_CMD_SAVE_PARAM;
                    xQueueSend(gStorageQueue, &s, 0);
                    break;
                }

                default:
                    break;
            }
        }

        if (save_due_tick != 0 && (int32_t)(xTaskGetTickCount() - save_due_tick) >= 0) {
            /* Deferred save elapsed: write to flash and clear timer. */
            storage_cmd_t s = STORAGE_CMD_SAVE_PARAM;
            xQueueSend(gStorageQueue, &s, 0);
            save_due_tick = 0;
        }

        {
            uint8_t fault;
            if (xQueueReceive(gSafetyQueue, &fault, 0) == pdPASS && fault) {
                /* Safety fault: stop and enter alarm state. */
                run_request = 0;
                pause_request = 0;
                left_enable = 0;
                right_enable = 0;
                LOG_W("AppTask SAFETY fault=%u -> stop, L_en=%u R_en=%u", fault, left_enable, right_enable);
                update_control_state();
                gAppState = APP_STATE_ALARM;
            }
        }

        if (control_active && !control_paused && session_timer_active) {
            TickType_t now = xTaskGetTickCount();
            if ((int32_t)(now - session_end_tick) >= 0) {
                run_request = 0;
                pause_request = 0;
                session_timer_active = 0;
                session_remaining_ticks = 0;
                update_control_state();

                /* 倒计时结束后，下位机主动通知上位机治疗已结束。 */
                tx_frame_t tx = {0};
                tx.type = TX_DATA_UINT8;
                tx.frame_id = U8_STOP_TREATMENT;
                tx.v.u8 = 1;
                xQueueSend(gTxQueue, &tx, 0);
            }
        }

        log_freertos_task_stats();
    }
}
