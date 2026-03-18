/**
 * @file app_task.c
 * @brief 璐熻矗搴旂敤灞傝皟搴﹂€昏緫锛氭帴鏀禔pp鍛戒护銆佺鐞嗚繍琛屾ā寮忋€佹洿鏂版帶鍒跺櫒閰嶇疆骞跺鐞嗗畨鍏?瀛樺偍浜嬩欢銆?
 *
 * 璇ユ枃浠舵槸TPS浠庢満搴旂敤鐨勨€滃ぇ鑴戔€濓紝閫氳繃闃熷垪涓庡悇妯″潡閫氫俊锛?
 *  - gCmdQueue: 鐢遍€氫俊浠诲姟/UART鍙戞潵鐨勪笂浣嶆満鎸囦护
 *  - gCtrlCmdQueue: 涓嬪彂鍒版帶鍒朵换鍔＄殑鍚姩/鍋滄/鍙傛暟鏇存柊鎸囦护
 *  - gStorageQueue: 瑙﹀彂鍙傛暟淇濆瓨
 *  - gSafetyQueue: 瀹夊叏浠诲姟涓婃姤鐨勬晠闅?
 */
#include <string.h>
#include "app_task.h"
#include "LOG.h"
#include "config.h"
#include "Uart_Communicate.h"
#include "AppMain/mode_curves.h"

extern SystemSettings_t g_settings;

/** 褰撳墠杩愯妯″紡缂栧彿锛?~4锛屽搴斾笉鍚屽帇鎺ф洸绾匡級 */
static uint8_t current_mode = 1;
/** 鎺у埗浠诲姟瀹為檯杩愯鐘舵€侊紙1=宸蹭笅鍙慡TART涓旀湭STOP锛?*/
static uint8_t control_active = 0;
/** 鐢ㄦ埛鏄惁璇锋眰杩愯锛圫TART/STOP鎺у埗姝ゆ爣蹇楋級 */
static uint8_t run_request = 0;
/** 宸﹀彸涓や晶姘旇浣胯兘鏍囧織 */
static uint8_t left_enable = 0;
static uint8_t right_enable = 0;
/** 鐩爣娓╁害锛埪癈锛?*/
static float   target_temp_c = 38.0f;
#define TEMP_CONTROL_COMP_SUB_C  (2.0f)
/** 褰撳墠妯″紡瀹炴椂浣跨敤鐨勫帇鍔?鏃堕棿鏇茬嚎锛堝惈涓婃淇濆瓨鐨勮嚜瀹氫箟鍙傛暟锛?*/
static ModeCurve_t gCurveRT;
/** ???????????5????1??????? */
static uint16_t treatment_minutes = 5;
/** ?????? */
static TickType_t session_end_tick = 0;
static uint8_t session_timer_active = 0;

static uint8_t storage_slot_for_mode(uint8_t mode)
{
    /* 妯″紡1~2鍐欏叆妲?锛屽叾浣欏啓鍏ユЫ1锛涚敤浜庢槧灏勫叏灞€璁剧疆缁撴瀯 */
    return (mode <= 1) ? 0 : 1;
}

static void fill_control_cfg(control_config_t *cfg, uint8_t running)
{
    float control_temp_c = target_temp_c - TEMP_CONTROL_COMP_SUB_C;
    if (control_temp_c < 0.0f) control_temp_c = 0.0f;
    /* 灏嗗綋鍓嶅簲鐢ㄥ眰閰嶇疆鎵撳寘鎴愭帶鍒朵换鍔¤兘鐞嗚В鐨勭粨鏋勶紝running鐢ㄤ簬淇濇寔鎺у埗鍣ㄧ姸鎬?*/
    cfg->mode            = current_mode;
    cfg->running         = running;
    cfg->temp_target     = control_temp_c;
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
    /* 鏋勯€犳帶鍒跺懡浠ゅ苟绔嬪嵆鎶曢€掑埌鎺у埗浠诲姟闃熷垪 */
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
    /* 鏍规嵁run_request涓庡乏鍙充娇鑳界姸鎬佸喅瀹氭槸鍚﹁椹卞姩鎺у埗浠诲姟杩愯 */
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
    /* 瑁呰浇鎸囧畾妯″紡鏇茬嚎骞跺簲鐢ㄧ敤鎴疯嚜瀹氫箟鐨勭洰鏍囧帇鍔?*/
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

    /* 涓婄數鍒濆鍖栵細鏍规嵁鎸佷箙鍖栬缃仮澶嶆ā寮?娓╁害/鐩爣鍘嬪姏 */
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
                    /* 鍒囨崲妯″紡锛岀珛鍒诲姞杞藉搴旀洸绾垮苟鍒锋柊鎺у埗鐘舵€?*/
                    g_settings.mode_select = cmd.v.u8;
                    load_mode_curve(cmd.v.u8);
                    update_control_state();
                    break;

                case APP_CMD_SET_TEMP:
                    /* 涓婁綅鏈鸿缃洰鏍囨俯搴︼紝寤舵椂鍐欏叆Flash锛堥槻姝㈤绻佹摝鍐欙級 */
                    target_temp_c = cmd.v.f32;
                    g_settings.left_temp_c  = target_temp_c;
                    g_settings.right_temp_c = target_temp_c;
                    save_due_tick = xTaskGetTickCount() + pdMS_TO_TICKS(3000);
                    update_control_state();
                    break;

                case APP_CMD_SET_PRESSURE_KPA:
                    /* 涓婁綅鏈轰紶鍏ュ崟浣嶄负 mmHg锛屽唴閮ㄧ粺涓€浣跨敤 mmHg锛堝瓧娈靛悕娌跨敤锛?*/
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
                    /* 绔嬪嵆瑙﹀彂瀛樺偍浠诲姟鍐欏叆Flash锛堥珮浼樺厛绾ц姹傦級 */
                    storage_cmd_t s = STORAGE_CMD_SAVE_PARAM;
                    xQueueSend(gStorageQueue, &s, 0);
                    break;
                }

                default:
                    break;
            }
        }

        if (save_due_tick != 0 && (int32_t)(xTaskGetTickCount() - save_due_tick) >= 0) {
            /* 寤惰繜淇濆瓨鍒版湡锛氬啓鍏lash骞舵竻闄よ鏃?*/
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


