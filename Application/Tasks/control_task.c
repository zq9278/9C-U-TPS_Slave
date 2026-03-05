#include "Uart_Communicate.h"
#include <string.h>
#include <stdbool.h>
#include "control_task.h"
#include "pid.h"
#include "heat.h"
#include "apply.h"
#include "tim.h"
#include "LOG.h"
#include "main.h"

#include "FreeRTOS.h"
#include "task.h"

/*
 * control_task.c
 *
 * 职责：
 * 1) 接收 AppTask 下发的控制命令（启动/停止/更新配置）
 * 2) 执行压力阶段状态机（预泄压->上升->保压->脉冲->周期间泄压）
 * 3) 执行气泵 PID、温控 PID、阀门与加热输出控制
 * 4) 周期上报压力/温度/阶段及加热盾状态
 */

/* 调试宏：忽略熔断保护（在位检测仍然生效） */
#define IGNORE_FUSE_PROTECTION_DEBUG

static PID_TypeDef pid_press;
static PID_TypeDef pid_heat_L, pid_heat_R;

static control_config_t gCfg;
static TickType_t t_start_tick = 0;
static uint8_t emergency_stop = 0;

typedef enum {
    CTRL_STATE_IDLE = 0,          // 空闲
    CTRL_STATE_PRE_RISE_VENT,     // 启动前预泄压
    CTRL_STATE_RISE,              // 压力上升
    CTRL_STATE_HOLD,              // 保压
    CTRL_STATE_PULSE,             // 脉冲
    CTRL_STATE_INTER_CYCLE_VENT,  // 周期之间泄压
} ctrl_state_t;

static ctrl_state_t ctrl_state = CTRL_STATE_IDLE;
static TickType_t state_enter_tick = 0;
static TickType_t cycle_start_tick = 0;
static TickType_t pre_vent_end_tick = 0;
static TickType_t inter_vent_end_tick = 0;
static uint8_t pre_vent_close_done = 0;
static uint8_t pre_r_vent_done = 0; // 本次运行中是否已经执行过“启动前预泄压”

#define TX_PRESS_MEDIAN_WINDOW 3

/*
 * 发送侧压力中位数滤波：仅用于上位机显示，不影响控制闭环。
 */
static float median_filter_tx(const float *buf, uint8_t count)
{
    float tmp[TX_PRESS_MEDIAN_WINDOW];
    for (uint8_t i = 0; i < count; ++i) tmp[i] = buf[i];
    for (uint8_t i = 0; i + 1 < count; ++i) {
        for (uint8_t j = i + 1; j < count; ++j) {
            if (tmp[j] < tmp[i]) {
                float t = tmp[i];
                tmp[i] = tmp[j];
                tmp[j] = t;
            }
        }
    }

    if (count == 0) return 0.0f;
    if (count & 1U) return tmp[count / 2U];
    return 0.5f * (tmp[count / 2U - 1U] + tmp[count / 2U]);
}

static inline uint32_t ms_since_tick(TickType_t tick)
{
    TickType_t now = xTaskGetTickCount();
    return (uint32_t)((now - tick) * portTICK_PERIOD_MS);
}

/* 切换状态并刷新状态进入时刻 */
static void enter_state(ctrl_state_t s)
{
    ctrl_state = s;
    state_enter_tick = xTaskGetTickCount();
}

/* 将所有执行器切到安全空闲态 */
static void apply_idle_outputs(void)
{
    HeatPWMSet(Left, 0);
    HeatPower(Left, 0);
    HeatPWMSet(Right, 0);
    HeatPower(Right, 0);
    VALVE_LEFT(0);
    VALVE_RIGHT(0);
    TIM15->CCR1 = 0;
}

/*
 * 启动一个压力周期。
 * allow_pre_vent=1 且本次还未预泄压时，先执行 1s 预泄压再进入 RISE。
 */
static void start_pressure_cycle(uint8_t allow_pre_vent)
{
    cycle_start_tick = xTaskGetTickCount();
    state_enter_tick = cycle_start_tick;
    t_start_tick = cycle_start_tick;

    if (allow_pre_vent && !pre_r_vent_done) {
        ctrl_state = CTRL_STATE_PRE_RISE_VENT;
        pre_vent_end_tick = state_enter_tick + pdMS_TO_TICKS(1000);
        pre_vent_close_done = 0;
        VALVE_LEFT(0);
        VALVE_RIGHT(0); // 预泄压
        TIM15->CCR1 = 0;
    } else {
        ctrl_state = CTRL_STATE_RISE;
    }
}

/* 启动周期间泄压（固定 2s） */
static void start_inter_cycle_vent(void)
{
    ctrl_state = CTRL_STATE_INTER_CYCLE_VENT;
    state_enter_tick = xTaskGetTickCount();
    inter_vent_end_tick = state_enter_tick + pdMS_TO_TICKS(2000);
    VALVE_LEFT(0);
    VALVE_RIGHT(0);
    TIM15->CCR1 = 0;
}

/*
 * 上报加热盾状态（在位/熔断），并按安全策略执行保护。
 */
static void SendHeaterStatusFrames(void)
{
    tx_frame_t tx = {0};

    /* 约定：低电平表示不存在/熔断，发送 0；高电平发送 1 */
    uint8_t right_present = (HAL_GPIO_ReadPin(MCU_Heat1_Sense_GPIO_Port, MCU_Heat1_Sense_Pin) == GPIO_PIN_RESET) ? 0 : 1;
    uint8_t left_present  = (HAL_GPIO_ReadPin(MCU_Heat2_Sense_GPIO_Port, MCU_Heat2_Sense_Pin) == GPIO_PIN_RESET) ? 0 : 1;
    uint8_t right_fuse    = (HAL_GPIO_ReadPin(Heat1_Fuse_Detection_GPIO_Port, Heat1_Fuse_Detection_Pin) == GPIO_PIN_RESET) ? 0 : 1;
    uint8_t left_fuse     = (HAL_GPIO_ReadPin(Heat2_Fuse_Detection_GPIO_Port, Heat2_Fuse_Detection_Pin) == GPIO_PIN_RESET) ? 0 : 1;

    tx.type = TX_DATA_UINT8;
    tx.frame_id = U8_LEFT_HEATER_PRESENT;
    tx.v.u8 = left_present;
    xQueueSend(gTxQueue, &tx, 0);

    tx.frame_id = U8_RIGHT_HEATER_PRESENT;
    tx.v.u8 = right_present;
    xQueueSend(gTxQueue, &tx, 0);

    tx.frame_id = U8_LEFT_HEATER_FUSE;
    tx.v.u8 = left_fuse;
    xQueueSend(gTxQueue, &tx, 0);

    tx.frame_id = U8_RIGHT_HEATER_FUSE;
    tx.v.u8 = right_fuse;
    xQueueSend(gTxQueue, &tx, 0);

    /* 盾不在位：强制关闭对应侧 */
    if (!left_present) {
        gCfg.press_enable_L = 0;
        HeatPWMSet(Left, 0);
        HeatPower(Left, 0);
        VALVE_LEFT(0);
    }
    if (!right_present) {
        gCfg.press_enable_R = 0;
        HeatPWMSet(Right, 0);
        HeatPower(Right, 0);
        VALVE_RIGHT(0);
    }

#ifndef IGNORE_FUSE_PROTECTION_DEBUG
    /* 熔断：强制关闭对应侧 */
    if (!left_fuse) {
        gCfg.press_enable_L = 0;
        HeatPWMSet(Left, 0);
        HeatPower(Left, 0);
        VALVE_LEFT(0);
    }
    if (!right_fuse) {
        gCfg.press_enable_R = 0;
        HeatPWMSet(Right, 0);
        HeatPower(Right, 0);
        VALVE_RIGHT(0);
    }
#else
    /* 调试模式：只告警，不因熔断自动关停 */
    static uint8_t fuse_warned = 0;
    if (!fuse_warned && (!left_fuse || !right_fuse)) {
        fuse_warned = 1;
        LOG_W("DEBUG: ignore fuse, heaters stay enabled (L_fuse=%u R_fuse=%u)", left_fuse, right_fuse);
    }
#endif
}

void ControlTask(void *argument)
{
    (void)argument;

    /* 初始化控制配置与 PID */
    memset(&gCfg, 0, sizeof(gCfg));

    PID_Init(&pid_heat_L, 390, 1.8, 200, 100000, 0, 1999, 0, 0);
    PID_Init(&pid_heat_R, 390, 1.8, 200, 100000, 0, 1999, 0, 0);
    PID_Init(&pid_press, 420, 500, 0, 1, 0, 255, 0, 0);
    HAL_TIM_PWM_Start(&htim15, TIM_CHANNEL_1); // 保证气泵 PWM 定时器已启动

    TickType_t next_tx_tick = 0;
    uint8_t heater_status_div = 0;
    const char *phase = "idle";

    /* 发送侧压力中位数滤波窗口 */
    float tx_press_buf_l[TX_PRESS_MEDIAN_WINDOW] = {0};
    float tx_press_buf_r[TX_PRESS_MEDIAN_WINDOW] = {0};
    uint8_t tx_press_idx_l = 0;
    uint8_t tx_press_idx_r = 0;
    uint8_t tx_press_count_l = 0;
    uint8_t tx_press_count_r = 0;

    apply_idle_outputs();
    ctrl_state = CTRL_STATE_IDLE;

    for (;;)
    {
        vTaskDelay(pdMS_TO_TICKS(2));

        /* 1) 处理来自 AppTask 的控制命令 */
        ctrl_cmd_t c;
        if (xQueueReceive(gCtrlCmdQueue, &c, 0) == pdPASS)
        {
            if (c.id == CTRL_CMD_STOP) {
                gCfg.running = 0;
                emergency_stop = 0;
                pre_r_vent_done = 0;
                ctrl_state = CTRL_STATE_IDLE;
                apply_idle_outputs();
            } else if (c.id == CTRL_CMD_START) {
                gCfg = c.cfg;
                gCfg.running = 1;
                emergency_stop = 0;
                pre_r_vent_done = 0;
                /* 仅在收到 START 命令时执行一次 OTP_Reset */
                if (gCfg.press_enable_L) {
                    OTP_Reset(Left);
                }
                if (gCfg.press_enable_R) {
                    OTP_Reset(Right);
                }
                pid_heat_L.setpoint = (float)(gCfg.temp_target * 100.0f);
                pid_heat_R.setpoint = (float)(gCfg.temp_target * 100.0f);
                start_pressure_cycle(1);
            } else if (c.id == CTRL_CMD_UPDATE_CFG) {
                  /* 仅在收到 START 命令时执行一次 OTP_Reset */
                if (gCfg.press_enable_L) {
                    OTP_Reset(Left);
                }
                if (gCfg.press_enable_R) {
                    OTP_Reset(Right);
                }
                gCfg = c.cfg;
                if (gCfg.running) {
                    start_pressure_cycle(0); // 运行中刷新周期，不重复预泄压
                }
                pid_heat_L.setpoint = (float)(gCfg.temp_target * 100.0f);
                pid_heat_R.setpoint = (float)(gCfg.temp_target * 100.0f);
            }
        }

        /* 2) 空闲/保护路径：未运行、急停或双侧都禁用 */
        if (!gCfg.running || emergency_stop || (!gCfg.press_enable_L && !gCfg.press_enable_R)) {
            static uint8_t last_idle_reason = 0xFF;
            uint8_t idle_reason = 0;
            bool reason_not_running = !gCfg.running;
            bool reason_emergency = (emergency_stop != 0);
            bool reason_both_disabled = (!gCfg.press_enable_L && !gCfg.press_enable_R);

            if (reason_not_running) idle_reason |= 0x01;
            if (reason_emergency) idle_reason |= 0x02;
            if (reason_both_disabled) idle_reason |= 0x04;

            if (idle_reason != last_idle_reason) {
                last_idle_reason = idle_reason;
                if (reason_not_running || reason_emergency || reason_both_disabled) {
                    LOG_W("控制空闲：%s%s%s(run=%u emg=%u enL=%u enR=%u)",
                          reason_not_running ? "未启动; " : "",
                          reason_emergency ? "急停; " : "",
                          reason_both_disabled ? "双侧压力禁用; " : "",
                          gCfg.running, emergency_stop, gCfg.press_enable_L, gCfg.press_enable_R);
                } else {
                    LOG_W("控制空闲：未知(run=%u emg=%u enL=%u enR=%u)",
                          gCfg.running, emergency_stop, gCfg.press_enable_L, gCfg.press_enable_R);
                }
            }

            ctrl_state = CTRL_STATE_IDLE;
            phase = "idle";
            apply_idle_outputs();
        } else {
            /* 3) 运行路径：执行压力状态机 */
            uint32_t t1_ms = (uint32_t)(gCfg.t1_rise_s * 1000.0f);
            uint32_t t2_ms = (uint32_t)(gCfg.t2_hold_s * 1000.0f);
            uint32_t t3_ms = (uint32_t)(gCfg.t3_pulse_s * 1000.0f);

            if (ctrl_state == CTRL_STATE_IDLE) {
                start_pressure_cycle(1);
            }

            float Pset_L = 0.0f, Pset_R = 0.0f;
            float Pmax = gCfg.press_target_max;
            bool pump_on = false;
            bool inflating_phase = false;
            bool on_phase = false;

            switch (ctrl_state) {
                case CTRL_STATE_PRE_RISE_VENT:
                    phase = "pre_vent";
                    if (!pre_vent_close_done && (int32_t)(xTaskGetTickCount() - pre_vent_end_tick) >= 0) {
                        pre_vent_close_done = 1;
                        pre_r_vent_done = 1;
                        enter_state(CTRL_STATE_RISE);
                    }
                    break;

                case CTRL_STATE_RISE: {
                    phase = "rise";
                    VALVE_LEFT(1);
                    VALVE_RIGHT(1); // 充气
                    uint32_t elapsed = ms_since_tick(state_enter_tick);
                    float ratio = (t1_ms > 0) ? (float)elapsed / (float)t1_ms : 1.0f;
                    if (ratio > 1.0f) ratio = 1.0f;
                    Pset_L = Pset_R = Pmax * ratio;
                    inflating_phase = true;

                    if (t1_ms == 0 || elapsed >= t1_ms) {
                        if (t2_ms > 0) {
                            enter_state(CTRL_STATE_HOLD);
                        } else if (t3_ms > 0) {
                            enter_state(CTRL_STATE_PULSE);
                        } else {
                            start_inter_cycle_vent();
                        }
                    }
                    break;
                }

                case CTRL_STATE_HOLD: {
                    phase = "hold";
                    Pset_L = Pset_R = Pmax;
                    inflating_phase = true;
                    uint32_t elapsed = ms_since_tick(state_enter_tick);
                    if (t2_ms == 0 || elapsed >= t2_ms) {
                        if (t3_ms > 0) {
                            enter_state(CTRL_STATE_PULSE);
                        } else {
                            start_inter_cycle_vent();
                        }
                    }
                    break;
                }

                case CTRL_STATE_PULSE: {
                    uint32_t elapsed = ms_since_tick(state_enter_tick);
                    uint32_t period = (uint32_t)(gCfg.pulse_on_ms + gCfg.pulse_off_ms);
                    on_phase = (period > 0 && gCfg.pulse_on_ms > 0) ?
                               ((elapsed % period) < (uint32_t)gCfg.pulse_on_ms) : false;
                    phase = on_phase ? "pulse_on" : "pulse_off";
                    Pset_L = Pset_R = on_phase ? Pmax : 0.0f;
                    inflating_phase = on_phase;

                    if (t3_ms == 0 || elapsed >= t3_ms) {
                        start_inter_cycle_vent();
                    }
                    break;
                }

                case CTRL_STATE_INTER_CYCLE_VENT:
                    phase = "vent";
                    VALVE_LEFT(0);
                    VALVE_RIGHT(0);
                    TIM15->CCR1 = 0;
                    if ((int32_t)(xTaskGetTickCount() - inter_vent_end_tick) >= 0) {
                        start_pressure_cycle(0);
                    }
                    break;

                default:
                    break;
            }

            if (!gCfg.press_enable_L) Pset_L = 0.0f;
            if (!gCfg.press_enable_R) Pset_R = 0.0f;

            /* 迟滞判断是否需要补压，避免抖动 */
            bool needL = (Pset_L > gSensorData.pressL + 0.2f);
            bool needR = (Pset_R > gSensorData.pressR + 0.2f);

            float set_eff = 0.0f;
            float meas_eff = 0.0f;
            bool force_pump = (ctrl_state == CTRL_STATE_RISE); // 上升阶段尽量连续驱动

            if (inflating_phase) {
                uint8_t left_valve_state = gCfg.press_enable_L;
                uint8_t right_valve_state = gCfg.press_enable_R;

                VALVE_LEFT(left_valve_state);
                VALVE_RIGHT(right_valve_state);

                if (force_pump) {
                    if (gCfg.press_enable_L && gCfg.press_enable_R) {
                        set_eff = (Pset_L > Pset_R) ? Pset_L : Pset_R;
                        meas_eff = (gSensorData.pressL < gSensorData.pressR) ? gSensorData.pressL : gSensorData.pressR;
                    } else if (gCfg.press_enable_L) {
                        set_eff = Pset_L;
                        meas_eff = gSensorData.pressL;
                    } else if (gCfg.press_enable_R) {
                        set_eff = Pset_R;
                        meas_eff = gSensorData.pressR;
                    }
                    pump_on = (gCfg.press_enable_L || gCfg.press_enable_R);
                } else {
                    if (needL && needR) {
                        set_eff = (Pset_L > Pset_R) ? Pset_L : Pset_R;
                        meas_eff = (gSensorData.pressL < gSensorData.pressR) ? gSensorData.pressL : gSensorData.pressR;
                        pump_on = true;
                    } else if (needL) {
                        set_eff = Pset_L;
                        meas_eff = gSensorData.pressL;
                        pump_on = true;
                    } else if (needR) {
                        set_eff = Pset_R;
                        meas_eff = gSensorData.pressR;
                        pump_on = true;
                    } else {
                        TIM15->CCR1 = 0;
                    }
                }
            } else {
                TIM15->CCR1 = 0;
                VALVE_LEFT(0);
                VALVE_RIGHT(0); // 泄压
            }

            /* 压力 PID -> 气泵 PWM */
            if (pump_on) {
                pid_press.setpoint = set_eff;
                int32_t u = PID_Compute(&pid_press, meas_eff);
                if (u < 20) u = 20;
                if (u > 255) u = 255;
                TIM15->CCR1 = (uint16_t)u;
            }

            /* 温控分通道执行 */
            if (gCfg.press_enable_L) {
                HeatPower(Left, 1);
                PID_Heat(Left, &pid_heat_L, (float)(gSensorData.tempL * 100.0f));
            } else {
                HeatPWMSet(Left, 0);
                HeatPower(Left, 0);
            }

            if (gCfg.press_enable_R) {
                HeatPower(Right, 1);
                PID_Heat(Right, &pid_heat_R, (float)(gSensorData.tempR * 100.0f));
            } else {
                HeatPWMSet(Right, 0);
                HeatPower(Right, 0);
            }
        }

telemetry:
        /* 4) 遥测上报：运行时按节拍发送 */
        if (gCfg.running) {
            TickType_t now = xTaskGetTickCount();
            if ((int32_t)(now - next_tx_tick) >= 0) {
                next_tx_tick = now + pdMS_TO_TICKS(20);

                tx_frame_t tx;

                /* 压力发送前再做一次中位数滤波 */
                tx_press_buf_l[tx_press_idx_l] = gSensorData.pressL;
                tx_press_idx_l = (uint8_t)((tx_press_idx_l + 1U) % TX_PRESS_MEDIAN_WINDOW);
                if (tx_press_count_l < TX_PRESS_MEDIAN_WINDOW) tx_press_count_l++;

                tx_press_buf_r[tx_press_idx_r] = gSensorData.pressR;
                tx_press_idx_r = (uint8_t)((tx_press_idx_r + 1U) % TX_PRESS_MEDIAN_WINDOW);
                if (tx_press_count_r < TX_PRESS_MEDIAN_WINDOW) tx_press_count_r++;

                float tx_press_l = median_filter_tx(tx_press_buf_l, tx_press_count_l);
                float tx_press_r = median_filter_tx(tx_press_buf_r, tx_press_count_r);

                tx.type = TX_DATA_FLOAT;
                tx.frame_id = F32_LEFT_PRESSURE_VALUE;
                tx.v.f32 = tx_press_l;
                xQueueSend(gTxQueue, &tx, 0);

                tx.frame_id = F32_RIGHT_PRESSURE_VALUE;
                tx.v.f32 = tx_press_r;
                xQueueSend(gTxQueue, &tx, 0);

                tx.frame_id = F32_LEFT_TEMP_VALUE;
                tx.v.f32 = gSensorData.tempL;
                xQueueSend(gTxQueue, &tx, 0);

                tx.frame_id = F32_RIGHT_TEMP_VALUE;
                tx.v.f32 = gSensorData.tempR;
                xQueueSend(gTxQueue, &tx, 0);

                tx.type = TX_DATA_UINT8;
                tx.frame_id = U8_MODE_CURVES;
                tx.v.u8 = (uint8_t)(phase[0]); // r/h/p
                xQueueSend(gTxQueue, &tx, 0);

                tx.type = TX_DATA_UINT8;

                /* 调试参考量（当前未打印） */
                uint16_t pump_pwm = TIM15->CCR1;
                uint16_t heat_pwm_L = TIM14->CCR1;
                uint16_t heat_pwm_R = TIM17->CCR1;
                (void)pump_pwm;
                (void)heat_pwm_L;
                (void)heat_pwm_R;
            }
        } else {
            /* 停机时重置发送滤波窗口，避免旧值带入 */
            next_tx_tick = xTaskGetTickCount() + pdMS_TO_TICKS(20);
            tx_press_idx_l = tx_press_idx_r = 0;
            tx_press_count_l = tx_press_count_r = 0;
        }

        /* 约每 100ms 上报一次加热盾状态 */
        if (++heater_status_div >= 10) {
            heater_status_div = 0;
            SendHeaterStatusFrames();
        }
    }
}
