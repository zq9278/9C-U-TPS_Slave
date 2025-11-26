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

static PID_TypeDef pid_press;
static PID_TypeDef pid_heat_L, pid_heat_R;

static control_config_t gCfg;
static TickType_t t_start_tick = 0;
static uint8_t emergency_stop = 0;

typedef enum {
    CTRL_STATE_IDLE = 0,
    CTRL_STATE_PRE_RISE_VENT,
    CTRL_STATE_RISE,
    CTRL_STATE_HOLD,
    CTRL_STATE_PULSE,
    CTRL_STATE_INTER_CYCLE_VENT,
} ctrl_state_t;

static ctrl_state_t ctrl_state = CTRL_STATE_IDLE;
static TickType_t state_enter_tick = 0;
static TickType_t cycle_start_tick = 0;
static TickType_t pre_vent_end_tick = 0;
static TickType_t inter_vent_end_tick = 0;
static uint8_t pre_vent_close_done = 0;
static uint8_t pre_r_vent_done = 0; // 上升阶段前的泄压动作是否已执行

static inline uint32_t ms_since_tick(TickType_t tick)
{
    TickType_t now = xTaskGetTickCount();
    return (uint32_t)((now - tick) * portTICK_PERIOD_MS);
}

static void enter_state(ctrl_state_t s)
{
    ctrl_state = s;
    state_enter_tick = xTaskGetTickCount();
}

static void apply_idle_outputs(void)
{
    HeatPWMSet(Left, 0);  HeatPower(Left, 0);
    HeatPWMSet(Right, 0); HeatPower(Right, 0);
    VALVE_LEFT(0); VALVE_RIGHT(0);
    TIM15->CCR1 = 0;
}

static void start_pressure_cycle(uint8_t allow_pre_vent)
{
    cycle_start_tick = xTaskGetTickCount();
    state_enter_tick = cycle_start_tick;
    t_start_tick = cycle_start_tick;
    if (allow_pre_vent && !pre_r_vent_done) {
        ctrl_state = CTRL_STATE_PRE_RISE_VENT;
        pre_vent_end_tick = state_enter_tick + pdMS_TO_TICKS(1000);
        pre_vent_close_done = 0;
        VALVE_LEFT(0); VALVE_RIGHT(0); // 通电泄气
        TIM15->CCR1 = 0;
    } else {
        ctrl_state = CTRL_STATE_RISE;
    }
}

static void start_inter_cycle_vent(void)
{
    ctrl_state = CTRL_STATE_INTER_CYCLE_VENT;
    state_enter_tick = xTaskGetTickCount();
    inter_vent_end_tick = state_enter_tick + pdMS_TO_TICKS(2000);
    VALVE_LEFT(0); VALVE_RIGHT(0);
    TIM15->CCR1 = 0;
}

static void SendHeaterStatusFrames(void)
{
    tx_frame_t tx = {0};
    uint8_t right_present = (HAL_GPIO_ReadPin(MCU_Heat1_Sense_GPIO_Port, MCU_Heat1_Sense_Pin) == GPIO_PIN_RESET) ? 0 : 1; // 低电平表示不存在，给0
    uint8_t left_present  = (HAL_GPIO_ReadPin(MCU_Heat2_Sense_GPIO_Port, MCU_Heat2_Sense_Pin) == GPIO_PIN_RESET) ? 0 : 1; // 低电平表示不存在，给0
    uint8_t right_fuse    = (HAL_GPIO_ReadPin(Heat1_Fuse_Detection_GPIO_Port, Heat1_Fuse_Detection_Pin) == GPIO_PIN_RESET) ? 0 : 1; // 低电平表示熔断，给0
    uint8_t left_fuse     = (HAL_GPIO_ReadPin(Heat2_Fuse_Detection_GPIO_Port, Heat2_Fuse_Detection_Pin) == GPIO_PIN_RESET) ? 0 : 1; // 低电平表示熔断，给0

    tx.type = TX_DATA_UINT8;
    tx.frame_id = U8_LEFT_HEATER_PRESENT;  tx.v.u8 = left_present;  xQueueSend(gTxQueue, &tx, 0);
    tx.frame_id = U8_RIGHT_HEATER_PRESENT; tx.v.u8 = right_present; xQueueSend(gTxQueue, &tx, 0);
    tx.frame_id = U8_LEFT_HEATER_FUSE;     tx.v.u8 = left_fuse;     xQueueSend(gTxQueue, &tx, 0);
    tx.frame_id = U8_RIGHT_HEATER_FUSE;    tx.v.u8 = right_fuse;    xQueueSend(gTxQueue, &tx, 0);

    /* 熔断或不存在时关闭对应侧阀门/加热 */
    if (!left_present || !left_fuse) {
        gCfg.press_enable_L = 0;
        HeatPWMSet(Left, 0);
        HeatPower(Left, 0);
        VALVE_LEFT(0);
    }
    if (!right_present || !right_fuse) {
        gCfg.press_enable_R = 0;
        HeatPWMSet(Right, 0);
        HeatPower(Right, 0);
        VALVE_RIGHT(0);
    }
}

void ControlTask(void *argument)
{
    (void)argument;
    /* Initialize control config and PID controllers (heating left/right, pressure) */
    memset(&gCfg, 0, sizeof(gCfg));

    PID_Init(&pid_heat_L, 400, 2, 200, 100000, 0, 1999, 0, 0);
    PID_Init(&pid_heat_R, 400, 2, 200, 100000, 0, 1999, 0, 0);
    PID_Init(&pid_press, 5000, 0, 0, 1, 0, 255, 0, 0);
    HAL_TIM_PWM_Start(&htim15, TIM_CHANNEL_1); // ensure pump PWM timer is running

    uint8_t last_tx_100ms = 0;
    uint8_t heater_status_div = 0; // 节流加热器状态上报
    const char *phase = "idle"; // 当前阶段标签，默认闲置
    apply_idle_outputs();
    ctrl_state = CTRL_STATE_IDLE;
    for(;;)
    {
        vTaskDelay(pdMS_TO_TICKS(2));

        /* Handle control commands from AppTask (start/stop/update config) */
        ctrl_cmd_t c;
        if (xQueueReceive(gCtrlCmdQueue, &c, 0) == pdPASS) // 有新控制命令
        {
            //LOG_I("ctrl cmd recv id=%d", c.id);
            if (c.id == CTRL_CMD_STOP) { // 停止治疗
                gCfg.running = 0;       // 标记停止
                emergency_stop = 0;     // 清紧急停
                pre_r_vent_done = 0;    // 下次启动可再做预泄压
                ctrl_state = CTRL_STATE_IDLE;
                apply_idle_outputs();
            } else if (c.id == CTRL_CMD_START) { // 启动治疗
                gCfg = c.cfg;                     // 应用新配置
                gCfg.running = 1;                 // 标记运行
                emergency_stop = 0;               // 清紧急停
                pre_r_vent_done = 0;              // 允许预泄压
                pid_heat_L.setpoint = (float)(gCfg.temp_target * 100.0f); // 左加热目标
                pid_heat_R.setpoint = (float)(gCfg.temp_target * 100.0f); // 右加热目标
                start_pressure_cycle(1);
            } else if (c.id == CTRL_CMD_UPDATE_CFG) { // 仅更新配置
                gCfg = c.cfg;                         // 应用新配置
                if (gCfg.running) {
                    start_pressure_cycle(0); // 运行中刷新周期，但不重复预泄压
                }
                pid_heat_L.setpoint = (float)(gCfg.temp_target * 100.0f); // 左加热目标
                pid_heat_R.setpoint = (float)(gCfg.temp_target * 100.0f); // 右加热目标
            }
        }



        /* Safety/idle path: not running or both sides disabled -> stop pump/heat and vent */
        if (!gCfg.running || emergency_stop || (!gCfg.press_enable_L && !gCfg.press_enable_R)) { // 未运行或急停或两侧都禁用
            static uint8_t last_idle_reason = 0xFF;
            uint8_t idle_reason = 0;
            if (!gCfg.running) idle_reason |= 0x01;        // bit0: not running
            if (emergency_stop) idle_reason |= 0x02;       // bit1: emergency stop
            if (!gCfg.press_enable_L && !gCfg.press_enable_R) idle_reason |= 0x04; // bit2: both disabled
            if (idle_reason != last_idle_reason) {
                last_idle_reason = idle_reason;
                LOG_W("ctrl idle reason mask=0x%02X run=%u emg=%u enL=%u enR=%u", idle_reason, gCfg.running, emergency_stop, gCfg.press_enable_L, gCfg.press_enable_R);
            }

            ctrl_state = CTRL_STATE_IDLE;
            phase = "idle";
            apply_idle_outputs();
        } else {
            /* Pressure profile state machine: idle/pre-vent/rise/hold/pulse */
            uint32_t t1_ms = (uint32_t)(gCfg.t1_rise_s * 1000.0f);
            uint32_t t2_ms = (uint32_t)(gCfg.t2_hold_s * 1000.0f);
            uint32_t t3_ms = (uint32_t)(gCfg.t3_pulse_s * 1000.0f);

            if (ctrl_state == CTRL_STATE_IDLE) {
                start_pressure_cycle(1); // 运行状态但还未启动周期时触发
            }

            float Pset_L = 0.0f, Pset_R = 0.0f;
            float Pmax = gCfg.press_target_max;
            bool pump_on = false;
            bool inflating_phase = false;
            bool on_phase = false;

            switch (ctrl_state) {
                case CTRL_STATE_PRE_RISE_VENT:
                    phase = "pre_vent";
                    /* 预泄压到时后关闭阀并进入上升阶段 */
                    if (!pre_vent_close_done && (int32_t)(xTaskGetTickCount() - pre_vent_end_tick) >= 0) {
                        // VALVE_LEFT(1); VALVE_RIGHT(1); // 通电泄气
                        // osDelay(1000);
                        pre_vent_close_done = 1;
                        pre_r_vent_done = 1;
                        enter_state(CTRL_STATE_RISE);
                    }
                    break;

                case CTRL_STATE_RISE: {
                    phase = "rise";
                    VALVE_LEFT(1); VALVE_RIGHT(1); // 断电充气
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
                            start_inter_cycle_vent(); // ???????2s???
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
                    VALVE_LEFT(0); VALVE_RIGHT(0);
                    TIM15->CCR1 = 0;
                    if ((int32_t)(xTaskGetTickCount() - inter_vent_end_tick) >= 0) {
                        start_pressure_cycle(0);
                    }
                    break;

                default:
                    break;
            }

            if (!gCfg.press_enable_L) Pset_L = 0.0f; // 左侧禁用则目标为0
            if (!gCfg.press_enable_R) Pset_R = 0.0f; // 右侧禁用则目标为0

            /* Decide whether each side needs more pressure (hysteresis +0.2kPa) */
            bool needL = (Pset_L > gSensorData.pressL + 0.2f); // 左侧是否需要补压
            bool needR = (Pset_R > gSensorData.pressR + 0.2f); // 右侧是否需要补压

            float set_eff = 0.0f;
            float meas_eff = 0.0f;
            bool force_pump = (ctrl_state == CTRL_STATE_RISE); // 上升阶段要求泵连续运行

            /* Valve/pump selection:
             * - Both need: open both valves, track higher setpoint vs lower measured side
             * - Single side need: open corresponding valve only
             * - None: close/vent and stop pump
             */
            if (inflating_phase) {
                /* 充气阶段：阀保持断电充气（1），按需要决定是否开泵 */
                uint8_t left_valve_state  = gCfg.press_enable_L;
                uint8_t right_valve_state = gCfg.press_enable_R;

                VALVE_LEFT(left_valve_state);
                VALVE_RIGHT(right_valve_state);

                if (force_pump) { // 上升阶段：泵保持连续
                    if (gCfg.press_enable_L && gCfg.press_enable_R) {
                        set_eff  = (Pset_L > Pset_R) ? Pset_L : Pset_R;
                        meas_eff = (gSensorData.pressL < gSensorData.pressR) ? gSensorData.pressL : gSensorData.pressR;
                    } else if (gCfg.press_enable_L) {
                        set_eff  = Pset_L;
                        meas_eff = gSensorData.pressL;
                    } else if (gCfg.press_enable_R) {
                        set_eff  = Pset_R;
                        meas_eff = gSensorData.pressR;
                    }
                    pump_on = (gCfg.press_enable_L || gCfg.press_enable_R);
                } else { // 其他充气阶段按需启停泵
                    if (needL && needR) {
                        set_eff  = (Pset_L > Pset_R) ? Pset_L : Pset_R;
                        meas_eff = (gSensorData.pressL < gSensorData.pressR) ? gSensorData.pressL : gSensorData.pressR;
                        pump_on = true;
                    } else if (needL) {
                        set_eff  = Pset_L;
                        meas_eff = gSensorData.pressL;
                        pump_on = true;
                    } else if (needR) {
                        set_eff  = Pset_R;
                        meas_eff = gSensorData.pressR;
                        pump_on = true;
                    } else {
                        TIM15->CCR1 = 0;
                    }
                }
            } else {
                /* 泄气阶段：阀通电泄气，停泵 */
                TIM15->CCR1 = 0;
                osDelay(500);
                VALVE_LEFT(0); VALVE_RIGHT(0);
                
            }

            /* Pump PID control (pressure) */
            if (pump_on) {
                pid_press.setpoint = (float)(set_eff * 1.0f); // 设定压 x1000
                int32_t u = PID_Compute(&pid_press, (float)(meas_eff * 1.0f)); // PID输出
                if (u < 20) u = 20; // 下限
                /* 上升阶段保持泵连续转动，避免PWM降为0导致反复启停 */
                //if (ctrl_state == CTRL_STATE_RISE && u < 100) u = 10;
                if (u > 255) u = 255; // 上限
                TIM15->CCR1 = (uint16_t)u; // 写泵PWM
            }

            /* Heating control per side */
            if (gCfg.press_enable_L) { // 左侧启用
                HeatPower(Left, 1); // 开左加热电源
                PID_Heat(Left, &pid_heat_L, (float)(gSensorData.tempL * 100.0f)); // 左温度PID
            } else {
                HeatPWMSet(Left, 0); // 关左PWM
                HeatPower(Left, 0);  // 断左加热
            }

            if (gCfg.press_enable_R) { // 右侧启用
                HeatPower(Right, 1); // 开右加热电源
                PID_Heat(Right, &pid_heat_R, (float)(gSensorData.tempR * 100.0f)); // 右温度PID
            } else {
                HeatPWMSet(Right, 0); // 关右PWM
                HeatPower(Right, 0);  // 断右加热
            }
        }

telemetry:
        /* Telemetry @100ms when running: send pressures, temps, heater status */
        if (gCfg.running) { // 仅运行时发送
            if (++last_tx_100ms >= 2) { // 2*10ms=20ms? 此处计数基于10ms循环
                last_tx_100ms = 0; // 复位计数
                tx_frame_t tx;
                tx.type = TX_DATA_FLOAT; // 浮点数据
                tx.frame_id = F32_LEFT_PRESSURE_VALUE;  tx.v.f32 = gSensorData.pressL; xQueueSend(gTxQueue, &tx, 0); // 左压
                tx.frame_id = F32_RIGHT_PRESSURE_VALUE; tx.v.f32 = gSensorData.pressR; xQueueSend(gTxQueue, &tx, 0); // 右压
                tx.frame_id = F32_LEFT_TEMP_VALUE;      tx.v.f32 = gSensorData.tempL;  xQueueSend(gTxQueue, &tx, 0); // 左温
                tx.frame_id = F32_RIGHT_TEMP_VALUE;     tx.v.f32 = gSensorData.tempR;  xQueueSend(gTxQueue, &tx, 0); // 右温
                tx.type = TX_DATA_UINT8;
                tx.frame_id = U8_MODE_CURVES;           tx.v.u8 = (uint8_t)(phase[0]); xQueueSend(gTxQueue, &tx, 0); // 用首字母标记阶段（r/h/p)
                //LOG_I("phase telem frame 0x%04X val=%c", tx.frame_id, (char)tx.v.u8);
                tx.type = TX_DATA_UINT8; // 切回u8类型（备用）

                /* Log motor/heater power and channel enable states */
                uint16_t pump_pwm = TIM15->CCR1;  // 0..255 泵PWM
                uint16_t heat_pwm_L = TIM14->CCR1; // 0..1999 左加热PWM
                uint16_t heat_pwm_R = TIM17->CCR1; // 0..1999 右加热PWM
                // LOG_I("ctrl phase=%s run=%u pump=%u/255 heatL=%u/1999 heatR=%u/1999 enL=%u enR=%u T(L,R)=%.2f,%.2f PL,PR=%.2f,%.2f\n",
                //       phase,
                //       gCfg.running, pump_pwm, heat_pwm_L, heat_pwm_R,
                //       gCfg.press_enable_L, gCfg.press_enable_R,
                //       gSensorData.tempL, gSensorData.tempR,
                //       gSensorData.pressL, gSensorData.pressR); // 打印控制状态

                
            }
        } else {
            last_tx_100ms = 0; // reset counter when stopped
        }
        /* 加热器存在/熔断状态节流上报（约~100ms，一直发送） */
        if (++heater_status_div >= 10) {
            heater_status_div = 0;
            SendHeaterStatusFrames();
        }
    }
}
