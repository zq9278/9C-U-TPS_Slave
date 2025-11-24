#include "Uart_Communicate.h"
#include <string.h>
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
static uint32_t t_start_tick = 0;
static uint8_t emergency_stop = 0;

static inline uint32_t ms_since_start(void)
{
    TickType_t now = xTaskGetTickCount();
    return (uint32_t)((now - t_start_tick) * portTICK_PERIOD_MS);
}

static void SendHeaterStatusFrames(void)
{
    tx_frame_t tx = {0};
    uint8_t right_present = (HAL_GPIO_ReadPin(MCU_Heat1_Sense_GPIO_Port, MCU_Heat1_Sense_Pin) == GPIO_PIN_RESET) ? 0 : 1;//低电平表示不存在，给0
    uint8_t left_present  = (HAL_GPIO_ReadPin(MCU_Heat2_Sense_GPIO_Port, MCU_Heat2_Sense_Pin) == GPIO_PIN_RESET) ? 0 : 1;//低电平表示不存在，给0
    uint8_t right_fuse    = (HAL_GPIO_ReadPin(Heat1_Fuse_Detection_GPIO_Port, Heat1_Fuse_Detection_Pin) == GPIO_PIN_SET) ? 0 : 1;//高电平表示没熔断，给0
    uint8_t left_fuse     = (HAL_GPIO_ReadPin(Heat2_Fuse_Detection_GPIO_Port, Heat2_Fuse_Detection_Pin) == GPIO_PIN_SET) ? 0 : 1;//高电平表示没熔断，给0

    tx.type = TX_DATA_UINT8;
    tx.frame_id = U8_LEFT_HEATER_PRESENT;  tx.v.u8 = left_present;  xQueueSend(gTxQueue, &tx, 0);
    tx.frame_id = U8_RIGHT_HEATER_PRESENT; tx.v.u8 = right_present; xQueueSend(gTxQueue, &tx, 0);
    tx.frame_id = U8_LEFT_HEATER_FUSE;     tx.v.u8 = left_fuse;     xQueueSend(gTxQueue, &tx, 0);
    tx.frame_id = U8_RIGHT_HEATER_FUSE;    tx.v.u8 = right_fuse;    xQueueSend(gTxQueue, &tx, 0);
}

void ControlTask(void *argument)
{
    (void)argument;
    /* Initialize control config and PID controllers (heating left/right, pressure) */
    memset(&gCfg, 0, sizeof(gCfg));

    PID_Init(&pid_heat_L, 400, 2, 200, 100000, 0, 1999, 0, 0);
    PID_Init(&pid_heat_R, 400, 2, 200, 100000, 0, 1999, 0, 0);
    PID_Init(&pid_press, 500, 5, 50, 100000, 0, 255, 0, 0);
    HAL_TIM_PWM_Start(&htim15, TIM_CHANNEL_1); // ensure pump PWM timer is running

    uint8_t last_tx_100ms = 0;

    for(;;)
    {
        vTaskDelay(pdMS_TO_TICKS(10));

        /* Handle control commands from AppTask (start/stop/update config) */
        ctrl_cmd_t c;
        if (xQueueReceive(gCtrlCmdQueue, &c, 0) == pdPASS)
        {
            if (c.id == CTRL_CMD_STOP) {
                gCfg.running = 0;
                emergency_stop = 0;
            } else if (c.id == CTRL_CMD_START) {
                gCfg = c.cfg;
                gCfg.running = 1;
                emergency_stop = 0;
                t_start_tick = xTaskGetTickCount();
                pid_heat_L.setpoint = (int32_t)(gCfg.temp_target * 100.0f);
                pid_heat_R.setpoint = (int32_t)(gCfg.temp_target * 100.0f);
            } else if (c.id == CTRL_CMD_UPDATE_CFG) {
                gCfg = c.cfg;
                if (gCfg.running) {
                    t_start_tick = xTaskGetTickCount();
                }
                pid_heat_L.setpoint = (int32_t)(gCfg.temp_target * 100.0f);
                pid_heat_R.setpoint = (int32_t)(gCfg.temp_target * 100.0f);
            }
        }

        /* Safety/idle path: not running or both sides disabled -> stop pump/heat and vent */
        if (!gCfg.running || emergency_stop || (!gCfg.press_enable_L && !gCfg.press_enable_R)) {
            HeatPWMSet(Left, 0);  HeatPower(Left, 0);
            HeatPWMSet(Right, 0); HeatPower(Right, 0);
            AirValve1(1); AirValve2(1);
            TIM15->CCR1 = 0;
        } else {
            /* Pressure profile: rise -> hold -> pulse */
            uint32_t t1_ms = (uint32_t)(gCfg.t1_rise_s * 1000.0f);
            uint32_t t2_ms = (uint32_t)(gCfg.t2_hold_s * 1000.0f);
            uint32_t t3_ms = (uint32_t)(gCfg.t3_pulse_s * 1000.0f);
            uint32_t total_ms = t1_ms + t2_ms + t3_ms;
            if (total_ms == 0) total_ms = 1;

            uint32_t ms = ms_since_start();
            if (ms >= total_ms) {
                t_start_tick = xTaskGetTickCount();
                ms = 0;
            }

            float Pmax = gCfg.press_target_max;
            float Pset_L = 0.0f, Pset_R = 0.0f;

            if (ms < t1_ms) {
                float ratio = (t1_ms > 0) ? (float)ms / (float)t1_ms : 1.0f;
                Pset_L = Pset_R = Pmax * ratio;
            } else if (ms < t1_ms + t2_ms) {
                Pset_L = Pset_R = Pmax;
            } else {
                uint32_t pulse_elapsed = ms - (t1_ms + t2_ms);
                uint32_t period = (uint32_t)(gCfg.pulse_on_ms + gCfg.pulse_off_ms);
                bool on_phase = (period > 0 && gCfg.pulse_on_ms > 0) ?
                                ((pulse_elapsed % period) < (uint32_t)gCfg.pulse_on_ms) : false;
                float v = on_phase ? Pmax : 0.0f;
                Pset_L = v;
                Pset_R = v;
            }

            if (!gCfg.press_enable_L) Pset_L = 0.0f;
            if (!gCfg.press_enable_R) Pset_R = 0.0f;

            /* Decide whether each side needs more pressure (hysteresis +0.2kPa) */
            bool needL = (Pset_L > gSensorData.pressL + 0.2f);
            bool needR = (Pset_R > gSensorData.pressR + 0.2f);

            float set_eff = 0.0f;
            float meas_eff = 0.0f;
            bool pump_on = false;

            /* Valve/pump selection:
             * - Both need: open both valves, track higher setpoint vs lower measured side
             * - Single side need: open corresponding valve only
             * - None: close/vent and stop pump
             */
            if (needL && needR) {
                AirValve1(0); AirValve2(0);
                set_eff  = (Pset_L > Pset_R) ? Pset_L : Pset_R;
                meas_eff = (gSensorData.pressL < gSensorData.pressR) ? gSensorData.pressL : gSensorData.pressR;
                pump_on = true;
            } else if (needL) {
                AirValve2(0); AirValve1(1);
                set_eff  = Pset_L;
                meas_eff = gSensorData.pressL;
                pump_on = true;
            } else if (needR) {
                AirValve1(0); AirValve2(1);
                set_eff  = Pset_R;
                meas_eff = gSensorData.pressR;
                pump_on = true;
            } else {
                AirValve1(1); AirValve2(1);
                TIM15->CCR1 = 0;
            }

            /* Pump PID control (pressure) */
            if (pump_on) {
                pid_press.setpoint = (int32_t)(set_eff * 1000.0f);
                int32_t u = PID_Compute(&pid_press, (int32_t)(meas_eff * 1000.0f));
                if (u < 0) u = 0;
                if (u > 255) u = 255;
                TIM15->CCR1 = (uint16_t)u;
            }

            /* Heating control per side */
            if (gCfg.press_enable_L) {
                HeatPower(Left, 1);
                PID_Heat(Left, &pid_heat_L, (int32_t)(gSensorData.tempL * 100.0f));
            } else {
                HeatPWMSet(Left, 0);
                HeatPower(Left, 0);
            }

            if (gCfg.press_enable_R) {
                HeatPower(Right, 1);
                PID_Heat(Right, &pid_heat_R, (int32_t)(gSensorData.tempR * 100.0f));
            } else {
                HeatPWMSet(Right, 0);
                HeatPower(Right, 0);
            }
        }

        /* Telemetry @100ms: send pressures, temps, heater status */
        if (++last_tx_100ms >= 100) {
            last_tx_100ms = 0;
            tx_frame_t tx;
            tx.type = TX_DATA_FLOAT;
            tx.frame_id = F32_LEFT_PRESSURE_VALUE;  tx.v.f32 = gSensorData.pressL; xQueueSend(gTxQueue, &tx, 0);
            tx.frame_id = F32_RIGHT_PRESSURE_VALUE; tx.v.f32 = gSensorData.pressR; xQueueSend(gTxQueue, &tx, 0);
            tx.frame_id = F32_LEFT_TEMP_VALUE;      tx.v.f32 = gSensorData.tempL;  xQueueSend(gTxQueue, &tx, 0);
            tx.frame_id = F32_RIGHT_TEMP_VALUE;     tx.v.f32 = gSensorData.tempR;  xQueueSend(gTxQueue, &tx, 0);
            tx.type = TX_DATA_UINT8;
            //tx.frame_id = U8_HEARTBEAT_ACK;         tx.v.u8 = 1;                   xQueueSend(gTxQueue, &tx, 0);

            /* Log motor/heater power and channel enable states */
            uint16_t pump_pwm = TIM15->CCR1;  // 0..255
            uint16_t heat_pwm_L = TIM14->CCR1; // 0..1999
            uint16_t heat_pwm_R = TIM17->CCR1; // 0..1999
            LOG_I("ctrl run=%u pump=%u/255 heatL=%u/1999 heatR=%u/1999 enL=%u enR=%u T(L,R)=%.2f,%.2f PL,PR=%.2f,%.2f",
                  gCfg.running, pump_pwm, heat_pwm_L, heat_pwm_R,
                  gCfg.press_enable_L, gCfg.press_enable_R,
                  gSensorData.tempL, gSensorData.tempR,
                  gSensorData.pressL, gSensorData.pressR,
                  gCfg.press_enable_L, gCfg.press_enable_R);

            SendHeaterStatusFrames();
        }
    }
}
