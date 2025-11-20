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
// Pressure PID instance (single shared pump)
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

void ControlTask(void *argument)
{
    (void)argument;
    memset(&gCfg, 0, sizeof(gCfg));

    // PID init (scale: heat PID expects setpoint x100 per existing code)
    PID_Init(&pid_heat_L, 400, 2, 200, 100000, 0, 1999, 0, 0);
    PID_Init(&pid_heat_R, 400, 2, 200, 100000, 0, 1999, 0, 0);

    // Single pressure PID (gains to be tuned)
    PID_Init(&pid_press, 500, 5, 50,  100000, 0, 255, 0, 0);

    uint8_t last_tx_100ms = 0;

    for(;;)
    {
        vTaskDelay(pdMS_TO_TICKS(10));

        // Apply control commands
        ctrl_cmd_t c;
        if (xQueueReceive(gCtrlCmdQueue, &c, 0) == pdPASS)
        {
            if (c.id == CTRL_CMD_STOP) {
                gCfg.running = 0; emergency_stop = 0;
            } else if (c.id == CTRL_CMD_START) {
                gCfg = c.cfg; gCfg.running = 1; emergency_stop = 0; t_start_tick = xTaskGetTickCount();
                // update heat PID setpoint
                pid_heat_L.setpoint = (int32_t)(gCfg.temp_target * 100.0f);
                pid_heat_R.setpoint = (int32_t)(gCfg.temp_target * 100.0f);
            } else if (c.id == CTRL_CMD_UPDATE_CFG) {
                gCfg = c.cfg;
                pid_heat_L.setpoint = (int32_t)(gCfg.temp_target * 100.0f);
                pid_heat_R.setpoint = (int32_t)(gCfg.temp_target * 100.0f);
            }
        }

        // Safety stop
        if (!gCfg.running || emergency_stop) {
            // Heaters off
            HeatPWMSet(Left, 0); HeatPower(Left, 0);
            HeatPWMSet(Right, 0); HeatPower(Right, 0);
            // Valves release
            AirValve1(1); AirValve2(1);
            // Pump off
            TIM15->CCR1 = 0;
            continue;
        }

        // Compute phase
        uint32_t ms = ms_since_start();
        float Pmax = gCfg.press_target_max; // kPa
        float Pset_L = 0.0f, Pset_R = 0.0f;

        uint32_t t1_ms = (uint32_t)(gCfg.t1_rise_s * 1000.0f);
        uint32_t t2_ms = (uint32_t)(gCfg.t2_hold_s * 1000.0f);
        uint32_t t3_ms = (uint32_t)(gCfg.t3_pulse_s * 1000.0f);

        if (ms < t1_ms) {
            float ratio = (t1_ms>0) ? (float)ms / (float)t1_ms : 1.0f;
            Pset_L = Pset_R = Pmax * ratio;
        } else if (ms < t1_ms + t2_ms) {
            Pset_L = Pset_R = Pmax;
        } else if (ms < t1_ms + t2_ms + t3_ms && (gCfg.pulse_on_ms + gCfg.pulse_off_ms) > 0.0f) {
            uint32_t pulse_elapsed = ms - (t1_ms + t2_ms);
            uint32_t period = (uint32_t)(gCfg.pulse_on_ms + gCfg.pulse_off_ms);
            uint32_t pos = pulse_elapsed % period;
            bool on_phase = pos < (uint32_t)gCfg.pulse_on_ms;
            if (gCfg.squeeze_mode == 1) { // alternate
                if (on_phase) { Pset_R = Pmax; Pset_L = 0.0f; }
                else          { Pset_R = 0.0f; Pset_L = Pmax; }
            } else {
                float v = on_phase ? Pmax : 0.0f;
                Pset_L = v; Pset_R = v;
            }
        } else {
            // cycle end
            gCfg.running = 0;
            continue;
        }

        // Apply per-side enable flags: disabled side setpoint = 0 (release/hold off)
        if (!gCfg.press_enable_L) Pset_L = 0.0f;
        if (!gCfg.press_enable_R) Pset_R = 0.0f;

        // Decide which valves need to open this tick
        bool needL = (Pset_L > gSensorData.pressL + 0.2f);
        bool needR = (Pset_R > gSensorData.pressR + 0.2f);

        float set_eff = 0.0f;
        float meas_eff = 0.0f;

        if (needL && needR) {
            // Inflate both: control to the higher target using the lower measured side
            AirValve1(0); AirValve2(0);
            set_eff  = (Pset_L > Pset_R) ? Pset_L : Pset_R;
            meas_eff = (gSensorData.pressL < gSensorData.pressR) ? gSensorData.pressL : gSensorData.pressR;
        } else if (needL) {
            // Inflate left only
            AirValve2(0); AirValve1(1);
            set_eff  = Pset_L;
            meas_eff = gSensorData.pressL;
        } else if (needR) {
            // Inflate right only
            AirValve1(0); AirValve2(1);
            set_eff  = Pset_R;
            meas_eff = gSensorData.pressR;
        } else {
            // No inflation needed -> release/hold, pump off
            AirValve1(1); AirValve2(1);
            TIM15->CCR1 = 0;
            // Temperature control still runs below
            goto TEMP_CTRL;
        }

        // Single PID compute (kPa -> Pa*1e3 scaling)
        pid_press.setpoint = (int32_t)(set_eff * 1000.0f);
        int32_t u = PID_Compute(&pid_press, (int32_t)(meas_eff * 1000.0f));
        uint16_t pwm = (u < 0) ? 0 : (u > 255 ? 255 : (uint16_t)u);
        TIM15->CCR1 = pwm;

TEMP_CTRL:
        // Temperature control (per-side enable at runtime)
        extern uint8_t g_temp_enable_L, g_temp_enable_R;
        if (g_temp_enable_L) {
            pid_heat_L.setpoint = (int32_t)(gCfg.temp_target * 100.0f);
            PID_Heat(Left,  &pid_heat_L, (int32_t)(gSensorData.tempL * 100.0f));
        } else {
            pid_heat_L.setpoint = 0;
            HeatPWMSet(Left, 0);
        }
        if (g_temp_enable_R) {
            pid_heat_R.setpoint = (int32_t)(gCfg.temp_target * 100.0f);
            PID_Heat(Right, &pid_heat_R, (int32_t)(gSensorData.tempR * 100.0f));
        } else {
            pid_heat_R.setpoint = 0;
            HeatPWMSet(Right, 0);
        }

        // Telemetry every 100ms
        if (++last_tx_100ms >= 10) {
            last_tx_100ms = 0;
            tx_frame_t tx;
            tx.type = TX_DATA_FLOAT;
            tx.frame_id = F32_LEFT_PRESSURE_VALUE;  tx.v.f32 = gSensorData.pressL; xQueueSend(gTxQueue, &tx, 0);
            tx.frame_id = F32_RIGHT_PRESSURE_VALUE; tx.v.f32 = gSensorData.pressR; xQueueSend(gTxQueue, &tx, 0);
            tx.frame_id = F32_LEFT_TEMP_VALUE;      tx.v.f32 = gSensorData.tempL;  xQueueSend(gTxQueue, &tx, 0);
            tx.frame_id = F32_RIGHT_TEMP_VALUE;     tx.v.f32 = gSensorData.tempR;  xQueueSend(gTxQueue, &tx, 0);

            // Heater presence and fuse status
            uint8_t right_present = (HAL_GPIO_ReadPin(MCU_Heat1_Sense_GPIO_Port, MCU_Heat1_Sense_Pin) == GPIO_PIN_RESET) ? 1 : 0; // Heat1 low=present
            uint8_t left_present  = (HAL_GPIO_ReadPin(MCU_Heat2_Sense_GPIO_Port, MCU_Heat2_Sense_Pin) == GPIO_PIN_RESET) ? 1 : 0; // Heat2 low=present
            uint8_t right_fuse    = (HAL_GPIO_ReadPin(Heat1_Fuse_Detection_GPIO_Port, Heat1_Fuse_Detection_Pin) == GPIO_PIN_SET) ? 1 : 0; // high=blown
            uint8_t left_fuse     = (HAL_GPIO_ReadPin(Heat2_Fuse_Detection_GPIO_Port, Heat2_Fuse_Detection_Pin) == GPIO_PIN_SET) ? 1 : 0; // high=blown

            tx.type = TX_DATA_UINT8;
            tx.frame_id = U8_LEFT_HEATER_PRESENT;   tx.v.u8 = left_present;  xQueueSend(gTxQueue, &tx, 0);
            tx.frame_id = U8_RIGHT_HEATER_PRESENT;  tx.v.u8 = right_present; xQueueSend(gTxQueue, &tx, 0);
            tx.frame_id = U8_LEFT_HEATER_FUSE;      tx.v.u8 = left_fuse;     xQueueSend(gTxQueue, &tx, 0);
            tx.frame_id = U8_RIGHT_HEATER_FUSE;     tx.v.u8 = right_fuse;    xQueueSend(gTxQueue, &tx, 0);
        }
    }
}
