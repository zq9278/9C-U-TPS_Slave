#include "Uart_Communicate.h"
#include <string.h>
#include "control_task.h"
#include "pid.h"
#include "heat.h"
#include "apply.h"
#include "tim.h"
#include "LOG.h"

#include "FreeRTOS.h"
#include "task.h"// Pressure PID instances
static PID_TypeDef pid_press_L, pid_press_R;
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
    TickType_t last = xTaskGetTickCount();
    memset(&gCfg, 0, sizeof(gCfg));

    // PID init (scale: heat PID expects setpoint x100 per existing code)
    PID_Init(&pid_heat_L, 400, 2, 200, 100000, 0, 1999, 0, 0);
    PID_Init(&pid_heat_R, 400, 2, 200, 100000, 0, 1999, 0, 0);

    // Simple pressure PID (arbitrary initial gains, to be tuned)
    PID_Init(&pid_press_L, 500, 5, 50,  100000, 0, 255, 0, 0);
    PID_Init(&pid_press_R, 500, 5, 50,  100000, 0, 255, 0, 0);

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

        // Pressure control (kPa -> setpoints, measure from gSensorData)
        float eL = Pset_L - gSensorData.pressL;
        float eR = Pset_R - gSensorData.pressR;
        pid_press_L.setpoint = (int32_t)(Pset_L * 1000.0f);
        pid_press_R.setpoint = (int32_t)(Pset_R * 1000.0f);
        int32_t uL = PID_Compute(&pid_press_L, (int32_t)(gSensorData.pressL * 1000.0f));
        int32_t uR = PID_Compute(&pid_press_R, (int32_t)(gSensorData.pressR * 1000.0f));
        uint16_t pwm = (uint16_t)((uL + uR) / 2); // single pump, average effort
        if (pwm > 255) pwm = 255;

        // Valve logic + pump
        if (Pset_L > gSensorData.pressL + 0.2f && Pset_R > gSensorData.pressR + 0.2f) {
            // inflate both
            AirValve1(0); AirValve2(0); TIM15->CCR1 = pwm;
        } else if (Pset_L > gSensorData.pressL + 0.2f) {
            // inflate left
            AirValve2(0); AirValve1(1); TIM15->CCR1 = pwm;
        } else if (Pset_R > gSensorData.pressR + 0.2f) {
            // inflate right
            AirValve1(0); AirValve2(1); TIM15->CCR1 = pwm;
        } else {
            // hold/deflate
            AirValve1(1); AirValve2(1); TIM15->CCR1 = 0;
        }

        // Temperature control (using project PID/heat API)
        pid_heat_L.setpoint = (int32_t)(gCfg.temp_target * 100.0f);
        pid_heat_R.setpoint = (int32_t)(gCfg.temp_target * 100.0f);
        PID_Heat(Left,  &pid_heat_L, (int32_t)(gSensorData.tempL * 100.0f));
        PID_Heat(Right, &pid_heat_R, (int32_t)(gSensorData.tempR * 100.0f));

        // Telemetry every 100ms
        if (++last_tx_100ms >= 10) {
            last_tx_100ms = 0;
            tx_frame_t tx;
            tx.type = TX_DATA_FLOAT;
            tx.frame_id = F32_LEFT_PRESSURE_VALUE;  tx.v.f32 = gSensorData.pressL; xQueueSend(gTxQueue, &tx, 0);
            tx.frame_id = F32_RIGHT_PRESSURE_VALUE; tx.v.f32 = gSensorData.pressR; xQueueSend(gTxQueue, &tx, 0);
            tx.frame_id = F32_LEFT_TEMP_VALUE;      tx.v.f32 = gSensorData.tempL;  xQueueSend(gTxQueue, &tx, 0);
            tx.frame_id = F32_RIGHT_TEMP_VALUE;     tx.v.f32 = gSensorData.tempR;  xQueueSend(gTxQueue, &tx, 0);
        }
    }
}

