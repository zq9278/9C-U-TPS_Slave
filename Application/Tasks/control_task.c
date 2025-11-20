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
static uint8_t emergency_stop = 0;

static void SendHeaterStatusFrames(void)
{
    tx_frame_t tx = {0};
    uint8_t right_present = (HAL_GPIO_ReadPin(MCU_Heat1_Sense_GPIO_Port, MCU_Heat1_Sense_Pin) == GPIO_PIN_RESET) ? 1 : 0;
    uint8_t left_present  = (HAL_GPIO_ReadPin(MCU_Heat2_Sense_GPIO_Port, MCU_Heat2_Sense_Pin) == GPIO_PIN_RESET) ? 1 : 0;
    uint8_t right_fuse    = (HAL_GPIO_ReadPin(Heat1_Fuse_Detection_GPIO_Port, Heat1_Fuse_Detection_Pin) == GPIO_PIN_SET) ? 1 : 0;
    uint8_t left_fuse     = (HAL_GPIO_ReadPin(Heat2_Fuse_Detection_GPIO_Port, Heat2_Fuse_Detection_Pin) == GPIO_PIN_SET) ? 1 : 0;

    tx.type = TX_DATA_UINT8;
    tx.frame_id = U8_LEFT_HEATER_PRESENT;  tx.v.u8 = left_present;  xQueueSend(gTxQueue, &tx, 0);
    tx.frame_id = U8_RIGHT_HEATER_PRESENT; tx.v.u8 = right_present; xQueueSend(gTxQueue, &tx, 0);
    tx.frame_id = U8_LEFT_HEATER_FUSE;     tx.v.u8 = left_fuse;     xQueueSend(gTxQueue, &tx, 0);
    tx.frame_id = U8_RIGHT_HEATER_FUSE;    tx.v.u8 = right_fuse;    xQueueSend(gTxQueue, &tx, 0);
}

void ControlTask(void *argument)
{
    (void)argument;
    memset(&gCfg, 0, sizeof(gCfg));

    PID_Init(&pid_heat_L, 400, 2, 200, 100000, 0, 1999, 0, 0);
    PID_Init(&pid_heat_R, 400, 2, 200, 100000, 0, 1999, 0, 0);
    PID_Init(&pid_press, 500, 5, 50, 100000, 0, 255, 0, 0);

    uint8_t last_tx_100ms = 0;

    for(;;)
    {
        vTaskDelay(pdMS_TO_TICKS(10));

        ctrl_cmd_t c;
        if (xQueueReceive(gCtrlCmdQueue, &c, 0) == pdPASS)
        {
            if (c.id == CTRL_CMD_STOP) {
                gCfg.running = 0;
                emergency_stop = 0;
            } else if (c.id == CTRL_CMD_START || c.id == CTRL_CMD_UPDATE_CFG) {
                gCfg = c.cfg;
                gCfg.running = (c.id == CTRL_CMD_START) ? 1 : gCfg.running;
                pid_heat_L.setpoint = (int32_t)(gCfg.temp_target * 100.0f);
                pid_heat_R.setpoint = (int32_t)(gCfg.temp_target * 100.0f);
            }
        }

        if (!gCfg.running || emergency_stop || (!gCfg.press_enable_L && !gCfg.press_enable_R)) {
            HeatPWMSet(Left, 0);  HeatPower(Left, 0);
            HeatPWMSet(Right, 0); HeatPower(Right, 0);
            AirValve1(1); AirValve2(1);
            TIM15->CCR1 = 0;
        } else {
            float Pset_L = gCfg.press_enable_L ? gCfg.press_target_max : 0.0f;
            float Pset_R = gCfg.press_enable_R ? gCfg.press_target_max : 0.0f;

            bool needL = (Pset_L > gSensorData.pressL + 0.2f);
            bool needR = (Pset_R > gSensorData.pressR + 0.2f);

            float set_eff = 0.0f;
            float meas_eff = 0.0f;
            bool pump_on = false;

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

            if (pump_on) {
                pid_press.setpoint = (int32_t)(set_eff * 1000.0f);
                int32_t u = PID_Compute(&pid_press, (int32_t)(meas_eff * 1000.0f));
                if (u < 0) u = 0;
                if (u > 255) u = 255;
                TIM15->CCR1 = (uint16_t)u;
            }

            if (gCfg.press_enable_L) {
                HeatPower(Left, 1);
                pid_heat_L.setpoint = (int32_t)(gCfg.temp_target * 100.0f);
                PID_Heat(Left, &pid_heat_L, (int32_t)(gSensorData.tempL * 100.0f));
            } else {
                HeatPWMSet(Left, 0);
                HeatPower(Left, 0);
            }

            if (gCfg.press_enable_R) {
                HeatPower(Right, 1);
                pid_heat_R.setpoint = (int32_t)(gCfg.temp_target * 100.0f);
                PID_Heat(Right, &pid_heat_R, (int32_t)(gSensorData.tempR * 100.0f));
            } else {
                HeatPWMSet(Right, 0);
                HeatPower(Right, 0);
            }
        }

        if (++last_tx_100ms >= 10) {
            last_tx_100ms = 0;
            tx_frame_t tx;
            tx.type = TX_DATA_FLOAT;
            tx.frame_id = F32_LEFT_PRESSURE_VALUE;  tx.v.f32 = gSensorData.pressL; xQueueSend(gTxQueue, &tx, 0);
            tx.frame_id = F32_RIGHT_PRESSURE_VALUE; tx.v.f32 = gSensorData.pressR; xQueueSend(gTxQueue, &tx, 0);
            tx.frame_id = F32_LEFT_TEMP_VALUE;      tx.v.f32 = gSensorData.tempL;  xQueueSend(gTxQueue, &tx, 0);
            tx.frame_id = F32_RIGHT_TEMP_VALUE;     tx.v.f32 = gSensorData.tempR;  xQueueSend(gTxQueue, &tx, 0);
            tx.type = TX_DATA_UINT8;
            tx.frame_id = U8_HEARTBEAT_ACK;         tx.v.u8 = 1;                   xQueueSend(gTxQueue, &tx, 0);
            SendHeaterStatusFrames();
        }
    }
}

