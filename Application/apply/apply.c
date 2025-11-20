// Compatibility/utility layer for legacy apply APIs
// Provides frame helpers, simple water/air control wrappers,
// and (optional) legacy work state scaffolding used by older code.

#include "apply.h"
#include "heat.h"
#include "water.h"
#include "Pressure_sensor.h"
#include "Config/config.h"
#include "LOG.h"

#include "pid.h"
#include "main.h"
#include "tim.h"
#include "usart.h"

// FreeRTOS timers (used by the legacy state machine helpers below)
#include "FreeRTOS.h"
#include "timers.h"

// ---------------- Defaults and legacy globals ----------------

#define DefultTemperature 3400

volatile u8 AirPump1PWM = 127;      // 气泵1 PWM (0-255)
volatile u16 PressureSet = 30000;   // 目标压力 (Pa: 5000-40000)
volatile u16 RightTempSet = DefultTemperature;
volatile u16 LeftTempSet  = DefultTemperature;

volatile u8 WorkMode = 0;           // 运行模式位图（兼容旧版）
volatile u8 WorkModeState = 0;      // 运行子状态（保留）

extern volatile u8 WaterState;      // 来自 water.c
extern u16 left_pressure, right_pressure;

// TIM15 和 UART3 由 CubeMX 生成
extern TIM_HandleTypeDef htim15;
extern UART_HandleTypeDef huart3;

// 为兼容旧版加热 PID 接口，提供本地 PID 实例
PID_TypeDef RightHeat;
PID_TypeDef LeftHeat;

// ---------------- 通讯: Modbus CRC 与帧封装 ----------------

uint16_t ModbusCRC16(const uint8_t *data, uint16_t len)
{
    uint16_t crc = 0xFFFF;
    for (uint16_t pos = 0; pos < len; pos++) {
        crc ^= data[pos];
        for (uint8_t i = 0; i < 8; i++) {
            if (crc & 0x0001) {
                crc >>= 1;
                crc ^= 0xA001;
            } else {
                crc >>= 1;
            }
        }
    }
    return crc;
}

void SendFrame(uint8_t seq, uint16_t response_id, const uint8_t *data, uint8_t data_len)
{
    // 旧版协议: 5A A5 | len | seq | id_hi id_lo | payload... | crc_lo crc_hi
    uint8_t tx_buffer[64];
    uint8_t len = (uint8_t)(1 + 2 + data_len + 2); // seq + id + payload + crc
    uint8_t idx = 0;

    tx_buffer[idx++] = 0x5A;
    tx_buffer[idx++] = 0xA5;
    tx_buffer[idx++] = len;
    tx_buffer[idx++] = seq;
    tx_buffer[idx++] = (uint8_t)((response_id >> 8) & 0xFF);
    tx_buffer[idx++] = (uint8_t)(response_id & 0xFF);
    for (uint8_t i = 0; i < data_len; i++) tx_buffer[idx++] = data[i];

    uint16_t crc = ModbusCRC16(&tx_buffer[3], (uint16_t)(len - 2));
    tx_buffer[idx++] = (uint8_t)(crc & 0xFF);        // CRC low
    tx_buffer[idx++] = (uint8_t)((crc >> 8) & 0xFF); // CRC high

    HAL_UART_Transmit(&huart3, tx_buffer, idx, 100);
}

void SendPressure(u16 RP, u16 LP)
{
    uint8_t data_buffer[4];
    data_buffer[0] = (uint8_t)(RP >> 8);
    data_buffer[1] = (uint8_t)(RP & 0xFF);
    data_buffer[2] = (uint8_t)(LP >> 8);
    data_buffer[3] = (uint8_t)(LP & 0xFF);
    SendFrame(0x80, 0x8001, data_buffer, 4);
}

void SendEyeTemperature(u16 RT, u16 LT)
{
    uint8_t data_buffer[4];
    data_buffer[0] = (uint8_t)(RT >> 8);
    data_buffer[1] = (uint8_t)(RT & 0xFF);
    data_buffer[2] = (uint8_t)(LT >> 8);
    data_buffer[3] = (uint8_t)(LT & 0xFF);
    SendFrame(0x80, 0x8002, data_buffer, 4);
}

void SendWaterState(u8 WaterSta, u8 WaterSen, u16 WaterTemp)
{
    uint8_t data_buffer[4];
    data_buffer[0] = WaterSta;
    data_buffer[1] = WaterSen;
    data_buffer[2] = (uint8_t)(WaterTemp >> 8);
    data_buffer[3] = (uint8_t)(WaterTemp & 0xFF);
    SendFrame(0x80, 0x8003, data_buffer, 4);
}

void SendDrainWaterState(u8 drain_water_state)
{
    uint8_t data_buffer[4] = {0, 0, 0, 0};
    data_buffer[3] = drain_water_state;
    SendFrame(0x80, 0x8103, data_buffer, 4);
}

void SendAddWaterState(u8 add_water_state)
{
    uint8_t data_buffer[4] = {0, 0, 0, 0};
    data_buffer[2] = add_water_state;
    SendFrame(0x80, 0x8103, data_buffer, 4);
}

// ---------------- 旧版工作状态机（可选使用） ----------------

WorkState_t workState = WORK_IDLE;
static TimerHandle_t xHoldTimer = NULL;
static TimerHandle_t xReleaseTimer = NULL;
static uint8_t currentEye = OD;     // 2=右眼,1=左眼,3=双眼（见 apply.h 枚举别名）

static uint32_t holdSeconds = 1000;    // 保压时长(ms)
static uint32_t releaseSeconds = 500;  // 泄气时长(ms)

// Pressure engine runtime counters (assuming 10ms tick via caller)
static uint32_t ramp_elapsed_ms = 0;
static uint32_t pulse_elapsed_ms = 0;
static uint32_t pulse_phase_elapsed_ms = 0;
static uint8_t  pulse_on_phase = 0;   // 0=off, 1=on
static uint8_t  pulse_alt_eye = 0;    // 0=right, 1=left (for alternate mode)

void Work_ResetPhases(void)
{
    ramp_elapsed_ms = 0;
    pulse_elapsed_ms = 0;
    pulse_phase_elapsed_ms = 0;
    pulse_on_phase = 0;
    pulse_alt_eye = 0;
}

static void vReleaseTimerCallback(TimerHandle_t xTimer)
{
    (void)xTimer;
    // 释放结束，准备再次进气
    switch (currentEye) {
        case OD: AirValve1(0); AirValve2(1); break;    // 右眼：关闭右泄气、打开左泄气
        case OS: AirValve2(0); AirValve1(1); break;    // 左眼
        case OU: AirValve1(0); AirValve2(0); break;    // 双眼
        default: break;
    }
    workState = WORK_INFLATE;
}

// 保压结束：若配置有脉动阶段，则进入脉动，否则直接泄气
static void vHoldTimerCallback2(TimerHandle_t xTimer)
{
    (void)xTimer;
    const PressureProfile_t *pf = &g_settings.mode[(g_settings.mode_select <= 1) ? 0 : 1];

    if (pf->t_pulse_total_ms > 0 && (pf->t_pulse_on_ms > 0 || pf->t_pulse_off_ms > 0)) {
        pulse_elapsed_ms = 0;
        pulse_phase_elapsed_ms = 0;
        pulse_on_phase = 1; // 先 ON
        pulse_alt_eye = 0;  // 从右眼开始
        workState = WORK_PULSE;
        return;
    }

    // 直接泄气
    switch (currentEye) {
        case OD: AirValve1(1); break;
        case OS: AirValve2(1); break;
        case OU: AirValve1(1); AirValve2(1); break;
        default: break;
    }
    TIM15->CCR1 = 0;  // 停止气泵
    workState = WORK_DEFLATE;
    xTimerChangePeriod(xReleaseTimer, pdMS_TO_TICKS(releaseSeconds), 0);
    xTimerStart(xReleaseTimer, 0);
}

void Work_Init(void)
{
    if (xHoldTimer == NULL) {
        xHoldTimer = xTimerCreate("HoldTimer",
                                  pdMS_TO_TICKS(holdSeconds),
                                  pdFALSE,
                                  NULL,
                                  vHoldTimerCallback2);
    }
    if (xReleaseTimer == NULL) {
        xReleaseTimer = xTimerCreate("ReleaseTimer",
                                     pdMS_TO_TICKS(releaseSeconds),
                                     pdFALSE,
                                     NULL,
                                     vReleaseTimerCallback);
    }
    workState = WORK_IDLE;
    Work_ResetPhases();
}

void Work_Expression(u8 eye)
{
    currentEye = eye;
    switch (workState)
    {
        // 初始化：先泄气，泵停，然后转入进气
        case WORK_IDLE:
            AirValve1(1);
            AirValve2(1);
            TIM15->CCR1 = 0;
            Work_ResetPhases();
            workState = WORK_INFLATE;
            break;

        // 进气阶段：目标压力采用慢启动斜坡
        case WORK_INFLATE:
        {
            const PressureProfile_t *pf = &g_settings.mode[(g_settings.mode_select <= 1) ? 0 : 1];
            const uint32_t dt_ms = 10; // 由调用方保证10ms调用一次
            uint32_t target_pa = (uint32_t)(pf->target_kpa * 1000.0f);
            if (pf->t_rise_ms > 0 && ramp_elapsed_ms < pf->t_rise_ms) {
                uint32_t ramp_sp = (uint32_t)((uint64_t)target_pa * ramp_elapsed_ms / pf->t_rise_ms);
                PressureSet = (u16)ramp_sp;
                ramp_elapsed_ms += dt_ms;
            } else {
                PressureSet = (u16)target_pa;
            }

            switch (eye)
            {
                case OD: // 右眼
                    if (right_pressure <= Pressure_Deflation) {
                        AirValve1(0);  // 右进气
                        AirValve2(1);  // 左泄气
                        TIM15->CCR1 = AirPump1PWM;
                    }
                    if (right_pressure >= PressureSet) {
                        TIM15->CCR1 = 0;
                        xTimerStart(xHoldTimer, 0);
                        workState = WORK_HOLD;
                    }
                    break;

                case OS: // 左眼
                    if (left_pressure <= Pressure_Deflation) {
                        AirValve2(0);  // 左进气
                        AirValve1(1);  // 右泄气
                        TIM15->CCR1 = AirPump1PWM;
                    }
                    if (left_pressure >= PressureSet) {
                        TIM15->CCR1 = 0;
                        xTimerStart(xHoldTimer, 0);
                        workState = WORK_HOLD;
                    }
                    break;

                case OU: // 双眼
                    if (right_pressure <= Pressure_Deflation && left_pressure <= Pressure_Deflation) {
                        AirValve1(0);
                        AirValve2(0);
                        TIM15->CCR1 = AirPump1PWM;
                    }
                    if (right_pressure >= PressureSet && left_pressure >= PressureSet) {
                        TIM15->CCR1 = 0;
                        xTimerStart(xHoldTimer, 0);
                        workState = WORK_HOLD;
                    }
                    break;

                default:
                    break;
            }
            break;
        }

        case WORK_HOLD:
            // 等待 Hold 定时器回调
            break;

        // 脉动阶段
        case WORK_PULSE:
        {
            const PressureProfile_t *pf = &g_settings.mode[(g_settings.mode_select <= 1) ? 0 : 1];
            const uint32_t dt_ms = 10;

            // 脉动总时长结束 -> 泄气
            if (pulse_elapsed_ms >= pf->t_pulse_total_ms) {
                switch (currentEye) {
                    case OD: AirValve1(1); AirValve2(0); break;
                    case OS: AirValve2(1); AirValve1(0); break;
                    case OU: AirValve1(1); AirValve2(1); break;
                    default: break;
                }
                TIM15->CCR1 = 0;
                workState = WORK_DEFLATE;
                xTimerChangePeriod(xReleaseTimer, pdMS_TO_TICKS(releaseSeconds), 0);
                xTimerStart(xReleaseTimer, 0);
                break;
            }

            // 更新 on/off 相位
            uint32_t phase_target = pulse_on_phase ? pf->t_pulse_on_ms : pf->t_pulse_off_ms;
            if (phase_target == 0) {
                pulse_on_phase = !pulse_on_phase;
                pulse_phase_elapsed_ms = 0;
            } else if (pulse_phase_elapsed_ms >= phase_target) {
                pulse_on_phase = !pulse_on_phase;
                pulse_phase_elapsed_ms = 0;
                if (pf->squeeze_mode == 1 && (eye == OU)) {
                    pulse_alt_eye ^= 1; // 交替左右
                }
            }

            // 根据相位/模式驱动阀与泵
            if (pulse_on_phase) {
                // ON: 充气/维持
                if (pf->squeeze_mode == 1 && (eye == OU)) {
                    if (pulse_alt_eye == 0) { // right on
                        AirValve1(0); AirValve2(1);
                    } else {
                        AirValve2(0); AirValve1(1);
                    }
                    TIM15->CCR1 = AirPump1PWM;
                } else {
                    if (eye == OD)      { AirValve1(0); AirValve2(1); }
                    else if (eye == OS) { AirValve2(0); AirValve1(1); }
                    else                { AirValve1(0); AirValve2(0); }
                    TIM15->CCR1 = AirPump1PWM;
                }
            } else {
                // OFF: 泄气
                if (pf->squeeze_mode == 1 && (eye == OU)) {
                    AirValve1(1); AirValve2(1); TIM15->CCR1 = 0;
                } else {
                    if (eye == OD)      { AirValve1(1); AirValve2(0); }
                    else if (eye == OS) { AirValve2(1); AirValve1(0); }
                    else                { AirValve1(1); AirValve2(1); }
                    TIM15->CCR1 = 0;
                }
            }

            // 推进时间
            pulse_elapsed_ms       += dt_ms;
            pulse_phase_elapsed_ms += dt_ms;
            break;
        }

        case WORK_DEFLATE:
            // 等待 ReleaseTimer 回调
            break;

        default:
            break;
    }
}

void Work_Heat(u8 eye)
{
    // 使用本地 PID 实例并以当前配置温度作为目标
    // 采样值取自系统聚合的 gSensorData（单位°C，转为 x100）
    extern volatile sensor_data_t gSensorData;

    switch (eye) {
        case OD:
            PID_Heat(Right, &RightHeat, (int32_t)(gSensorData.tempR * 100.0f));
            break;
        case OS:
            PID_Heat(Left,  &LeftHeat,  (int32_t)(gSensorData.tempL * 100.0f));
            break;
        case OU:
            PID_Heat(Right, &RightHeat, (int32_t)(gSensorData.tempR * 100.0f));
            PID_Heat(Left,  &LeftHeat,  (int32_t)(gSensorData.tempL * 100.0f));
            break;
        default:
            break;
    }
}

// ---------------- 旧版加/排水流程（基于状态机） ----------------

u8 Work_DrainWater(void)
{
    // 返回: 2=进行中, 0=失败/中止, 1=完成
    static u16 delay_count = 0;
    static u8  DrainWaterState = 0;

    switch (DrainWaterState)
    {
        case 0: // 关闭水泵
            WaterPump1(0);
            DrainWaterState = 1;
            break;

        case 1: // 等待 1s
            if (++delay_count >= 100) {
                delay_count = 0;
                DrainWaterState = 2;
            }
            break;

        case 2: // 打开所有电磁阀
            WaterValve1(1); WaterValve2(1); WaterValve3(1); WaterValve4(1); WaterValve5(1);
            DrainWaterState = 3;
            break;

        case 3: // 等待 1s
            if (++delay_count >= 100) {
                delay_count = 0;
                DrainWaterState = 4;
            }
            break;

        case 4: // 启动水泵
            WaterPump1(1);
            DrainWaterState = 5;
            break;

        case 5: // 等待液位到位或 20s 超时
            if (HAL_GPIO_ReadPin(WS1_GPIO_Port, WS1_Pin) == GPIO_PIN_SET) {
                DrainWaterState = 6;
            }
            if (++delay_count >= 2000) {
                delay_count = 0;
                DrainWaterState = 0;
                WaterPump1(0);
                AirPump3(0);
                WaterValve1(0); WaterValve2(0); WaterValve3(0); WaterValve4(0); WaterValve5(0);
                WaterState &= (u8)~0x02; // 清除排水标志
                WorkMode   &= (u8)~0x10; // 清除排水工作位
                return 0;
            }
            break;

        case 6: // 等待 15s（原注释“10s”，此处按 15s 保留）
            if (++delay_count >= 1500) {
                delay_count = 0;
                DrainWaterState = 7;
            }
            break;

        case 7: // 打开气泵3
            AirPump3(1);
            DrainWaterState = 8;
            break;

        case 8: // 等待 10s
            if (++delay_count >= 1000) {
                delay_count = 0;
                DrainWaterState = 9;
            }
            break;

        case 9: // 关闭气泵3
            AirPump3(0);
            DrainWaterState = 10;
            break;

        case 10: // 等待 10s
            if (++delay_count >= 1000) {
                delay_count = 0;
                DrainWaterState = 11;
            }
            break;

        case 11: // 结束，复位
            delay_count = 0;
            WaterPump1(0);
            WorkMode   &= (u8)~0x10; // 清除排水工作位
            DrainWaterState = 0;
            WaterState &= (u8)~0x02; // 清除排水标志
            WaterValve1(0); WaterValve2(0); WaterValve3(0); WaterValve4(0); WaterValve5(0);
            return 1;

        default:
            delay_count = 0;
            WorkMode   &= (u8)~0x10;
            WaterState &= (u8)~0x02;
            DrainWaterState = 0;
            WaterPump1(0);
            AirPump3(0);
            WaterValve1(0); WaterValve2(0); WaterValve3(0); WaterValve4(0); WaterValve5(0);
            return 0;
    }

    WaterState |= 0x02; // 排水中标志
    return 2;
}

u8 Work_AddWater(void)
{
    // 返回: 2=进行中, 0=失败/中止, 1=完成
    static u16 AddWater_delay_count = 0;
    static u16 WaterSenseCount = 0;
    static u8  AddWaterState = 0;

    switch (AddWaterState)
    {
        case 0: // 预置：关泵，打开补水阀
            WaterPump1(0);
            WaterValve5(1);
            AddWaterState = 1;
            break;

        case 1: // 等待 1s
            if (++AddWater_delay_count >= 100) {
                AddWater_delay_count = 0;
                AddWaterState = 2;
            }
            break;

        case 2: // 等待传感器到位（低电平有效）或超时
            if (HAL_GPIO_ReadPin(WS1_GPIO_Port, WS1_Pin) == GPIO_PIN_RESET) {
                AddWaterState = 3;
                WaterSenseCount = 0;
            }
            if (++WaterSenseCount >= 30000) {
                WaterPump1(0);
                WaterValve5(1);
                AddWater_delay_count = 0;
                WorkMode   &= (u8)~0x20;
                WaterState &= (u8)~0x04;
                WaterSenseCount = 0;
                AddWaterState = 0;
                return 0;
            }
            break;

        case 3: // 开泵关阀，开始补水
            WaterPump1(1);
            WaterValve5(0);
            AddWaterState = 4;
            break;

        case 4: // 等待 10s
            if (++AddWater_delay_count >= 1000) {
                AddWater_delay_count = 0;
                AddWaterState = 5;
            }
            break;

        case 5: // 打开补水阀，并等待到位或超时
            WaterValve5(1);
            if (HAL_GPIO_ReadPin(WS1_GPIO_Port, WS1_Pin) == GPIO_PIN_RESET) {
                AddWaterState = 6;
            }
            if (++WaterSenseCount >= 30000) {
                WaterPump1(0);
                WaterValve5(1);
                AddWater_delay_count = 0;
                WorkMode   &= (u8)~0x20;
                WaterState &= (u8)~0x04;
                WaterSenseCount = 0;
                AddWaterState = 0;
                return 0;
            }
            break;

        case 6: // 再等待 1s
            WaterValve5(0);
            if (++AddWater_delay_count >= 100) {
                AddWater_delay_count = 0;
                AddWaterState = 7;
            }
            break;

        case 7: // 检查到位完成
            if (HAL_GPIO_ReadPin(WS1_GPIO_Port, WS1_Pin) == GPIO_PIN_RESET) {
                WaterPump1(0);
                WaterValve5(0);
                AddWater_delay_count = 0;
                WorkMode   &= (u8)~0x20;
                WaterState &= (u8)~0x04;
                WaterSenseCount = 0;
                AddWaterState = 0;
                return 1;
            } else {
                AddWaterState = 5;
            }
            break;

        default:
            WaterPump1(0);
            WaterValve5(0);
            AddWater_delay_count = 0;
            WorkMode   &= (u8)~0x20;
            WaterState &= (u8)~0x04;
            WaterSenseCount = 0;
            AddWaterState = 0;
            return 0;
    }

    WaterState |= 0x04; // 补水中标志
    return 2;
}

// ---------------- 旧版协议帧调度器 (简单映射) ----------------

void FrameDispatcher(const ProtocolFrame_t *frame)
{
    if (frame == NULL) return;
    uint16_t id = (uint16_t)((frame->frame_id[0] << 8) | frame->frame_id[1]);
    const uint8_t *payload = frame->payload;

    switch (id)
    {
        case 0x0001: // 气泵/压力控制（兼容旧版含启停位）
            if ((payload[0] & 0x03) == 0x00) {
                WorkMode &= (u8)~0x03; // stop
                WorkModeState = 0;
                HAL_TIM_PWM_Stop(&htim15, TIM_CHANNEL_1);
                TIM15->CCR1 = 0;
                AirValve1(0);
                AirValve2(0);
            } else {
                WorkMode |= (payload[0] & 0x03);
                AirPump1PWM = payload[1];
                PressureSet = (u16)((payload[2] << 8) | payload[3]);
                HAL_TIM_PWM_Start(&htim15, TIM_CHANNEL_1);
                TIM15->CCR1 = AirPump1PWM;
            }
            break;

        case 0x0002: // 加热设置
        {
            WorkMode &= (u8)~0x0C;
            WorkMode |= (u8)((payload[0] & 0x03) << 2);

            RightTempSet = (u16)(DefultTemperature + 50 * payload[2]);
            LeftTempSet  = (u16)(DefultTemperature + 50 * payload[3]);

            PID_Init(&RightHeat, 400, 2, 200, 100000, 0, 1999, 0, RightTempSet);
            PID_Init(&LeftHeat,  400, 2, 200, 100000, 0, 1999, 0, LeftTempSet);

            HeatPower(1, (payload[0] & 0x02) >> 1);
            HeatPWMSet(1, 0);
            HeatPower(2, (payload[0] & 0x01));
            HeatPWMSet(2, 0);
            break;
        }

        case 0x0003: // 水路命令
            // payload[0]=加水, payload[1]=排水, payload[2]=循环
            if (payload[0] + payload[1] + payload[2] == 1 && (WorkMode & 0xF0) == 0) {
                if (payload[0] == 0x01) {
                    WorkMode |= 0x20; // 加水工作位
                } else if (payload[1] == 0x01) {
                    WorkMode |= 0x10; // 排水工作位
                } else if (payload[2] == 0x01) {
                    WaterState = StartWaterPump();
                }
            } else if (payload[2] == 0) {
                WaterState = 0;
                StopWaterPump();
            }
            break;

        case 0x0004: // IPL 冷却
            if (payload[0] == 0x00) {
                StopIPLCold();
            } else {
                StartIPLCold(payload[3]);
            }
            break;

        default:
            // 未知帧，忽略
            break;
    }

    if (frame->payload) {
        vPortFree(frame->payload); // 兼容旧版释放策略
    }
}

