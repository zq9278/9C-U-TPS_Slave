#include "HeaterShieldStatus/heater_shield_status.h"

#include "Uart_Communicate.h"
#include "apply.h"
#include "heat.h"
#include "LOG.h"
#include "main.h"
#include "task.h"

/*
 * 模块内部保存消抖后的稳定状态。
 * 上电默认认为两侧都在位、都未熔断，后续由 GPIO 采样结果逐步修正。
 */
static uint8_t s_right_present_stable = 1;
static uint8_t s_left_present_stable = 1;
static uint8_t s_right_fuse_stable = 1;
static uint8_t s_left_fuse_stable = 1;

/* 消抖计数器：只有连续多次观察到变化，才真正更新稳定状态。 */
static uint8_t s_right_present_count = 0;
static uint8_t s_left_present_count = 0;
static uint8_t s_right_fuse_count = 0;
static uint8_t s_left_fuse_count = 0;

/* 熔断脉冲宽度：保持 10ms 高电平后自动拉低。 */
#define HEATER_SHIELD_FUSE_BLOW_PULSE_MS 10U

/* 异步熔断请求状态：
 * - pending_mask：等待启动的熔断脉冲
 * - active_mask：当前已经拉高，正在计时的熔断脉冲
 * - deadline_tick：本次脉冲何时结束
 */
static uint8_t s_fuse_blow_pending_mask = 0;
static uint8_t s_fuse_blow_active_mask = 0;
static TickType_t s_fuse_blow_deadline_tick = 0;

static void heater_shield_apply_fuse_mask(uint8_t mask, GPIO_PinState state)
{
    if ((mask & 0x01U) != 0U) {
        HAL_GPIO_WritePin(Heat1_Fuse_Blown_GPIO_Port, Heat1_Fuse_Blown_Pin, state);
    }
    if ((mask & 0x02U) != 0U) {
        HAL_GPIO_WritePin(Heat2_Fuse_Blown_GPIO_Port, Heat2_Fuse_Blown_Pin, state);
    }
}

static void heater_shield_debounce_u8(uint8_t raw_value,
                                      uint8_t *stable_value,
                                      uint8_t *count)
{
    if (raw_value != *stable_value) {
        if (++(*count) >= HEATER_SHIELD_STATUS_DEBOUNCE_COUNT) {
            *stable_value = raw_value;
            *count = 0;
        }
    } else {
        *count = 0;
    }
}

static void heater_shield_send_u8(uint16_t frame_id, uint8_t value)
{
    tx_frame_t tx = {0};

    tx.type = TX_DATA_UINT8;
    tx.frame_id = frame_id;
    tx.v.u8 = value;
    xQueueSend(gTxQueue, &tx, 0);
}

void HeaterShieldStatus_Init(void)
{
    s_right_present_stable = 1;
    s_left_present_stable = 1;
    s_right_fuse_stable = 1;
    s_left_fuse_stable = 1;

    s_right_present_count = 0;
    s_left_present_count = 0;
    s_right_fuse_count = 0;
    s_left_fuse_count = 0;

    s_fuse_blow_pending_mask = 0;
    s_fuse_blow_active_mask = 0;
    s_fuse_blow_deadline_tick = 0;

    /* 上电时先确保两路熔断控制脚保持低电平。 */
    heater_shield_apply_fuse_mask(0x03U, GPIO_PIN_RESET);
}

/*
 * 串口线程只负责登记“要不要打熔断脉冲”，这里不直接阻塞等待。
 * 实际的 10ms 高电平由 HeaterShieldStatus_Service() 在控制循环中异步完成。
 */
void HeaterShieldStatus_RequestFuseBlow(uint8_t blow_left, uint8_t blow_right)
{
    if (blow_left) {
        s_fuse_blow_pending_mask |= 0x01U;
    }
    if (blow_right) {
        s_fuse_blow_pending_mask |= 0x02U;
    }
}

/*
 * 熔断脉冲异步服务函数：
 * - 有待处理请求时，把对应控制脚拉高，并记录截止时间
 * - 到时后自动拉低
 *
 * 这样可以完全替代 Uart_Communicate.c 里的 vTaskDelay(10)，避免串口解析任务被阻塞。
 */
void HeaterShieldStatus_Service(void)
{
    TickType_t now = xTaskGetTickCount();

    /*
     * 如果本轮有新的请求，且当前没有活动脉冲，就立即启动一轮 10ms 高电平。
     * 如果脉冲还没结束，又来了新请求，则把新请求并到 active 里，并重新开始计时，
     * 让两路都能获得完整的 10ms 脉冲宽度。
     */
    if (s_fuse_blow_pending_mask != 0U) {
        s_fuse_blow_active_mask |= s_fuse_blow_pending_mask;
        heater_shield_apply_fuse_mask(s_fuse_blow_pending_mask, GPIO_PIN_SET);
        s_fuse_blow_pending_mask = 0;
        s_fuse_blow_deadline_tick = now + pdMS_TO_TICKS(HEATER_SHIELD_FUSE_BLOW_PULSE_MS);
    }

    /* 活动脉冲到期后自动拉低，整个过程对调用者完全异步。 */
    if ((s_fuse_blow_active_mask != 0U) &&
        ((int32_t)(now - s_fuse_blow_deadline_tick) >= 0)) {
        heater_shield_apply_fuse_mask(s_fuse_blow_active_mask, GPIO_PIN_RESET);
        s_fuse_blow_active_mask = 0;
    }
}

/*
 * 采集加热盾在位/熔断状态，并把稳定后的结果同步到：
 * - gSensorData：便于 Live Watch / 调试观察
 * - 上位机状态帧：便于界面显示
 * - gCfg：在不在位/熔断时直接切断对应侧的控制使能
 */
void HeaterShieldStatus_Process(control_config_t *cfg)
{
    uint8_t right_present_raw =
        (HAL_GPIO_ReadPin(MCU_Heat1_Sense_GPIO_Port, MCU_Heat1_Sense_Pin) == GPIO_PIN_RESET) ? 0 : 1;
    uint8_t left_present_raw =
        (HAL_GPIO_ReadPin(MCU_Heat2_Sense_GPIO_Port, MCU_Heat2_Sense_Pin) == GPIO_PIN_RESET) ? 0 : 1;
    uint8_t right_fuse_raw =
        (HAL_GPIO_ReadPin(Heat1_Fuse_Detection_GPIO_Port, Heat1_Fuse_Detection_Pin) == GPIO_PIN_RESET) ? 0 : 1;
    uint8_t left_fuse_raw =
        (HAL_GPIO_ReadPin(Heat2_Fuse_Detection_GPIO_Port, Heat2_Fuse_Detection_Pin) == GPIO_PIN_RESET) ? 0 : 1;

    heater_shield_debounce_u8(right_present_raw, &s_right_present_stable, &s_right_present_count);
    heater_shield_debounce_u8(left_present_raw, &s_left_present_stable, &s_left_present_count);
    heater_shield_debounce_u8(right_fuse_raw, &s_right_fuse_stable, &s_right_fuse_count);
    heater_shield_debounce_u8(left_fuse_raw, &s_left_fuse_stable, &s_left_fuse_count);

    /*
     * 同步到全局传感器聚合结构，便于调试器直接观察。
     * 这些字段本身不参与闭环计算，只是状态可视化。
     */
    gSensorData.heaterPresentL = s_left_present_stable;
    gSensorData.heaterPresentR = s_right_present_stable;
    gSensorData.heaterFuseL = s_left_fuse_stable;
    gSensorData.heaterFuseR = s_right_fuse_stable;

    /* 将稳定后的状态持续上报给上位机。 */
    heater_shield_send_u8(U8_LEFT_HEATER_PRESENT, s_left_present_stable);
    heater_shield_send_u8(U8_RIGHT_HEATER_PRESENT, s_right_present_stable);
    heater_shield_send_u8(U8_LEFT_HEATER_FUSE, s_left_fuse_stable);
    heater_shield_send_u8(U8_RIGHT_HEATER_FUSE, s_right_fuse_stable);

    /*
     * 不在位保护：
     * 一旦某侧不在位，立即禁用该侧压力和加热输出。
     */
    if (!s_left_present_stable) {
        cfg->press_enable_L = 0;
        HeatPWMSet(Left, 0);
        HeatPower(Left, 0);
        VALVE_LEFT(0);
    }

    if (!s_right_present_stable) {
        cfg->press_enable_R = 0;
        HeatPWMSet(Right, 0);
        HeatPower(Right, 0);
        VALVE_RIGHT(0);
    }

#if HEATER_SHIELD_IGNORE_FUSE_PROTECTION_DEBUG
    /*
     * 联调模式：
     * 熔断状态仍然会上报，但不自动关停输出，避免调试阶段被保护逻辑打断。
     */
    static uint8_t fuse_warned = 0;
    if (!fuse_warned && (!s_left_fuse_stable || !s_right_fuse_stable)) {
        fuse_warned = 1;
        LOG_W("DEBUG: ignore fuse, heaters stay enabled (L_fuse=%u R_fuse=%u)",
              s_left_fuse_stable,
              s_right_fuse_stable);
    }
#else
    /*
     * 正常保护模式：
     * 熔断后与“不在位”一样，直接关闭对应侧控制。
     */
    if (!s_left_fuse_stable) {
        cfg->press_enable_L = 0;
        HeatPWMSet(Left, 0);
        HeatPower(Left, 0);
        VALVE_LEFT(0);
    }

    if (!s_right_fuse_stable) {
        cfg->press_enable_R = 0;
        HeatPWMSet(Right, 0);
        HeatPower(Right, 0);
        VALVE_RIGHT(0);
    }
#endif
}
