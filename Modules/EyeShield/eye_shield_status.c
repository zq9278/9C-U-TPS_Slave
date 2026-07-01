#include "eye_shield_status.h"

#include <string.h>
#include "adc.h"
#include "BSP/Gpio/bsp_gpio.h"
#include "FreeRTOS.h"
#define MODULE_LOG_ENABLED MODULE_LOG_EYE_SHIELD_ENABLE
#include "Modules/Heat/treatment_heating_control.h"
#include "Modules/Log/module_log.h"
#include "Modules/communication/Protocol/protocol_ids.h"
#include "main.h"
#include "queue.h"
#include "task.h"

/* 保险丝击穿脉冲宽度与左右侧 bit mask。 */
#define EYE_SHIELD_MASK_LEFT          0x01U
#define EYE_SHIELD_MASK_RIGHT         0x02U
#define EYE_SHIELD_ADC_FULL_SCALE     4095U
#define EYE_SHIELD_ADC_FAULT_MARGIN   8U
#define EYE_SHIELD_ADC_FAULT_TH       (EYE_SHIELD_ADC_FULL_SCALE - EYE_SHIELD_ADC_FAULT_MARGIN)

static BspGpioPin s_left_present_pin;
static BspGpioPin s_right_present_pin;
static BspGpioPin s_left_fuse_detect_pin;
static BspGpioPin s_right_fuse_detect_pin;
static BspGpioPin s_left_fuse_blow_pin;
static BspGpioPin s_right_fuse_blow_pin;

static uint8_t s_initialized = 0U;
static uint8_t s_left_present_stable = 1U;
static uint8_t s_right_present_stable = 1U;
static uint8_t s_left_fuse_stable = 1U;
static uint8_t s_right_fuse_stable = 1U;
static uint8_t s_left_present_count = 0U;
static uint8_t s_right_present_count = 0U;
static uint8_t s_left_fuse_count = 0U;
static uint8_t s_right_fuse_count = 0U;
static uint8_t s_fuse_blow_pending_mask = 0U;
static uint8_t s_fuse_blow_active_mask = 0U;
static TickType_t s_fuse_blow_deadline_tick = 0U;

/* 按统一配置初始化一个 GPIO 状态引脚。 */
static void EyeShieldStatus_InitPin(BspGpioPin *pin,
                                    GPIO_TypeDef *port,
                                    uint16_t gpio_pin,
                                    GPIO_PinState active_state)
{
    BspGpioPinConfig cfg;

    cfg.port = port;
    cfg.pin = gpio_pin;
    cfg.active_state = active_state;
    BspGpio_Init(pin, &cfg);
}

/* 按 mask 同时控制左右保险丝击穿引脚。 */
static void EyeShieldStatus_ApplyFuseMask(uint8_t mask, GPIO_PinState state)
{
    if ((mask & EYE_SHIELD_MASK_LEFT) != 0U)
    {
        BspGpio_Write(&s_left_fuse_blow_pin, state);
    }

    if ((mask & EYE_SHIELD_MASK_RIGHT) != 0U)
    {
        BspGpio_Write(&s_right_fuse_blow_pin, state);
    }
}

/* 对在位/保险丝输入做简单计数去抖。 */
static void EyeShieldStatus_Debounce(uint8_t raw_value,
                                     uint8_t *stable_value,
                                     uint8_t *count,
                                     uint8_t threshold)
{
    if (raw_value != *stable_value)
    {
        ++(*count);
        if (*count >= threshold)
        {
            *stable_value = raw_value;
            *count = 0U;
        }
    }
    else
    {
        *count = 0U;
    }
}

/* 把状态值压入统一发送队列，交由 CommTxTask 上报。 */
static void EyeShieldStatus_EnqueueU8(uint16_t frame_id, uint8_t value)
{
    tx_frame_t tx;

    if (gTxQueue == NULL)
    {
        return;
    }

    (void)memset(&tx, 0, sizeof(tx));
    tx.frame_id = frame_id;
    tx.type = TX_DATA_UINT8;
    tx.v.u8 = value;
    (void)xQueueSend(gTxQueue, &tx, 0U);
}

/* 关闭某一侧治疗：清除使能并切断加热。 */
static void EyeShieldStatus_DisableSide(TreatmentSide side, uint8_t *enable)
{
    if (enable != NULL)
    {
        *enable = 0U;
    }

    TreatmentHeatingControl_DisableSide(side);
}

static uint8_t EyeShieldStatus_ReadAdc(uint32_t channel, uint16_t *value)
{
    ADC_ChannelConfTypeDef cfg;

    if (value == NULL)
    {
        return 0U;
    }

    (void)memset(&cfg, 0, sizeof(cfg));
    cfg.Channel = channel;
    cfg.Rank = ADC_REGULAR_RANK_1;
    cfg.SamplingTime = ADC_SAMPLINGTIME_COMMON_1;
    if (HAL_ADC_ConfigChannel(&hadc1, &cfg) != HAL_OK)
    {
        return 0U;
    }
    if (HAL_ADC_Start(&hadc1) != HAL_OK)
    {
        return 0U;
    }
    if (HAL_ADC_PollForConversion(&hadc1, 10U) != HAL_OK)
    {
        (void)HAL_ADC_Stop(&hadc1);
        return 0U;
    }

    *value = (uint16_t)HAL_ADC_GetValue(&hadc1);
    (void)HAL_ADC_Stop(&hadc1);
    return 1U;
}

void EyeShieldStatus_Init(void)
{
    if (s_initialized != 0U)
    {
        return;
    }

    /* 保持现有板级映射：左眼走 Heat2，右眼走 Heat1。 */
    EyeShieldStatus_InitPin(&s_left_present_pin,
                            MCU_Heat2_Sense_GPIO_Port,
                            MCU_Heat2_Sense_Pin,
                            GPIO_PIN_SET);
    EyeShieldStatus_InitPin(&s_right_present_pin,
                            MCU_Heat1_Sense_GPIO_Port,
                            MCU_Heat1_Sense_Pin,
                            GPIO_PIN_SET);
    EyeShieldStatus_InitPin(&s_left_fuse_detect_pin,
                            Heat2_Fuse_Detection_GPIO_Port,
                            Heat2_Fuse_Detection_Pin,
                            GPIO_PIN_SET);
    EyeShieldStatus_InitPin(&s_right_fuse_detect_pin,
                            Heat1_Fuse_Detection_GPIO_Port,
                            Heat1_Fuse_Detection_Pin,
                            GPIO_PIN_SET);
    EyeShieldStatus_InitPin(&s_left_fuse_blow_pin,
                            Heat2_Fuse_Blown_GPIO_Port,
                            Heat2_Fuse_Blown_Pin,
                            GPIO_PIN_SET);
    EyeShieldStatus_InitPin(&s_right_fuse_blow_pin,
                            Heat1_Fuse_Blown_GPIO_Port,
                            Heat1_Fuse_Blown_Pin,
                            GPIO_PIN_SET);

    s_left_present_stable = 1U;
    s_right_present_stable = 1U;
    s_left_fuse_stable = 1U;
    s_right_fuse_stable = 1U;
    s_left_present_count = 0U;
    s_right_present_count = 0U;
    s_left_fuse_count = 0U;
    s_right_fuse_count = 0U;
    s_fuse_blow_pending_mask = 0U;
    s_fuse_blow_active_mask = 0U;
    s_fuse_blow_deadline_tick = 0U;

    EyeShieldStatus_ApplyFuseMask(EYE_SHIELD_MASK_LEFT | EYE_SHIELD_MASK_RIGHT, GPIO_PIN_RESET);
    s_initialized = 1U;
}

void EyeShieldStatus_RequestFuseBlow(uint8_t blow_left, uint8_t blow_right)
{
    if (s_initialized == 0U)
    {
        EyeShieldStatus_Init();
    }

    if (blow_left != 0U)
    {
        s_fuse_blow_pending_mask |= EYE_SHIELD_MASK_LEFT;
    }

    if (blow_right != 0U)
    {
        s_fuse_blow_pending_mask |= EYE_SHIELD_MASK_RIGHT;
    }

    LOG_I("eye shield fuse blow request L=%u R=%u mask=0x%02X",
          blow_left,
          blow_right,
          s_fuse_blow_pending_mask);
}

void EyeShieldStatus_Service(void)
{
    TickType_t now;

    if (s_initialized == 0U)
    {
        EyeShieldStatus_Init();
    }

    now = xTaskGetTickCount();

    if (s_fuse_blow_pending_mask != 0U)
    {
        /* 收到请求后拉高对应击穿引脚，并在超时后自动拉低。 */
        s_fuse_blow_active_mask |= s_fuse_blow_pending_mask;
        EyeShieldStatus_ApplyFuseMask(s_fuse_blow_pending_mask, GPIO_PIN_SET);
        LOG_I("eye shield fuse pulse start mask=0x%02X", s_fuse_blow_pending_mask);
        s_fuse_blow_pending_mask = 0U;
        s_fuse_blow_deadline_tick = now + pdMS_TO_TICKS(EYE_SHIELD_FUSE_BLOW_PULSE_MS);
    }

    if ((s_fuse_blow_active_mask != 0U) &&
        ((int32_t)(now - s_fuse_blow_deadline_tick) >= 0))
    {
        EyeShieldStatus_ApplyFuseMask(s_fuse_blow_active_mask, GPIO_PIN_RESET);
        LOG_I("eye shield fuse pulse done mask=0x%02X", s_fuse_blow_active_mask);
        s_fuse_blow_active_mask = 0U;
    }
}

uint8_t EyeShieldStatus_Process(control_config_t *cfg, stop_reason_t *stop_reason)
{
    uint8_t left_present_raw;
    uint8_t right_present_raw;
    uint8_t left_fuse_raw;
    uint8_t right_fuse_raw;
    uint8_t stop_requested = 0U;
    uint8_t left_required = 0U;
    uint8_t right_required = 0U;
    uint16_t left_protector_adc = 0U;
    uint16_t right_protector_adc = 0U;
    uint8_t left_adc_valid = 0U;
    uint8_t right_adc_valid = 0U;

    if (s_initialized == 0U)
    {
        EyeShieldStatus_Init();
    }

    if (stop_reason != NULL)
    {
        *stop_reason = STOP_REASON_NONE;
    }

    /* 采集原始 GPIO 状态，再通过去抖更新稳定态。 */
    left_present_raw = BspGpio_ReadActive(&s_left_present_pin);
    right_present_raw = BspGpio_ReadActive(&s_right_present_pin);
    left_fuse_raw = BspGpio_ReadActive(&s_left_fuse_detect_pin);
    right_fuse_raw = BspGpio_ReadActive(&s_right_fuse_detect_pin);

    EyeShieldStatus_Debounce(left_present_raw,
                             &s_left_present_stable,
                             &s_left_present_count,
                             EYE_SHIELD_PRESENT_DEBOUNCE_COUNT);
    EyeShieldStatus_Debounce(right_present_raw,
                             &s_right_present_stable,
                             &s_right_present_count,
                             EYE_SHIELD_PRESENT_DEBOUNCE_COUNT);
    EyeShieldStatus_Debounce(left_fuse_raw,
                             &s_left_fuse_stable,
                             &s_left_fuse_count,
                             EYE_SHIELD_FUSE_DEBOUNCE_COUNT);
    EyeShieldStatus_Debounce(right_fuse_raw,
                             &s_right_fuse_stable,
                             &s_right_fuse_count,
                             EYE_SHIELD_FUSE_DEBOUNCE_COUNT);

    {
        static uint8_t last_left_present = 0xFFU;
        static uint8_t last_right_present = 0xFFU;
        static uint8_t last_left_fuse = 0xFFU;
        static uint8_t last_right_fuse = 0xFFU;

        if ((last_left_present != s_left_present_stable) ||
            (last_right_present != s_right_present_stable) ||
            (last_left_fuse != s_left_fuse_stable) ||
            (last_right_fuse != s_right_fuse_stable))
        {
            last_left_present = s_left_present_stable;
            last_right_present = s_right_present_stable;
            last_left_fuse = s_left_fuse_stable;
            last_right_fuse = s_right_fuse_stable;
            LOG_I("eye shield L_pre=%u R_pre=%u L_fuse=%u R_fuse=%u raw=%u%u%u%u",
                  s_left_present_stable,
                  s_right_present_stable,
                  s_left_fuse_stable,
                  s_right_fuse_stable,
                  left_present_raw,
                  right_present_raw,
                  left_fuse_raw,
                  right_fuse_raw);
        }
    }

    /* 同步更新共享传感器快照，供其他任务直接使用。 */
    gSensorData.heaterPresentL = s_left_present_stable;
    gSensorData.heaterPresentR = s_right_present_stable;
    gSensorData.heaterFuseL = s_left_fuse_stable;
    gSensorData.heaterFuseR = s_right_fuse_stable;

    EyeShieldStatus_EnqueueU8(PROTOCOL_ID_U8_LEFT_HEATER_PRESENT, s_left_present_stable);
    EyeShieldStatus_EnqueueU8(PROTOCOL_ID_U8_RIGHT_HEATER_PRESENT, s_right_present_stable);
    EyeShieldStatus_EnqueueU8(PROTOCOL_ID_U8_LEFT_HEATER_FUSE, s_left_fuse_stable);
    EyeShieldStatus_EnqueueU8(PROTOCOL_ID_U8_RIGHT_HEATER_FUSE, s_right_fuse_stable);

    if (cfg == NULL)
    {
        return 0U;
    }

    /* 仅在治疗中且该侧启用时，眼罩缺失才升级为停机条件。 */
    left_required = (uint8_t)((cfg->running != 0U) && (cfg->press_enable_L != 0U));
    right_required = (uint8_t)((cfg->running != 0U) && (cfg->press_enable_R != 0U));

    {
        static uint8_t left_missing_warned = 0U;

        if (s_left_present_stable != 0U)
        {
            left_missing_warned = 0U;
        }
        else
        {
            if (left_missing_warned == 0U)
            {
                left_missing_warned = 1U;
                LOG_W("eye shield L not present, disable L heat");
            }
            if (left_required != 0U)
            {
                stop_requested = 1U;
                if (stop_reason != NULL)
                {
                    *stop_reason = STOP_REASON_EYE_SHIELD_OFFLINE;
                }
            }
            EyeShieldStatus_DisableSide(TREATMENT_SIDE_LEFT, &cfg->press_enable_L);
        }
    }

    {
        static uint8_t right_missing_warned = 0U;

        if (s_right_present_stable != 0U)
        {
            right_missing_warned = 0U;
        }
        else
        {
            if (right_missing_warned == 0U)
            {
                right_missing_warned = 1U;
                LOG_W("eye shield R not present, disable R heat");
            }
            if (right_required != 0U)
            {
                stop_requested = 1U;
                if (stop_reason != NULL)
                {
                    *stop_reason = STOP_REASON_EYE_SHIELD_OFFLINE;
                }
            }
            EyeShieldStatus_DisableSide(TREATMENT_SIDE_RIGHT, &cfg->press_enable_R);
        }
    }

#if EYE_SHIELD_IGNORE_FUSE_PROTECTION_DEBUG
    {
        static uint8_t fuse_warned = 0U;
        if ((fuse_warned == 0U) &&
            ((s_left_fuse_stable == 0U) || (s_right_fuse_stable == 0U)))
        {
            fuse_warned = 1U;
            LOG_W("DEBUG: eye shield fuse ignored L=%u R=%u",
                  s_left_fuse_stable,
                  s_right_fuse_stable);
        }
    }
#else
    if (s_left_fuse_stable == 0U)
    {
        EyeShieldStatus_DisableSide(TREATMENT_SIDE_LEFT, &cfg->press_enable_L);
    }

    if (s_right_fuse_stable == 0U)
    {
        EyeShieldStatus_DisableSide(TREATMENT_SIDE_RIGHT, &cfg->press_enable_R);
    }
#endif

    /*
     * 保护器件 ADC 故障只在“确认眼盾在线”时才允许触发：
     * - 稳定态在线：避免去抖期间误报；
     * - 原始态也在线：避免拔出瞬间 ADC 飘到满量程先报保护器件故障。
     *
     * 同时，若前面已经因为眼盾离线请求停机，则不再允许 5/6 覆盖离线原因。
     */
    if ((stop_requested == 0U) &&
        (left_required != 0U) &&
        (s_left_present_stable != 0U) &&
        (left_present_raw != 0U))
    {
        left_adc_valid = EyeShieldStatus_ReadAdc(ADC_CHANNEL_8, &left_protector_adc);
        if ((left_adc_valid != 0U) && (left_protector_adc >= EYE_SHIELD_ADC_FAULT_TH))
        {
            LOG_E("eye shield L protector adc fault value=%u", left_protector_adc);
            stop_requested = 1U;
            if (stop_reason != NULL)
            {
                *stop_reason = STOP_REASON_LEFT_HEATER_PROTECTOR_FAULT;
            }
        }
    }

    if ((stop_requested == 0U) &&
        (right_required != 0U) &&
        (s_right_present_stable != 0U) &&
        (right_present_raw != 0U))
    {
        right_adc_valid = EyeShieldStatus_ReadAdc(ADC_CHANNEL_5, &right_protector_adc);
        if ((right_adc_valid != 0U) && (right_protector_adc >= EYE_SHIELD_ADC_FAULT_TH))
        {
            LOG_E("eye shield R protector adc fault value=%u", right_protector_adc);
            stop_requested = 1U;
            if (stop_reason != NULL)
            {
                *stop_reason = STOP_REASON_RIGHT_HEATER_PROTECTOR_FAULT;
            }
        }
    }

    return stop_requested;
}
