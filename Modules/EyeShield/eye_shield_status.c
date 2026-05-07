#include "eye_shield_status.h"

#include <string.h>
#include "BSP/Gpio/bsp_gpio.h"
#include "FreeRTOS.h"
#include "Modules/Log/module_log.h"
#include "Modules/communication/Protocol/protocol_ids.h"
#include "TreatmentActuators.h"
#include "main.h"
#include "queue.h"
#include "task.h"

#define EYE_SHIELD_FUSE_BLOW_PULSE_MS 10U
#define EYE_SHIELD_MASK_LEFT          0x01U
#define EYE_SHIELD_MASK_RIGHT         0x02U

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

static void EyeShieldStatus_Debounce(uint8_t raw_value,
                                     uint8_t *stable_value,
                                     uint8_t *count)
{
    if (raw_value != *stable_value)
    {
        ++(*count);
        if (*count >= EYE_SHIELD_STATUS_DEBOUNCE_COUNT)
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

static void EyeShieldStatus_DisableSide(TreatmentSide side, uint8_t *enable)
{
    if (enable != NULL)
    {
        *enable = 0U;
    }

    TreatmentActuators_SetHeaterPwm(side, 0U);
    TreatmentActuators_SetHeaterPower(side, 0U);
}

void EyeShieldStatus_Init(void)
{
    if (s_initialized != 0U)
    {
        return;
    }

    /*
     * Keep the old board mapping:
     * left eye shield is wired to Heat2 pins, right eye shield is wired to Heat1 pins.
     */
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

void EyeShieldStatus_Process(control_config_t *cfg)
{
    uint8_t left_present_raw;
    uint8_t right_present_raw;
    uint8_t left_fuse_raw;
    uint8_t right_fuse_raw;

    if (s_initialized == 0U)
    {
        EyeShieldStatus_Init();
    }

    left_present_raw = BspGpio_ReadActive(&s_left_present_pin);
    right_present_raw = BspGpio_ReadActive(&s_right_present_pin);
    left_fuse_raw = BspGpio_ReadActive(&s_left_fuse_detect_pin);
    right_fuse_raw = BspGpio_ReadActive(&s_right_fuse_detect_pin);

    EyeShieldStatus_Debounce(left_present_raw, &s_left_present_stable, &s_left_present_count);
    EyeShieldStatus_Debounce(right_present_raw, &s_right_present_stable, &s_right_present_count);
    EyeShieldStatus_Debounce(left_fuse_raw, &s_left_fuse_stable, &s_left_fuse_count);
    EyeShieldStatus_Debounce(right_fuse_raw, &s_right_fuse_stable, &s_right_fuse_count);

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
        return;
    }

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
}
