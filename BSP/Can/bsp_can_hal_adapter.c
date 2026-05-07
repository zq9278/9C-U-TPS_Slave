#include "bsp_can_hal_adapter.h"

#include <stddef.h>

#ifdef HAL_CAN_MODULE_ENABLED
static void BspCanHal_FrameToHeader(const BspCanFrame *frame, CAN_TxHeaderTypeDef *header)
{
    header->StdId = frame->id & 0x7FFU;
    header->ExtId = frame->id & 0x1FFFFFFFU;
    header->IDE = frame->is_extended_id ? CAN_ID_EXT : CAN_ID_STD;
    header->RTR = frame->is_remote_frame ? CAN_RTR_REMOTE : CAN_RTR_DATA;
    header->DLC = frame->dlc;
    header->TransmitGlobalTime = DISABLE;
}

static void BspCanHal_HeaderToFrame(const CAN_RxHeaderTypeDef *header,
                                    const uint8_t *data,
                                    BspCanFrame *frame)
{
    uint8_t index;

    frame->is_extended_id = (uint8_t)(header->IDE == CAN_ID_EXT);
    frame->is_remote_frame = (uint8_t)(header->RTR == CAN_RTR_REMOTE);
    frame->id = frame->is_extended_id ? header->ExtId : header->StdId;
    frame->dlc = (uint8_t)(header->DLC > 8U ? 8U : header->DLC);

    for (index = 0U; index < frame->dlc; ++index)
    {
        frame->data[index] = data[index];
    }
}
#endif

static uint8_t BspCanHal_StartIt(void *context)
{
#ifdef HAL_CAN_MODULE_ENABLED
    CAN_HandleTypeDef *hcan = (CAN_HandleTypeDef *)context;

    if ((hcan == NULL) || (HAL_CAN_Start(hcan) != HAL_OK))
    {
        return 0U;
    }

    return (uint8_t)(HAL_CAN_ActivateNotification(hcan,
                                                  CAN_IT_RX_FIFO0_MSG_PENDING |
                                                  CAN_IT_RX_FIFO1_MSG_PENDING |
                                                  CAN_IT_TX_MAILBOX_EMPTY |
                                                  CAN_IT_ERROR) == HAL_OK);
#else
    (void)context;
    return 0U;
#endif
}

static uint8_t BspCanHal_StartPolling(void *context)
{
#ifdef HAL_CAN_MODULE_ENABLED
    CAN_HandleTypeDef *hcan = (CAN_HandleTypeDef *)context;
    return (uint8_t)((hcan != NULL) && (HAL_CAN_Start(hcan) == HAL_OK));
#else
    (void)context;
    return 0U;
#endif
}

static uint8_t BspCanHal_Send(void *context,
                              const BspCanFrame *frame,
                              uint32_t timeout_ms)
{
#ifdef HAL_CAN_MODULE_ENABLED
    CAN_HandleTypeDef *hcan = (CAN_HandleTypeDef *)context;
    CAN_TxHeaderTypeDef header;
    uint32_t mailbox;
    uint32_t start_tick;

    if ((hcan == NULL) || (frame == NULL) || (frame->dlc > 8U))
    {
        return 0U;
    }

    start_tick = HAL_GetTick();
    while (HAL_CAN_GetTxMailboxesFreeLevel(hcan) == 0U)
    {
        if ((HAL_GetTick() - start_tick) >= timeout_ms)
        {
            return 0U;
        }
    }

    BspCanHal_FrameToHeader(frame, &header);
    return (uint8_t)(HAL_CAN_AddTxMessage(hcan, &header, (uint8_t *)frame->data, &mailbox) == HAL_OK);
#else
    (void)context; (void)frame; (void)timeout_ms;
    return 0U;
#endif
}

static uint8_t BspCanHal_Recover(void *context)
{
#ifdef HAL_CAN_MODULE_ENABLED
    CAN_HandleTypeDef *hcan = (CAN_HandleTypeDef *)context;

    if (hcan == NULL)
    {
        return 0U;
    }

    (void)HAL_CAN_Stop(hcan);
    return (uint8_t)(HAL_CAN_Start(hcan) == HAL_OK);
#else
    (void)context;
    return 0U;
#endif
}

void BspCanHalAdapter_MakeConfig(BspCanConfig *config,
                                 BspCanHalHandle *hcan,
                                 BspCanHalMode mode,
                                 uint32_t default_timeout_ms)
{
    if (config == NULL)
    {
        return;
    }

    config->ops.start = (mode == BSP_CAN_HAL_MODE_IT) ? BspCanHal_StartIt : BspCanHal_StartPolling;
    config->ops.send = BspCanHal_Send;
    config->ops.recover = BspCanHal_Recover;
    config->low_level_context = hcan;
    config->default_timeout_ms = default_timeout_ms;
    config->auto_recover_on_error = 1U;
}

uint8_t BspCanHalAdapter_ReadRxFifo0(BspCanHalHandle *hcan, BspCanFrame *frame)
{
#ifdef HAL_CAN_MODULE_ENABLED
    CAN_RxHeaderTypeDef header;
    uint8_t data[8];

    if ((hcan == NULL) || (frame == NULL))
    {
        return 0U;
    }

    if (HAL_CAN_GetRxMessage((CAN_HandleTypeDef *)hcan, CAN_RX_FIFO0, &header, data) != HAL_OK)
    {
        return 0U;
    }

    BspCanHal_HeaderToFrame(&header, data, frame);
    return 1U;
#else
    (void)hcan; (void)frame;
    return 0U;
#endif
}

uint8_t BspCanHalAdapter_ReadRxFifo1(BspCanHalHandle *hcan, BspCanFrame *frame)
{
#ifdef HAL_CAN_MODULE_ENABLED
    CAN_RxHeaderTypeDef header;
    uint8_t data[8];

    if ((hcan == NULL) || (frame == NULL))
    {
        return 0U;
    }

    if (HAL_CAN_GetRxMessage((CAN_HandleTypeDef *)hcan, CAN_RX_FIFO1, &header, data) != HAL_OK)
    {
        return 0U;
    }

    BspCanHal_HeaderToFrame(&header, data, frame);
    return 1U;
#else
    (void)hcan; (void)frame;
    return 0U;
#endif
}

uint32_t BspCanHalAdapter_GetError(BspCanHalHandle *hcan)
{
#ifdef HAL_CAN_MODULE_ENABLED
    return (hcan != NULL) ? HAL_CAN_GetError((CAN_HandleTypeDef *)hcan) : 0U;
#else
    (void)hcan;
    return 0U;
#endif
}
