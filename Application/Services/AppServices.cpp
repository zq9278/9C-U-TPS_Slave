#include "AppServices.hpp"
#include "Uart_Communicate.h"

bool ControlService::post(ctrl_cmd_id_t id, const control_config_t &cfg) const
{
    ctrl_cmd_t cmd = {};
    cmd.id = id;
    cmd.cfg = cfg;
    return xQueueSend(gCtrlCmdQueue, &cmd, 0) == pdPASS;
}

bool StorageService::requestSave() const
{
    storage_cmd_t cmd = STORAGE_CMD_SAVE_PARAM;
    return xQueueSend(gStorageQueue, &cmd, 0) == pdPASS;
}

bool HostNotifyService::sendTreatmentStopped() const
{
    return sendU8(U8_STOP_TREATMENT, 1);
}

bool HostNotifyService::sendU8(uint16_t frameId, uint8_t value) const
{
    tx_frame_t tx = {};
    tx.type = TX_DATA_UINT8;
    tx.frame_id = frameId;
    tx.v.u8 = value;
    return xQueueSend(gTxQueue, &tx, 0) == pdPASS;
}
