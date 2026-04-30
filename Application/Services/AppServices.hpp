#ifndef APP_SERVICES_HPP
#define APP_SERVICES_HPP

#include <stdint.h>
#include "system_app.h"

class ControlService {
public:
    bool post(ctrl_cmd_id_t id, const control_config_t &cfg) const;
};

class StorageService {
public:
    bool requestSave() const;
};

class HostNotifyService {
public:
    bool sendTreatmentStopped() const;
    bool sendU8(uint16_t frameId, uint8_t value) const;
};

#endif // APP_SERVICES_HPP
