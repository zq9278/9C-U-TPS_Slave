#ifndef TREATMENT_APP_HPP
#define TREATMENT_APP_HPP

#include <stdint.h>
#include "FreeRTOS.h"
#include "task.h"
#include "system_app.h"
#include "AppServices.hpp"

class TreatmentApp {
public:
    void init();
    void handleEvent(const app_event_t &event);
    void handleCommand(const app_cmd_t &cmd);
    void handleSafetyFault(uint8_t fault);
    void service();

private:
    static constexpr float kTempControlCompSubC = 2.0f;

    uint8_t storageSlotForMode(uint8_t mode) const;
    void fillControlConfig(control_config_t &cfg, uint8_t running) const;
    void postControl(ctrl_cmd_id_t id, uint8_t running);
    void updateControlState();
    void loadPressureTargetForMode(uint8_t mode);
    uint32_t curveCycleDurationMs() const;
    void armSessionTimer();
    void freezeSessionTimer();
    void resumeSessionTimer();
    void scheduleSave();
    void serviceDeferredSave();
    void serviceSessionTimer();

    ControlService control_;
    StorageService storage_;
    HostNotifyService host_;

    uint8_t currentMode_ = 1;
    uint8_t controlActive_ = 0;
    uint8_t runRequest_ = 0;
    uint8_t pauseRequest_ = 0;
    uint8_t controlPaused_ = 0;
    uint8_t leftEnable_ = 0;
    uint8_t rightEnable_ = 0;
    float targetTempC_ = 38.0f;
    float currentPressureTargetKpa_ = 25.0f;
    uint16_t treatmentMinutes_ = 5;
    TickType_t sessionEndTick_ = 0;
    uint8_t sessionTimerActive_ = 0;
    TickType_t sessionRemainingTicks_ = 0;
    TickType_t saveDueTick_ = 0;
};

#endif // TREATMENT_APP_HPP
