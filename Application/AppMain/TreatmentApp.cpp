#include "TreatmentApp.hpp"

#include "LOG.h"
#include "config.h"
#include "wave_control.h"

/*
 * TreatmentApp
 *
 * 这个类是“治疗业务状态机”，位于协议层和控制层之间。
 *
 * 它不直接解析串口帧，也不直接控制硬件：
 * - 串口帧解析由 HostProtocol / Uart_Communicate.cpp 完成。
 * - 气泵、阀、加热 PID 等闭环控制由 ControlTask 完成。
 *
 * TreatmentApp 负责回答更高层的问题：
 * - 当前是否应该运行？
 * - 左右眼是否使能？
 * - 当前目标温度、目标压力是多少？
 * - 收到暂停命令后，倒计时应该如何冻结？
 * - 收到安全故障后，应该如何停机？
 * - 参数变更后什么时候保存？
 *
 * 外部输入：
 * - APP_EVT_HOST_COMMAND：上位机命令，内部携带 app_cmd_t。
 * - APP_EVT_SAFETY_FAULT：安全任务上报的故障。
 *
 * 外部输出：
 * - ControlService：向 ControlTask 发送 ctrl_cmd_t。
 * - StorageService：请求 StorageTask 保存参数。
 * - HostNotifyService：通过 gTxQueue 通知上位机。
 */

/* 系统参数，全局保存在 config.c 中，由 StorageTask/Config 模块负责加载和保存。 */
extern SystemSettings_t g_settings;

/*
 * 根据模式号选择参数保存槽位。
 *
 * 当前 SystemSettings_t 里只有两个 mode[] 槽位：
 * - 模式 1 使用 mode[0]
 * - 其他模式使用 mode[1]
 *
 * 注意：
 * 这个映射是为了兼容现有参数结构。如果后面真的要支持 4 套独立参数，
 * 需要先扩展 SystemSettings_t::mode[] 的长度，再调整这里。
 */
uint8_t TreatmentApp::storageSlotForMode(uint8_t mode) const
{
    return (mode <= 1U) ? 0U : 1U;
}

/*
 * 把 TreatmentApp 当前业务状态打包成 ControlTask 能理解的 control_config_t。
 *
 * TreatmentApp 内部保存的是“上位机/业务层目标”：
 * - targetTempC_：用户设置的目标温度
 * - currentPressureTargetKpa_：当前模式使用的目标压力
 * - leftEnable_/rightEnable_：左右眼使能
 *
 * ControlTask 只关心控制参数，所以这里会把业务状态转换为控制配置。
 *
 * 温度补偿：
 * 控制层实际目标温度 = 用户目标温度 - kTempControlCompSubC。
 * 这个补偿用于抵消传感器/结构上的温差，避免直接用上位机温度做闭环目标。
 */
void TreatmentApp::fillControlConfig(control_config_t &cfg, uint8_t running) const
{
    float controlTempC = targetTempC_ - kTempControlCompSubC;
    if (controlTempC < 0.0f) {
        controlTempC = 0.0f;
    }

    cfg.mode = currentMode_;
    cfg.running = running;
    cfg.temp_target = controlTempC;
    cfg.press_target_max = currentPressureTargetKpa_;
    cfg.t1_rise_s = WAVE_CONTROL_RISE_S;
    cfg.t2_hold_s = WAVE_CONTROL_HOLD_S;
    cfg.t3_pulse_s = WAVE_CONTROL_PULSE_S;
    cfg.pulse_on_ms = 0.0f;
    cfg.pulse_off_ms = 0.0f;
    cfg.squeeze_mode = 0;
    cfg.press_enable_L = leftEnable_;
    cfg.press_enable_R = rightEnable_;
    cfg.treatment_minutes = (treatmentMinutes_ == 0U) ? 1U : treatmentMinutes_;
}

/*
 * 向 ControlTask 下发控制命令。
 *
 * id：
 * - CTRL_CMD_START：开始控制
 * - CTRL_CMD_STOP：停止控制
 * - CTRL_CMD_PAUSE：暂停输出
 * - CTRL_CMD_RESUME：恢复输出
 * - CTRL_CMD_UPDATE_CFG：运行中更新配置
 *
 * running：
 * 这个字段会写入 cfg.running，告诉 ControlTask 当前业务层认为是否处于运行状态。
 */
void TreatmentApp::postControl(ctrl_cmd_id_t id, uint8_t running)
{
    control_config_t cfg = {};
    fillControlConfig(cfg, running);
    (void)control_.post(id, cfg);
}

/*
 * 单个治疗波形周期时长。
 *
 * 当前固定为 WAVE_CONTROL_CYCLE_MS，即 60 秒。
 * treatmentMinutes_ 这个名字来自原协议字段，但当前逻辑里它表示“60 秒周期个数”。
 */
uint32_t TreatmentApp::curveCycleDurationMs() const
{
    return WAVE_CONTROL_CYCLE_MS;
}

/*
 * 启动或刷新治疗倒计时。
 *
 * totalMs = 单个波形周期时长 * 周期数量。
 *
 * 使用 TickType_t 做时间比较时，后续统一用：
 * static_cast<int32_t>(now - targetTick) >= 0
 * 这样可以兼容 tick 回绕。
 */
void TreatmentApp::armSessionTimer()
{
    const uint32_t minutes = (treatmentMinutes_ == 0U) ? 1U : static_cast<uint32_t>(treatmentMinutes_);
    const TickType_t now = xTaskGetTickCount();
    const uint32_t totalMs = curveCycleDurationMs() * minutes;

    sessionEndTick_ = now + pdMS_TO_TICKS(totalMs);
    sessionTimerActive_ = 1;
}

/*
 * 暂停治疗时冻结倒计时。
 *
 * 暂停不是结束治疗：
 * - 控制输出会被暂停。
 * - 治疗倒计时要保存剩余时间。
 * - 恢复时继续用剩余时间，而不是重新开始整段治疗。
 */
void TreatmentApp::freezeSessionTimer()
{
    const TickType_t now = xTaskGetTickCount();
    if (sessionTimerActive_ && (sessionEndTick_ > now)) {
        sessionRemainingTicks_ = sessionEndTick_ - now;
    } else {
        sessionRemainingTicks_ = 0;
    }
    sessionTimerActive_ = 0;
}

/*
 * 恢复暂停前冻结的倒计时。
 *
 * 如果 sessionRemainingTicks_ 为 0，说明没有可恢复的剩余时间，
 * 此时不启动倒计时。
 */
void TreatmentApp::resumeSessionTimer()
{
    if (sessionRemainingTicks_ > 0) {
        sessionEndTick_ = xTaskGetTickCount() + sessionRemainingTicks_;
        sessionTimerActive_ = 1;
    }
}

/*
 * 选择模式，并加载该模式对应的目标压力。
 *
 * mode 会被限制在 1..4。
 * 但参数保存槽位目前只有两个，实际压力来自 storageSlotForMode() 映射后的槽位。
 *
 * 如果 flash 参数中保存的压力无效（<=0），使用默认 25kPa。
 */
void TreatmentApp::loadPressureTargetForMode(uint8_t mode)
{
    if (mode < 1U) {
        mode = 1U;
    }
    if (mode > 4U) {
        mode = 4U;
    }

    currentMode_ = mode;
    const float stored = g_settings.mode[storageSlotForMode(currentMode_)].target_kpa;
    currentPressureTargetKpa_ = (stored > 0.0f) ? stored : 25.0f;
}

/*
 * TreatmentApp 上电初始化。
 *
 * 这里只初始化业务状态，不创建队列、不创建任务。
 * 队列和任务由 AppMain_FreeRTOS_Init() 创建。
 *
 * 上电默认：
 * - 加载保存的模式和压力。
 * - 温度使用保存的 left_temp_c。
 * - 左右眼默认不使能。
 * - 不自动运行。
 * - App 状态为 IDLE。
 */
void TreatmentApp::init()
{
    const uint8_t bootMode =
        (g_settings.mode_select >= 1U && g_settings.mode_select <= 4U) ? g_settings.mode_select : 1U;

    loadPressureTargetForMode(bootMode);
    targetTempC_ = g_settings.left_temp_c;
    leftEnable_ = 0;
    rightEnable_ = 0;
    runRequest_ = 0;
    pauseRequest_ = 0;
    controlPaused_ = 0;
    controlActive_ = 0;
    sessionTimerActive_ = 0;
    sessionRemainingTicks_ = 0;
    saveDueTick_ = 0;
    gAppState = APP_STATE_IDLE;
}

/*
 * 根据当前业务状态决定 ControlTask 应该处于什么状态。
 *
 * 核心输入状态：
 * - runRequest_：用户是否请求运行。
 * - pauseRequest_：用户是否请求暂停。
 * - leftEnable_/rightEnable_：左右眼是否至少有一路使能。
 * - controlActive_：ControlTask 当前是否已被启动。
 * - controlPaused_：ControlTask 当前是否处于暂停保持状态。
 *
 * shouldRun 的含义：
 * 只有用户请求运行，并且至少一路眼部通道使能，才允许真正运行。
 *
 * 状态转移大致如下：
 *
 * STOPPED --START--> RUNNING
 * RUNNING --PAUSE--> PAUSED
 * PAUSED  --RESUME-> RUNNING
 * RUNNING --STOP---> STOPPED
 * PAUSED  --STOP---> STOPPED
 *
 * 这个函数是 TreatmentApp 的核心，所有会影响运行状态的命令
 * 最后都应该调用它统一做状态决策。
 */
void TreatmentApp::updateControlState()
{
    const uint8_t shouldRun = (runRequest_ && (leftEnable_ || rightEnable_)) ? 1U : 0U;

    if (shouldRun) {
        if (pauseRequest_) {
            /*
             * 进入暂停：
             * - 对外不认为正在治疗运行。
             * - App 状态回到 READY。
             * - 只在第一次进入暂停时给 ControlTask 发 CTRL_CMD_PAUSE。
             */
            gTreatmentRunning = 0;
            gAppState = APP_STATE_READY;
            if (controlActive_ && !controlPaused_) {
                controlPaused_ = 1;
                LOG_W("TreatmentApp PAUSE run=%u L=%u R=%u", runRequest_, leftEnable_, rightEnable_);
                postControl(CTRL_CMD_PAUSE, 1);
            }
            return;
        }

        /*
         * 应该运行且没有暂停请求。
         * 根据当前 ControlTask 状态，决定是 START、RESUME 还是 UPDATE_CFG。
         */
        gTreatmentRunning = 1;
        gAppState = APP_STATE_RUN_MODE1;
        if (!controlActive_) {
            /* 从停止状态进入运行状态。 */
            controlActive_ = 1;
            controlPaused_ = 0;
            LOG_I("TreatmentApp START run=%u L=%u R=%u", runRequest_, leftEnable_, rightEnable_);
            postControl(CTRL_CMD_START, 1);
            if (!sessionTimerActive_) {
                if (sessionRemainingTicks_ > 0) {
                    resumeSessionTimer();
                } else {
                    armSessionTimer();
                }
            }
        } else if (controlPaused_) {
            /* 从暂停状态恢复运行。 */
            controlPaused_ = 0;
            LOG_I("TreatmentApp RESUME run=%u L=%u R=%u", runRequest_, leftEnable_, rightEnable_);
            postControl(CTRL_CMD_RESUME, 1);
            if (!sessionTimerActive_) {
                resumeSessionTimer();
            }
        } else {
            /*
             * 已经在运行中，此时一般是温度、压力、左右使能等参数变化。
             * 不重启控制，只更新配置。
             */
            postControl(CTRL_CMD_UPDATE_CFG, 1);
            if (!sessionTimerActive_) {
                armSessionTimer();
            }
        }
    } else {
        /*
         * 不应该运行：
         * - 用户没有请求运行，或者左右眼全部关闭。
         * - 如果控制层之前处于运行或暂停，发 STOP。
         * - 清除治疗倒计时和暂停剩余时间。
         */
        gTreatmentRunning = 0;
        if (controlActive_ || controlPaused_) {
            controlActive_ = 0;
            controlPaused_ = 0;
            LOG_W("TreatmentApp STOP run=%u L=%u R=%u", runRequest_, leftEnable_, rightEnable_);
            postControl(CTRL_CMD_STOP, 0);
        }
        gAppState = APP_STATE_READY;
        sessionTimerActive_ = 0;
        sessionRemainingTicks_ = 0;
    }
}

/*
 * 延迟保存参数。
 *
 * 温度/压力这类参数可能被上位机连续拖动修改。
 * 如果每次修改都立即写 flash，会造成频繁擦写。
 *
 * 当前策略：
 * - 收到参数修改后，设置 saveDueTick_ = 当前时间 + 3 秒。
 * - 如果 3 秒内又修改参数，保存时间会被刷新。
 * - 到期后由 serviceDeferredSave() 统一请求保存。
 */
void TreatmentApp::scheduleSave()
{
    saveDueTick_ = xTaskGetTickCount() + pdMS_TO_TICKS(3000);
}

/*
 * 应用事件入口。
 *
 * AppTask 只从 gAppEventQueue 收 app_event_t，然后调用这里。
 *
 * 当前事件类型：
 * - APP_EVT_HOST_COMMAND：上位机命令，转给 handleCommand()。
 * - APP_EVT_SAFETY_FAULT：安全故障，转给 handleSafetyFault()。
 *
 * 后续如果增加内部事件，比如按键、传感器触发、PID 调试触发，
 * 建议也从这里分发。
 */
void TreatmentApp::handleEvent(const app_event_t &event)
{
    switch (event.id) {
        case APP_EVT_HOST_COMMAND:
            handleCommand(event.v.host_cmd);
            break;
        case APP_EVT_SAFETY_FAULT:
            handleSafetyFault(event.v.safety_fault);
            break;
        case APP_EVT_NONE:
        default:
            break;
    }
}

/*
 * 处理上位机业务命令。
 *
 * 这里处理的是 app_cmd_t，不是原始串口帧。
 * 原始串口帧已经由 HostProtocol 转换过。
 *
 * 添加新上位机命令时，一般流程是：
 * 1. 在 Uart_Communicate.h 里加 FrameId_t。
 * 2. 在 system_app.h 里加 APP_CMD_xxx。
 * 3. 在 HostProtocol::dispatch() 里把帧转换成 APP_CMD_xxx。
 * 4. 在这里添加 case，更新业务状态或调用 Service。
 */
void TreatmentApp::handleCommand(const app_cmd_t &cmd)
{
    switch (cmd.id) {
        case APP_CMD_MODE_SELECT:
            /* 切换模式，并加载该模式对应的目标压力。 */
            g_settings.mode_select = cmd.v.u8;
            loadPressureTargetForMode(cmd.v.u8);
            updateControlState();
            break;

        case APP_CMD_SET_TEMP:
            /*
             * 设置目标温度。
             * 当前左右眼共用同一个目标温度，所以同时写 left/right。
             */
            targetTempC_ = cmd.v.f32;
            g_settings.left_temp_c = targetTempC_;
            g_settings.right_temp_c = targetTempC_;
            scheduleSave();
            updateControlState();
            break;

        case APP_CMD_SET_PRESSURE_KPA:
            /*
             * 设置当前模式目标压力。
             * 同时更新 g_settings，稍后由延迟保存机制写入 flash。
             */
            currentPressureTargetKpa_ = cmd.v.f32;
            g_settings.mode[storageSlotForMode(currentMode_)].target_kpa = cmd.v.f32;
            scheduleSave();
            updateControlState();
            break;

        case APP_CMD_LEFT_ENABLE:
            /* 左眼通道使能变化。运行中变化时会触发 UPDATE_CFG。 */
            leftEnable_ = cmd.v.u8 ? 1U : 0U;
            updateControlState();
            break;

        case APP_CMD_RIGHT_ENABLE:
            /* 右眼通道使能变化。运行中变化时会触发 UPDATE_CFG。 */
            rightEnable_ = cmd.v.u8 ? 1U : 0U;
            updateControlState();
            break;

        case APP_CMD_SET_TREATMENT_TIME:
            /*
             * 设置治疗时间。
             * 当前字段实际含义是“60 秒波形周期个数”。
             * 如果正在运行，立即刷新倒计时。
             */
            treatmentMinutes_ = (cmd.v.u16 == 0U) ? 1U : cmd.v.u16;
            if (controlActive_) {
                armSessionTimer();
                updateControlState();
            }
            break;

        case APP_CMD_START:
            /*
             * 开始治疗。
             * 如果左右眼都没使能，默认同时打开左右眼，避免用户点开始后没有任何输出。
             */
            if (leftEnable_ == 0U && rightEnable_ == 0U) {
                leftEnable_ = 1;
                rightEnable_ = 1;
            }
            pauseRequest_ = 0;
            runRequest_ = 1;
            updateControlState();
            break;

        case APP_CMD_STOP:
            /* 停止治疗，并清除暂停请求。 */
            pauseRequest_ = 0;
            runRequest_ = 0;
            updateControlState();
            break;

        case APP_CMD_PAUSE_RESUME:
            /*
             * 暂停/继续：
             * - cmd.v.u8 == 0：暂停
             * - cmd.v.u8 != 0：继续
             *
             * 暂停时冻结治疗倒计时；
             * 恢复时由 updateControlState() 调用 resumeSessionTimer()。
             */
            if (cmd.v.u8 == 0U) {
                if (runRequest_ && !pauseRequest_) {
                    pauseRequest_ = 1;
                    freezeSessionTimer();
                    updateControlState();
                }
            } else {
                if (runRequest_ && pauseRequest_) {
                    pauseRequest_ = 0;
                    updateControlState();
                }
            }
            break;

        case APP_CMD_READ_PARAM:
            /* 主机读取当前参数，直接广播保存的设置。 */
            Settings_Broadcast();
            break;

        case APP_CMD_SAVE_PARAM:
            /* 主机主动请求保存参数，立即通知 StorageTask。 */
            (void)storage_.requestSave();
            break;

        case APP_CMD_NONE:
        default:
            break;
    }
}

/*
 * 处理安全故障。
 *
 * 安全故障来自 SafetyTask，并通过 APP_EVT_SAFETY_FAULT 进入这里。
 *
 * 当前策略：
 * - 清除运行请求。
 * - 清除暂停请求。
 * - 关闭左右眼使能。
 * - 调用 updateControlState()，统一向 ControlTask 发 STOP。
 * - App 状态置为 ALARM。
 */
void TreatmentApp::handleSafetyFault(uint8_t fault)
{
    if (fault == 0U) {
        return;
    }

    runRequest_ = 0;
    pauseRequest_ = 0;
    leftEnable_ = 0;
    rightEnable_ = 0;
    LOG_W("TreatmentApp SAFETY fault=%u -> stop", fault);
    updateControlState();
    gAppState = APP_STATE_ALARM;
}

/*
 * 处理延迟保存。
 *
 * AppTask 会周期调用 TreatmentApp::service()，
 * service() 再调用这里检查保存时间是否到期。
 */
void TreatmentApp::serviceDeferredSave()
{
    if (saveDueTick_ != 0 && static_cast<int32_t>(xTaskGetTickCount() - saveDueTick_) >= 0) {
        (void)storage_.requestSave();
        saveDueTick_ = 0;
    }
}

/*
 * 处理治疗倒计时。
 *
 * 只有在以下条件同时满足时才检查结束：
 * - controlActive_：控制层已经启动。
 * - !controlPaused_：当前没有暂停。
 * - sessionTimerActive_：倒计时有效。
 *
 * 到时间后：
 * - 清除运行/暂停状态。
 * - 调用 updateControlState() 发 STOP。
 * - 通知上位机治疗结束。
 */
void TreatmentApp::serviceSessionTimer()
{
    if (controlActive_ && !controlPaused_ && sessionTimerActive_) {
        const TickType_t now = xTaskGetTickCount();
        if (static_cast<int32_t>(now - sessionEndTick_) >= 0) {
            runRequest_ = 0;
            pauseRequest_ = 0;
            sessionTimerActive_ = 0;
            sessionRemainingTicks_ = 0;
            updateControlState();
            (void)host_.sendTreatmentStopped();
        }
    }
}

/*
 * 周期服务函数。
 *
 * AppTask 每轮循环都会调用它。
 * 它用于处理“不来自队列事件，但需要随时间推进”的业务：
 * - 延迟保存
 * - 治疗倒计时结束
 */
void TreatmentApp::service()
{
    serviceDeferredSave();
    serviceSessionTimer();
}
