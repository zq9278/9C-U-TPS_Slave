#include "Uart_Communicate.h"
#include <string.h>
#include <stdbool.h>
#include "control_task.h"
#include "WaveControl/wave_control.h"
#include "HeaterShieldStatus/heater_shield_status.h"
#include "ValveControl/valve_control.h"
#include "pid.h"
#include "heat.h"
#include "tim.h"
#include "LOG.h"
#include "main.h"

#include "FreeRTOS.h"
#include "task.h"

/*
 * control_task.c
 *
 * 模块职责：
 * 1) 接收 AppTask 下发的控制命令（启动 / 停止 / 更新配置）
 * 2) 维护压力治疗状态机：预泄压 -> 上升 -> 保压 -> 脉冲 -> 周期间泄压
 * 3) 根据当前状态计算统一目标压力，并驱动气泵 / 阀门执行闭环控制
 * 4) 根据左右通道使能状态执行温控
 * 5) 周期上报压力 / 温度 / 当前阶段 / 加热盾在位与熔断状态
 *
 * 这个任务本质上是“控制中心”：
 * - 压力通道由一个公共气泵 + 左右阀门组成，闭环统一使用 gSensorData.pressL
 * - 温控通道左右独立，但使能状态受在位 / 熔断保护影响
 * - 所有保护动作最终都会收敛到关闭对应输出
 */

/* 调试宏：为了排查温度采样异常，临时强制关闭左右加热输出。
 * 打开后，加热电源和加热 PWM 都会保持为 0，但压力控制和遥测仍照常运行。
 */
#define DISABLE_HEATING_FOR_DEBUG 0

/* 上报到界面的温度补偿值，控制本身仍使用真实传感器温度。 */
#define TEMP_DISPLAY_OFFSET_C 0.0f

/*
 * 遥测发送频率配置。
 * 这里统一用“每秒点数”定义，周期由代码自动换算：
 * period_ms = 1000 / points_per_sec
 */
#define PRESS_TX_POINTS_PER_SEC 100U
#define TEMP_TX_POINTS_PER_SEC  50U

#define PRESS_TX_PERIOD_MS (1000U / PRESS_TX_POINTS_PER_SEC)
#define TEMP_TX_PERIOD_MS  (1000U / TEMP_TX_POINTS_PER_SEC)

#define TEMP_VALID_MIN_C (-20.0f)
#define TEMP_VALID_MAX_C (80.0f)

static PID_TypeDef pid_press;
static PID_TypeDef pid_heat_L, pid_heat_R;

/* 公开给其他模块观察的调试量：当前气泵 PWM。 */
volatile uint16_t gPumpPwmDebug = 0;

/* 当前控制配置，由 START / UPDATE_CFG 命令更新。 */
static control_config_t gCfg;

/* 急停标志，预留给外部保护逻辑使用。 */
static uint8_t emergency_stop = 0;

static bool temp_value_valid_for_tx(float temp_c)
{
    return (temp_c > TEMP_VALID_MIN_C) && (temp_c < TEMP_VALID_MAX_C);
}

/*
 * 新波形规则：
 * - 总治疗时间 = N 个 60 秒循环
 * - 每个 60 秒循环内固定分为 3 个阶段：
 *   1) 缓慢上升
 *   2) 持续保压
 *   3) 脉动挤压
 * - 暂停时只冻结“总治疗时间位置”，恢复后继续该时刻的波形
 */
typedef enum {
    CTRL_STATE_IDLE = 0,
    CTRL_STATE_RISE,
    CTRL_STATE_HOLD,
    CTRL_STATE_PULSE,
} ctrl_state_t;

static ctrl_state_t ctrl_state = CTRL_STATE_IDLE;

/* 波形锚点：当前治疗从哪个 tick 开始计算总治疗时间位置。 */
static TickType_t wave_anchor_tick = 0;
/* 暂停标志：暂停时保持当前状态机上下文，但所有输出强制关闭。 */
static uint8_t control_paused = 0;
/* 暂停时记录已经走过的总治疗时间，用于恢复时继续同一波形位置。 */
static TickType_t paused_elapsed_ticks = 0;

/*
 * apply_idle_outputs
 * 将所有执行器切换到安全空闲态。
 *
 * 执行动作：
 * - 左右加热 PWM = 0
 * - 左右加热电源关闭
 * - 左右治疗通道阀恢复默认态
 * - 波形阀关闭
 * - 气泵 PWM = 0
 *
 * 这个函数只负责“输出层”，不负责修改上层配置。
 */
static void apply_idle_outputs(void)
{
    HeatPWMSet(Left, 0);
    HeatPower(Left, 0);

    HeatPWMSet(Right, 0);
    HeatPower(Right, 0);

    /* 阀门恢复到统一封装的空闲态。 */
    ValveControl_SetIdle();

    TIM15->CCR1 = 0;
    gPumpPwmDebug = 0;
}

/*
 * ControlTask
 * 主控制任务，每 2ms 调度一次。
 *
 * 执行顺序：
 * 1) 处理控制命令
 * 2) 判断是否应进入空闲 / 保护路径
 * 3) 运行压力状态机与温控
 * 4) 周期发送遥测
 * 5) 周期检查加热盾状态
 */
void ControlTask(void *argument)
{
    (void)argument;

    /* 上电默认配置清零，等待外部 START 命令写入有效参数。 */
    memset(&gCfg, 0, sizeof(gCfg));
    HeaterShieldStatus_Init();

    /*
     * PID 参数说明：
     * - 左右温控 PID 独立，输出范围到 1999，对应加热 PWM
     * - 压力 PID 控制公共气泵，统一使用 gSensorData.pressL 作为闭环反馈，输出范围限制到 255
     */
    PID_Init(&pid_heat_L, 3.9, 1.8, 200, 100000, 0, 1999, 0, 0);
    PID_Init(&pid_heat_R, 3.9, 1.8, 200, 100000, 0, 1999, 0, 0);
    /*
     * 保持原来的压力 PID 上电初值不变。
     * 这样本次重构只调整代码归属，不改变系统刚启动时的实际控制行为。
     */
    PID_Init(&pid_press,
             50.0f,
             10.0f,
             0.0f,
             200,
             -200,
             255,
             0,
             0);

    /* 气泵 PWM 所在定时器必须先启动，否则写 CCR 无效。 */
    HAL_TIM_PWM_Start(&htim15, TIM_CHANNEL_1);

    /*
     * 遥测发送节拍基准。
     * - 压力单通道: PRESS_TX_POINTS_PER_SEC 点/秒
     * - 温度单通道: TEMP_TX_POINTS_PER_SEC 点/秒
     * - 阶段信息跟随温度一起发送
     */
    TickType_t next_press_tx_tick = 0;
    TickType_t next_temp_tx_tick = 0;
    float last_tx_temp_l = 0.0f;
    float last_tx_temp_r = 0.0f;
    bool last_tx_temp_l_valid = false;
    bool last_tx_temp_r_valid = false;

    /* 每 10 次循环检查一次加热盾状态，约 100ms。 */
    uint8_t heater_status_div = 0;

    /* 当前阶段给上位机上报一个简写字符。 */
    const char *phase = "idle";
    const char *last_logged_phase = "idle";
    wave_control_pid_stage_t last_pressure_pid_stage = WAVE_CONTROL_PID_STAGE_RISE;
    bool pressure_pid_profile_loaded = false;
    uint32_t last_pressure_pid_version = WaveControl_GetPressurePidVersion();

    apply_idle_outputs();
    ctrl_state = CTRL_STATE_IDLE;

    for (;;)
    {
        /*
         * 控制主循环周期。
         * 由于后续大量逻辑都基于 tick 差值计算时间，因此这里无需阻塞更长时间。
         */
        vTaskDelay(pdMS_TO_TICKS(2));

        /* 熔断脉冲的异步高/低电平切换在这里统一服务，
         * 这样 Uart_Communicate 不需要再为了 10ms 脉冲阻塞任务。
         */
        HeaterShieldStatus_Service();

        /* 1) 处理来自 AppTask 的控制命令 */
        ctrl_cmd_t c;
        if (xQueueReceive(gCtrlCmdQueue, &c, 0) == pdPASS)
        {
            if (c.id == CTRL_CMD_STOP) {
                /*
                 * STOP：
                 * - 终止运行
                 * - 清除急停状态
                 * - 进入 IDLE 并关闭所有输出
                 */
                gCfg.running = 0;
                emergency_stop = 0;
                control_paused = 0;
                paused_elapsed_ticks = 0;
                ctrl_state = CTRL_STATE_IDLE;
                apply_idle_outputs();
            } else if (c.id == CTRL_CMD_START) {
                /*
                 * START：
                 * - 全量接收新配置
                 * - 标记进入运行态
                 * - 从总治疗时间起点开始一个新的 60 秒循环序列
                 * - 对已使能通道执行一次 OTP_Reset
                 * - 更新温控 PID 目标
                 */
                gCfg = c.cfg;
                gCfg.running = 1;
                emergency_stop = 0;
                control_paused = 0;
                paused_elapsed_ticks = 0;
                wave_anchor_tick = xTaskGetTickCount();
                ctrl_state = CTRL_STATE_RISE;

                if (gCfg.press_enable_L) {
                    OTP_Reset(Left);
                }
                if (gCfg.press_enable_R) {
                    OTP_Reset(Right);
                }

                pid_heat_L.setpoint = (float)(gCfg.temp_target * 100.0f);
                pid_heat_R.setpoint = (float)(gCfg.temp_target * 100.0f);
            } else if (c.id == CTRL_CMD_PAUSE) {
                /* Pause keeps the current state machine position, but all
                 * outputs are forced off until a resume command arrives.
                 */
                control_paused = 1;
                paused_elapsed_ticks = xTaskGetTickCount() - wave_anchor_tick;
                apply_idle_outputs();
            } else if (c.id == CTRL_CMD_RESUME) {
                /* Resume restores the same total-treatment-time position by
                 * rebuilding the anchor tick from the paused elapsed ticks.
                 */
                if (control_paused) {
                    TickType_t now = xTaskGetTickCount();
                    wave_anchor_tick = now - paused_elapsed_ticks;
                    control_paused = 0;
                }
            } else if (c.id == CTRL_CMD_UPDATE_CFG) {
                /*
                 * UPDATE_CFG：
                 * - 用新配置替换旧配置
                 * - 更新温控目标
                 *
                 * 这里不重启波形时间轴，确保运行中改参数不会把
                 * “当前治疗已经走到哪个时刻”重置回 0。
                 */
                if (gCfg.press_enable_L) {
                    OTP_Reset(Left);
                }
                if (gCfg.press_enable_R) {
                    OTP_Reset(Right);
                }

                gCfg = c.cfg;

                pid_heat_L.setpoint = (float)(gCfg.temp_target * 100.0f);
                pid_heat_R.setpoint = (float)(gCfg.temp_target * 100.0f);
            }
        }

        /* Pause is handled before the normal idle/running split:
         * the treatment session still exists, but all outputs must remain off
         * until the host explicitly sends resume or stop.
         */
        if (control_paused) {
            phase = "pause";
            apply_idle_outputs();
            goto telemetry;
        }

        /*
         * 2) 空闲 / 保护路径
         *
         * 满足以下任一条件就不进入运行控制：
         * - 未运行
         * - 急停
         * - 左右两侧都被禁用
         *
         * 这里仍然保留一条状态变化日志，便于观察为什么进入空闲。
         */
        if (!gCfg.running || emergency_stop || (!gCfg.press_enable_L && !gCfg.press_enable_R)) {
            static uint8_t last_idle_reason = 0xFF;
            uint8_t idle_reason = 0;
            bool reason_not_running = !gCfg.running;
            bool reason_emergency = (emergency_stop != 0);
            bool reason_both_disabled = (!gCfg.press_enable_L && !gCfg.press_enable_R);

            if (reason_not_running) idle_reason |= 0x01;
            if (reason_emergency) idle_reason |= 0x02;
            if (reason_both_disabled) idle_reason |= 0x04;

            if (idle_reason != last_idle_reason) {
                last_idle_reason = idle_reason;
                if (reason_not_running || reason_emergency || reason_both_disabled) {
                LOG_W("控制空闲：%s%s%s(run=%u emg=%u enL=%u enR=%u)",
                          reason_not_running ? "关闭; " : "",
                          reason_emergency ? "急停; " : "",
                          reason_both_disabled ? "双侧压力禁用; " : "",
                          gCfg.running, emergency_stop, gCfg.press_enable_L, gCfg.press_enable_R);
                } else {
                LOG_W("控制空闲：未知(run=%u emg=%u enL=%u enR=%u)",
                          gCfg.running, emergency_stop, gCfg.press_enable_L, gCfg.press_enable_R);
                }
            }

            ctrl_state = CTRL_STATE_IDLE;
            phase = "idle";
            apply_idle_outputs();
        } else {
            /* 3) 运行路径：执行压力状态机 */

            /*
             * 按总治疗时间位置直接计算当前所处波形阶段。
             * 一个固定 60 秒循环中，t1/t2/t3 只决定三个阶段的占比；
             * normalize_wave_stage_ms() 会把这三个时长映射到 60 秒总窗内。
             */
            /*
             * 当前统一目标压力。
             * 由于系统不再区分左右压差，控制闭环只维护一个公共压力目标。
             */
            wave_control_pressure_plan_t pressure_plan;
            uint16_t pump_pwm = 0U;

            /*
             * 当前压力控制所需的阶段、目标压力、波形阀动作统一由 WaveControl 给出，
             * ControlTask 这里只负责把计划翻译成硬件执行。
             */
            WaveControl_BuildPressurePlan(&gCfg, wave_anchor_tick, &pressure_plan);
            phase = WaveControl_PhaseName(pressure_plan.snapshot.phase);

            switch (pressure_plan.snapshot.phase) {
            case WAVE_CONTROL_PHASE_RISE:
                ctrl_state = CTRL_STATE_RISE;
                break;
            case WAVE_CONTROL_PHASE_HOLD:
                ctrl_state = CTRL_STATE_HOLD;
                break;
            case WAVE_CONTROL_PHASE_PULSE_ON:
            case WAVE_CONTROL_PHASE_PULSE_OFF:
                ctrl_state = CTRL_STATE_PULSE;
                break;
            case WAVE_CONTROL_PHASE_IDLE:
            case WAVE_CONTROL_PHASE_PAUSE:
            default:
                ctrl_state = CTRL_STATE_IDLE;
                break;
            }

            /*
             * 压力 PID 的阶段切换也统一交给 WaveControl。
             * 只有波形阶段真正变化时，才重载一遍 PID 参数，避免积分串段。
             */
            uint32_t pressure_pid_version = WaveControl_GetPressurePidVersion();
            if ((!pressure_pid_profile_loaded) ||
                (pressure_plan.pid_stage != last_pressure_pid_stage) ||
                (pressure_pid_version != last_pressure_pid_version)) {
                WaveControl_ApplyPressurePidProfile(&pid_press, pressure_plan.pid_stage);
                last_pressure_pid_stage = pressure_plan.pid_stage;
                last_pressure_pid_version = pressure_pid_version;
                pressure_pid_profile_loaded = true;
            }

            /* 调试日志：只在阶段变化时打印一次当前 60 秒循环的分段结果。
             * 这样可以直接看到这轮实际生效的是不是 60 秒，以及 t1/t2/t3
             * 被归一化后各占多少毫秒。
             */
            if (strcmp(phase, last_logged_phase) != 0) {
                // LOG_I("波形阶段: phase=%s cycle_ms=%lu elapsed_ms=%lu rise_ms=%lu hold_ms=%lu pulse_ms=%lu Pmax=%.2f",
                //       phase,
                //       (unsigned long)WAVE_CONTROL_CYCLE_MS,
                //       (unsigned long)pressure_plan.snapshot.cycle_elapsed_ms,
                //       (unsigned long)pressure_plan.snapshot.rise_ms,
                //       (unsigned long)pressure_plan.snapshot.hold_ms,
                //       (unsigned long)pressure_plan.snapshot.pulse_ms,
                //       (double)Pmax);
                last_logged_phase = phase;
            }

            /*
             * 如果左右两侧都被禁用，上面已经在空闲路径提前返回。
             * 因此这里不再区分左右目标压力，只保留一个公共目标。
             */

            if (pressure_plan.open_wave_valve) {
                /*
                 * 需要补压的阶段：
                 * - 先根据单双眼配置选择治疗通道
                 * - 再用公共波形阀 VALVE(1) 打开当前这拍挤压
                 */
                ValveControl_ApplyTreatmentRoute(gCfg.press_enable_L, gCfg.press_enable_R);
                ValveControl_SetWave(1);
            } else {
                /*
                 * 当前不是补压阶段：
                 * - 气泵关闭
                 * - 阀门关闭
                 *
                 * 这里注释里写“泄压”，但从代码动作看，本质是关闭充气通路。
                 * 实际泄压效果取决于外部气路设计。
                */
                TIM15->CCR1 = 0;
                gPumpPwmDebug = 0;
                ValveControl_ApplyTreatmentRoute(gCfg.press_enable_L, gCfg.press_enable_R);
                ValveControl_SetWave(0);
            }

            /*
             * 压力 PID -> 气泵 PWM
             *
             * 统一目标压力 -> 气泵 PWM
             *
             * 输出下限 20：
             * 防止 PID 刚起步时输出过小，气泵无法有效动作。
             * 输出上限 255：
             * 对应当前硬件 PWM 范围上限。
             */
            pump_pwm = WaveControl_ComputePumpPwm(&pid_press,
                                                  pressure_plan.snapshot.target_pressure_kpa,
                                                  gSensorData.pressL,
                                                  pressure_plan.pump_enabled &&
                                                      (gCfg.press_enable_L || gCfg.press_enable_R));
            TIM15->CCR1 = pump_pwm;
            gPumpPwmDebug = pump_pwm;

            /*
             * 温控分左右独立执行：
             * - 常规模式下：该侧使能时打开加热电源并交给 PID_Heat 控制 PWM
             * - 调试模式下：无论状态如何，都强制关闭左右加热，便于排查
             *   “开始治疗后温度采样是否受加热干扰”这个问题。
             */
#if DISABLE_HEATING_FOR_DEBUG
            HeatPWMSet(Left, 0);
            HeatPower(Left, 0);
            HeatPWMSet(Right, 0);
            HeatPower(Right, 0);
#else
            if (gCfg.press_enable_L) {
                HeatPower(Left, 1);
                PID_Heat(Left, &pid_heat_L, (float)(gSensorData.tempL * 100.0f));
            } else {
                HeatPWMSet(Left, 0);
                HeatPower(Left, 0);
            }

            if (gCfg.press_enable_R) {
                HeatPower(Right, 1);
                PID_Heat(Right, &pid_heat_R, (float)(gSensorData.tempR * 100.0f));
            } else {
                HeatPWMSet(Right, 0);
                HeatPower(Right, 0);
            }
#endif
        }

telemetry:
        /*
         * 4) 遥测上报
         *
         * 压力和温度分开按不同节拍发送：
         * - 左右压力：按 PRESS_TX_POINTS_PER_SEC 发送
         * - 左右温度：按 TEMP_TX_POINTS_PER_SEC 发送
         * - 当前阶段：跟随温度一起发送
         */
        if (gCfg.running) {
            TickType_t now = xTaskGetTickCount();
            tx_frame_t tx;

            if ((int32_t)(now - next_press_tx_tick) >= 0) {
                next_press_tx_tick = now + pdMS_TO_TICKS(PRESS_TX_PERIOD_MS);

                tx.type = TX_DATA_FLOAT;

                tx.frame_id = F32_LEFT_PRESSURE_VALUE;
                tx.v.f32 = gSensorData.pressL;
                xQueueSend(gTxQueue, &tx, 0);
            }

            if ((int32_t)(now - next_temp_tx_tick) >= 0) {
                next_temp_tx_tick = now + pdMS_TO_TICKS(TEMP_TX_PERIOD_MS);

                tx.type = TX_DATA_FLOAT;

                if (temp_value_valid_for_tx(gSensorData.tempL)) {
                    last_tx_temp_l = gSensorData.tempL;
                    last_tx_temp_l_valid = true;
                }
                if (temp_value_valid_for_tx(gSensorData.tempR)) {
                    last_tx_temp_r = gSensorData.tempR;
                    last_tx_temp_r_valid = true;
                }

                /*
                 * 温度上报时附加一个显示偏移量。
                 * 控制回路使用的仍然是原始采样值 gSensorData.tempX。
                 * 如果当前采样值异常，则继续发送上一次有效值，避免曲线出现瞬时跳 0。
                 */
                tx.frame_id = F32_LEFT_TEMP_VALUE;
                tx.v.f32 = (last_tx_temp_l_valid ? last_tx_temp_l : gSensorData.tempL) + TEMP_DISPLAY_OFFSET_C;
                xQueueSend(gTxQueue, &tx, 0);

                tx.frame_id = F32_RIGHT_TEMP_VALUE;
                tx.v.f32 = (last_tx_temp_r_valid ? last_tx_temp_r : gSensorData.tempR) + TEMP_DISPLAY_OFFSET_C;
                xQueueSend(gTxQueue, &tx, 0);

                /*
                 * 当前阶段仅上传 phase[0]：
                 * - rise -> 'r'
                 * - hold -> 'h'
                 * - pulse_on / pulse_off -> 'p'
                 * - vent -> 'v'
                 * - idle -> 'i'
                 */
                tx.type = TX_DATA_UINT8;
                tx.frame_id = U8_MODE_CURVES;
                tx.v.u8 = (uint8_t)(phase[0]);
                xQueueSend(gTxQueue, &tx, 0);

                /*
                 * 以下调试量当前不发送，但保留下来便于断点观察或后续扩展。
                 */
                tx.type = TX_DATA_UINT8;
                uint16_t pump_pwm = TIM15->CCR1;
                uint16_t heat_pwm_L = TIM14->CCR1;
                uint16_t heat_pwm_R = TIM17->CCR1;
                (void)pump_pwm;
                (void)heat_pwm_L;
                (void)heat_pwm_R;
            }
        } else {
            next_press_tx_tick = xTaskGetTickCount() + pdMS_TO_TICKS(PRESS_TX_PERIOD_MS);
            next_temp_tx_tick = xTaskGetTickCount() + pdMS_TO_TICKS(TEMP_TX_PERIOD_MS);
        }

        /*
         * 5) 加热盾状态检查
         * 每约 100ms 执行一次，频率不需要太高，但必须持续检查。
         */
        if (++heater_status_div >= 10) {
            heater_status_div = 0;
            HeaterShieldStatus_Process(&gCfg);
        }

    }
}
