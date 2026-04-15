#include "Uart_Communicate.h"
#include <string.h>
#include <stdbool.h>
#include "control_task.h"
#include "pid.h"
#include "heat.h"
#include "apply.h"
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

/* 调试宏：忽略熔断保护（在位检测仍然生效） */
#define IGNORE_FUSE_PROTECTION_DEBUG
/* 调试宏：为了排查温度采样异常，临时强制关闭左右加热输出。
 * 打开后，加热电源和加热 PWM 都会保持为 0，但压力控制和遥测仍照常运行。
 */
#define DISABLE_HEATING_FOR_DEBUG 0

/*
 * 对“在位检测”和“熔断检测”做简单消抖。
 * 控制循环周期约 2ms，当前状态每 100ms 检查一次；
 * 连续 3 次检测一致后，才更新稳定状态。
 */
#define HEATER_STATUS_DEBOUNCE_COUNT 3U

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
 * 压力状态机定义。
 *
 * IDLE：
 *   安全空闲态，所有输出关闭
 * PRE_RISE_VENT：
 *   启动治疗前先泄压 1s，确保系统从已知低压状态开始
 * RISE：
 *   目标压力从 0 线性爬升到 Pmax
 * HOLD：
 *   维持在 Pmax
 * PULSE：
 *   在 Pmax 和 0 之间按 on/off 周期切换
 * INTER_CYCLE_VENT：
 *   一个周期完成后的固定泄压阶段
 */
typedef enum {
    CTRL_STATE_IDLE = 0,
    CTRL_STATE_PRE_RISE_VENT,
    CTRL_STATE_RISE,
    CTRL_STATE_HOLD,
    CTRL_STATE_PULSE,
    CTRL_STATE_INTER_CYCLE_VENT,
} ctrl_state_t;

static ctrl_state_t ctrl_state = CTRL_STATE_IDLE;

/* 当前状态进入时刻。所有“本状态已运行多久”的逻辑都依赖它。 */
static TickType_t state_enter_tick = 0;

/* 当前压力周期开始时刻。 */
static TickType_t cycle_start_tick = 0;

/* 预泄压阶段结束时刻。 */
static TickType_t pre_vent_end_tick = 0;

/* 周期间泄压阶段结束时刻。 */
static TickType_t inter_vent_end_tick = 0;

/* 标记预泄压阶段是否已经到时并完成收尾。 */
static uint8_t pre_vent_close_done = 0;

/* 当前一次运行中，是否已经执行过启动前预泄压。 */
static uint8_t pre_r_vent_done = 0;
/* 暂停标志：暂停时保持当前状态机上下文，但所有输出强制关闭。 */
static uint8_t control_paused = 0;
/* 进入暂停的时刻，用于恢复时整体平移时间戳，冻结状态机时间。 */
static TickType_t pause_enter_tick = 0;

/*
 * ms_since_tick
 * 返回从给定 tick 到当前时刻经过的毫秒数。
 * 主要用于状态机定时。
 */
static inline uint32_t ms_since_tick(TickType_t tick)
{
    TickType_t now = xTaskGetTickCount();
    return (uint32_t)((now - tick) * portTICK_PERIOD_MS);
}

/*
 * enter_state
 * 统一切换状态并刷新状态进入时间。
 * 所有状态跳转都尽量通过这个函数完成，避免遗漏时间戳更新。
 */
static void enter_state(ctrl_state_t s)
{
    ctrl_state = s;
    state_enter_tick = xTaskGetTickCount();
}

/*
 * apply_idle_outputs
 * 将所有执行器切换到安全空闲态。
 *
 * 执行动作：
 * - 左右加热 PWM = 0
 * - 左右加热电源关闭
 * - 左右阀门关闭
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

    VALVE_LEFT(0);
    VALVE_RIGHT(0);

    TIM15->CCR1 = 0;
    gPumpPwmDebug = 0;
}

/*
 * start_pressure_cycle
 * 启动一个新的压力周期。
 *
 * allow_pre_vent = 1 且本次运行还没做过预泄压时：
 * - 先进入 PRE_RISE_VENT，泄压 1 秒
 *
 * 否则：
 * - 直接进入 RISE
 *
 * 每次启动周期都会刷新：
 * - cycle_start_tick：本周期开始时间
 * - state_enter_tick：当前状态开始时间
 */
static void start_pressure_cycle(uint8_t allow_pre_vent)
{
    cycle_start_tick = xTaskGetTickCount();
    state_enter_tick = cycle_start_tick;

    if (allow_pre_vent && !pre_r_vent_done) {
        ctrl_state = CTRL_STATE_PRE_RISE_VENT;
        pre_vent_end_tick = state_enter_tick + pdMS_TO_TICKS(1000);
        pre_vent_close_done = 0;

        /* 预泄压期间关闭阀门和气泵，让系统回到低压初始状态。 */
        VALVE_LEFT(0);
        VALVE_RIGHT(0);
        TIM15->CCR1 = 0;
        gPumpPwmDebug = 0;
    } else {
        ctrl_state = CTRL_STATE_RISE;
    }
}

/*
 * start_inter_cycle_vent
 * 一个完整周期结束后，进入固定 2 秒的周期间泄压阶段。
 *
 * 该阶段关闭阀门和气泵，不再补压；
 * 到时后重新开始下一轮周期。
 */
static void start_inter_cycle_vent(void)
{
    ctrl_state = CTRL_STATE_INTER_CYCLE_VENT;
    state_enter_tick = xTaskGetTickCount();
    inter_vent_end_tick = state_enter_tick + pdMS_TO_TICKS(2000);

    VALVE_LEFT(0);
    VALVE_RIGHT(0);
    TIM15->CCR1 = 0;
    gPumpPwmDebug = 0;
}

/*
 * SendHeaterStatusFrames
 * 采集并上报加热盾相关状态，同时执行保护动作。
 *
 * 检查内容：
 * - 在位检测：MCU_Heat1_Sense / MCU_Heat2_Sense
 * - 熔断检测：Heat1_Fuse_Detection / Heat2_Fuse_Detection
 *
 * 协议约定：
 * - 高电平 = 正常 / 在位 / 未熔断，发送 1
 * - 低电平 = 不在位 / 熔断，发送 0
 *
 * 安全策略：
 * - 不在位：强制关闭对应侧压力、加热、阀门
 * - 熔断：在非调试模式下，强制关闭对应侧
 */
static void SendHeaterStatusFrames(void)
{
    tx_frame_t tx = {0};

    /* 保存消抖后的稳定状态。默认上电认为两侧都正常。 */
    static uint8_t right_present_stable = 1;
    static uint8_t left_present_stable = 1;
    static uint8_t right_fuse_stable = 1;
    static uint8_t left_fuse_stable = 1;

    /* 消抖计数器：只有连续多次不同，才更新稳定状态。 */
    static uint8_t right_present_count = 0;
    static uint8_t left_present_count = 0;
    static uint8_t right_fuse_count = 0;
    static uint8_t left_fuse_count = 0;

    uint8_t right_present_raw =
        (HAL_GPIO_ReadPin(MCU_Heat1_Sense_GPIO_Port, MCU_Heat1_Sense_Pin) == GPIO_PIN_RESET) ? 0 : 1;
    uint8_t left_present_raw =
        (HAL_GPIO_ReadPin(MCU_Heat2_Sense_GPIO_Port, MCU_Heat2_Sense_Pin) == GPIO_PIN_RESET) ? 0 : 1;
    uint8_t right_fuse_raw =
        (HAL_GPIO_ReadPin(Heat1_Fuse_Detection_GPIO_Port, Heat1_Fuse_Detection_Pin) == GPIO_PIN_RESET) ? 0 : 1;
    uint8_t left_fuse_raw =
        (HAL_GPIO_ReadPin(Heat2_Fuse_Detection_GPIO_Port, Heat2_Fuse_Detection_Pin) == GPIO_PIN_RESET) ? 0 : 1;

    uint8_t right_present;
    uint8_t left_present;
    uint8_t right_fuse;
    uint8_t left_fuse;

    /*
     * 在位检测消抖：
     * raw 与 stable 不同时累加计数；
     * 达到阈值后才真正更新 stable。
     */
    if (right_present_raw != right_present_stable) {
        if (++right_present_count >= HEATER_STATUS_DEBOUNCE_COUNT) {
            right_present_stable = right_present_raw;
            right_present_count = 0;
        }
    } else {
        right_present_count = 0;
    }

    if (left_present_raw != left_present_stable) {
        if (++left_present_count >= HEATER_STATUS_DEBOUNCE_COUNT) {
            left_present_stable = left_present_raw;
            left_present_count = 0;
        }
    } else {
        left_present_count = 0;
    }

    /* 熔断检测消抖逻辑与在位检测相同。 */
    if (right_fuse_raw != right_fuse_stable) {
        if (++right_fuse_count >= HEATER_STATUS_DEBOUNCE_COUNT) {
            right_fuse_stable = right_fuse_raw;
            right_fuse_count = 0;
        }
    } else {
        right_fuse_count = 0;
    }

    if (left_fuse_raw != left_fuse_stable) {
        if (++left_fuse_count >= HEATER_STATUS_DEBOUNCE_COUNT) {
            left_fuse_stable = left_fuse_raw;
            left_fuse_count = 0;
        }
    } else {
        left_fuse_count = 0;
    }

    right_present = right_present_stable;
    left_present = left_present_stable;
    right_fuse = right_fuse_stable;
    left_fuse = left_fuse_stable;

    /*
     * 同步到全局传感器聚合结构，便于 Cortex Live Watch / 调试器直接观察。
     * 这些字段不参与控制闭环，只作为调试可视化状态使用。
     */
    gSensorData.heaterPresentL = left_present;
    gSensorData.heaterPresentR = right_present;
    gSensorData.heaterFuseL = left_fuse;
    gSensorData.heaterFuseR = right_fuse;

    /* 将稳定后的状态上报给上位机。 */
    tx.type = TX_DATA_UINT8;

    tx.frame_id = U8_LEFT_HEATER_PRESENT;
    tx.v.u8 = left_present;
    xQueueSend(gTxQueue, &tx, 0);

    tx.frame_id = U8_RIGHT_HEATER_PRESENT;
    tx.v.u8 = right_present;
    xQueueSend(gTxQueue, &tx, 0);

    tx.frame_id = U8_LEFT_HEATER_FUSE;
    tx.v.u8 = left_fuse;
    xQueueSend(gTxQueue, &tx, 0);

    tx.frame_id = U8_RIGHT_HEATER_FUSE;
    tx.v.u8 = right_fuse;
    xQueueSend(gTxQueue, &tx, 0);

    /*
     * 不在位保护：
     * 对应侧一旦不在位，立即禁用该侧，并关闭加热 / 阀门输出。
     */
    if (!left_present) {
        gCfg.press_enable_L = 0;
        HeatPWMSet(Left, 0);
        HeatPower(Left, 0);
        VALVE_LEFT(0);
    }

    if (!right_present) {
        gCfg.press_enable_R = 0;
        HeatPWMSet(Right, 0);
        HeatPower(Right, 0);
        VALVE_RIGHT(0);
    }

#ifndef IGNORE_FUSE_PROTECTION_DEBUG
    /*
     * 熔断保护：
     * 与“不在位保护”类似，但来源是熔断检测脚。
     */
    if (!left_fuse) {
        gCfg.press_enable_L = 0;
        HeatPWMSet(Left, 0);
        HeatPower(Left, 0);
        VALVE_LEFT(0);
    }

    if (!right_fuse) {
        gCfg.press_enable_R = 0;
        HeatPWMSet(Right, 0);
        HeatPower(Right, 0);
        VALVE_RIGHT(0);
    }
#else
    /*
     * 调试模式：
     * 熔断状态仍然会上报，但不会自动关停输出，方便联调。
     */
    static uint8_t fuse_warned = 0;
    if (!fuse_warned && (!left_fuse || !right_fuse)) {
        fuse_warned = 1;
            LOG_W("DEBUG: ignore fuse, heaters stay enabled (L_fuse=%u R_fuse=%u)", left_fuse, right_fuse);
    }
#endif
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

    /*
     * PID 参数说明：
     * - 左右温控 PID 独立，输出范围到 1999，对应加热 PWM
     * - 压力 PID 控制公共气泵，统一使用 gSensorData.pressL 作为闭环反馈，输出范围限制到 255
     */
    PID_Init(&pid_heat_L, 390, 1.8, 200, 100000, 0, 1999, 0, 0);
    PID_Init(&pid_heat_R, 390, 1.8, 200, 100000, 0, 1999, 0, 0);
    PID_Init(&pid_press, 100, 10, 0, 200, -200, 255, 0, 0);

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

    apply_idle_outputs();
    ctrl_state = CTRL_STATE_IDLE;

    for (;;)
    {
        /*
         * 控制主循环周期。
         * 由于后续大量逻辑都基于 tick 差值计算时间，因此这里无需阻塞更长时间。
         */
        vTaskDelay(pdMS_TO_TICKS(2));

        /* 1) 处理来自 AppTask 的控制命令 */
        ctrl_cmd_t c;
        if (xQueueReceive(gCtrlCmdQueue, &c, 0) == pdPASS)
        {
            if (c.id == CTRL_CMD_STOP) {
                /*
                 * STOP：
                 * - 终止运行
                 * - 清除急停状态
                 * - 允许下次 START 时重新执行启动前预泄压
                 * - 进入 IDLE 并关闭所有输出
                 */
                gCfg.running = 0;
                emergency_stop = 0;
                pre_r_vent_done = 0;
                control_paused = 0;
                ctrl_state = CTRL_STATE_IDLE;
                apply_idle_outputs();
            } else if (c.id == CTRL_CMD_START) {
                /*
                 * START：
                 * - 全量接收新配置
                 * - 标记进入运行态
                 * - 重置启动前状态
                 * - 对已使能通道执行一次 OTP_Reset
                 * - 更新温控 PID 目标
                 * - 启动一个新的压力周期
                 */
                gCfg = c.cfg;
                gCfg.running = 1;
                emergency_stop = 0;
                pre_r_vent_done = 0;
                control_paused = 0;

                if (gCfg.press_enable_L) {
                    OTP_Reset(Left);
                }
                if (gCfg.press_enable_R) {
                    OTP_Reset(Right);
                }

                pid_heat_L.setpoint = (float)(gCfg.temp_target * 100.0f);
                pid_heat_R.setpoint = (float)(gCfg.temp_target * 100.0f);

                start_pressure_cycle(1);
            } else if (c.id == CTRL_CMD_PAUSE) {
                /* Pause keeps the current state machine position, but all
                 * outputs are forced off until a resume command arrives.
                 */
                control_paused = 1;
                pause_enter_tick = xTaskGetTickCount();
                apply_idle_outputs();
            } else if (c.id == CTRL_CMD_RESUME) {
                /* Resume freezes time while paused by shifting all timestamps
                 * forward by the paused duration. This lets rise/hold/pulse
                 * continue from the same point instead of restarting.
                 */
                if (control_paused) {
                    TickType_t now = xTaskGetTickCount();
                    TickType_t paused_delta = now - pause_enter_tick;
                    state_enter_tick += paused_delta;
                    cycle_start_tick += paused_delta;
                    pre_vent_end_tick += paused_delta;
                    inter_vent_end_tick += paused_delta;
                    control_paused = 0;
                }
            } else if (c.id == CTRL_CMD_UPDATE_CFG) {
                /*
                 * UPDATE_CFG：
                 * - 用新配置替换旧配置
                 * - 如果系统当前处于运行态，则从当前时刻重新开始一个周期
                 *   但不重复执行“启动前预泄压”
                 * - 更新温控目标
                 *
                 * 这里先对旧配置判断左右使能，再执行 OTP_Reset，
                 * 保持现有代码行为不变。
                 */
                if (gCfg.press_enable_L) {
                    OTP_Reset(Left);
                }
                if (gCfg.press_enable_R) {
                    OTP_Reset(Right);
                }

                gCfg = c.cfg;

                if (gCfg.running) {
                    start_pressure_cycle(0);
                }

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
             * 将配置里的秒转换成毫秒，便于和 tick 差值对比。
             * t1: 上升阶段时长
             * t2: 保压阶段时长
             * t3: 脉冲阶段总时长
             */
            uint32_t t1_ms = (uint32_t)(gCfg.t1_rise_s * 1000.0f);
            uint32_t t2_ms = (uint32_t)(gCfg.t2_hold_s * 1000.0f);
            uint32_t t3_ms = (uint32_t)(gCfg.t3_pulse_s * 1000.0f);

            /*
             * 如果当前状态意外落在 IDLE，但系统又是运行态，
             * 则自动补起一个新的周期，防止控制链路中断。
             */
            if (ctrl_state == CTRL_STATE_IDLE) {
                start_pressure_cycle(1);
            }

            /*
             * 当前统一目标压力。
             * 由于系统不再区分左右压差，控制闭环只维护一个公共压力目标。
             */
            float Pset = 0.0f;

            /* 当前配置中的最大目标压力。 */
            float Pmax = gCfg.press_target_max;

            /* 是否需要驱动气泵。 */
            bool pump_on = false;

            /* 当前是否属于“应当补压”的阶段。 */
            bool inflating_phase = false;

            /* 脉冲阶段当前是否处于 on 相。 */
            bool on_phase = false;

            switch (ctrl_state) {
                case CTRL_STATE_PRE_RISE_VENT:
                    phase = "pre_vent";

                    /*
                     * 预泄压阶段不主动补压。
                     * 到时后只执行一次状态跳转，并标记本次运行已做过预泄压。
                     */
                    if (!pre_vent_close_done &&
                        (int32_t)(xTaskGetTickCount() - pre_vent_end_tick) >= 0) {
                        pre_vent_close_done = 1;
                        pre_r_vent_done = 1;
                        enter_state(CTRL_STATE_RISE);
                    }
                    break;

                case CTRL_STATE_RISE: {
                    phase = "rise";

                    /*
                     * 上升阶段：
                     * - 打开左右阀门
                     * - 目标压力按时间线性从 0 上升到 Pmax
                     */
                    VALVE_LEFT(1);
                    VALVE_RIGHT(1);

                    uint32_t elapsed = ms_since_tick(state_enter_tick);
                    float ratio = (t1_ms > 0) ? (float)elapsed / (float)t1_ms : 1.0f;
                    if (ratio > 1.0f) ratio = 1.0f;

                    Pset = Pmax * ratio;
                    inflating_phase = true;

                    /*
                     * 上升阶段结束后：
                     * - 若存在保压时间，进入 HOLD
                     * - 否则若存在脉冲时间，进入 PULSE
                     * - 否则直接进入周期间泄压
                     */
                    if (t1_ms == 0 || elapsed >= t1_ms) {
                        if (t2_ms > 0) {
                            enter_state(CTRL_STATE_HOLD);
                        } else if (t3_ms > 0) {
                            enter_state(CTRL_STATE_PULSE);
                        } else {
                            start_inter_cycle_vent();
                        }
                    }
                    break;
                }

                case CTRL_STATE_HOLD: {
                    phase = "hold";

                    /* 保压阶段目标恒定为 Pmax。 */
                    Pset = Pmax;
                    inflating_phase = true;

                    uint32_t elapsed = ms_since_tick(state_enter_tick);
                    if (t2_ms == 0 || elapsed >= t2_ms) {
                        if (t3_ms > 0) {
                            enter_state(CTRL_STATE_PULSE);
                        } else {
                            start_inter_cycle_vent();
                        }
                    }
                    break;
                }

                case CTRL_STATE_PULSE: {
                    /*
                     * 脉冲阶段：
                     * - 按 pulse_on_ms / pulse_off_ms 周期切换
                     * - on 相目标为 Pmax
                     * - off 相目标为 0
                     */
                    uint32_t elapsed = ms_since_tick(state_enter_tick);
                    uint32_t period = (uint32_t)(gCfg.pulse_on_ms + gCfg.pulse_off_ms);

                    on_phase = (period > 0 && gCfg.pulse_on_ms > 0) ?
                               ((elapsed % period) < (uint32_t)gCfg.pulse_on_ms) : false;

                    phase = on_phase ? "pulse_on" : "pulse_off";
                    Pset = on_phase ? Pmax : 0.0f;
                    inflating_phase = on_phase;

                    if (t3_ms == 0 || elapsed >= t3_ms) {
                        start_inter_cycle_vent();
                    }
                    break;
                }

                case CTRL_STATE_INTER_CYCLE_VENT:
                    phase = "vent";

                    /*
                     * 周期间泄压阶段不保压、不补压；
                     * 到时后直接开启下一周期。
                     */
                    VALVE_LEFT(0);
                    VALVE_RIGHT(0);
                    TIM15->CCR1 = 0;
                    gPumpPwmDebug = 0;

                    if ((int32_t)(xTaskGetTickCount() - inter_vent_end_tick) >= 0) {
                        start_pressure_cycle(0);
                    }
                    break;

                default:
                    break;
            }

            /*
             * 如果左右两侧都被禁用，上面已经在空闲路径提前返回。
             * 因此这里不再区分左右目标压力，只保留一个公共目标。
             */

            if (inflating_phase) {
                /*
                 * 需要补压的阶段，阀门状态跟随左右使能。
                 * 使能哪侧，就打开哪侧阀门。
                 */
                uint8_t left_valve_state = gCfg.press_enable_L;
                uint8_t right_valve_state = gCfg.press_enable_R;

                VALVE_LEFT(left_valve_state);
                VALVE_RIGHT(right_valve_state);

                /*
                 * 只要当前处于补压阶段且至少有一侧开启，就驱动公共气泵。
                 * 压力闭环统一使用 gSensorData.pressL 作为反馈值；
                 * gSensorData.pressR 保留给上报和调试观察，不参与控制。
                 */
                pump_on = (gCfg.press_enable_L || gCfg.press_enable_R);
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
                VALVE_LEFT(0);
                VALVE_RIGHT(0);
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
            if (pump_on) {
                pid_press.setpoint = Pset;
                int32_t u = PID_Compute(&pid_press, gSensorData.pressL);

                //if (u < 20) u = 20;
                if (u > 255) u = 255;

                TIM15->CCR1 = (uint16_t)u;
                gPumpPwmDebug = (uint16_t)u;
            }

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
            SendHeaterStatusFrames();
        }

    }
}
