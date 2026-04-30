#include "WaveControl/wave_control.h"

/*
 * 压力 PID 分阶段参数：
 * - RISE：缓慢上升阶段，强调跟随性
 * - HOLD：持续保压阶段，强调稳态维持
 * - PULSE：脉动挤压阶段，强调快速建立压力
 *
 * 现在这三组参数集中放在 WaveControl 内部统一维护，
 * 后续调压时只需要改这一个文件。
 */
static float s_press_pid_rise_kp = 80.0f;
static float s_press_pid_rise_ki = 0.0f;
static float s_press_pid_rise_kd = 0.0f;

static float s_press_pid_hold_kp = 140.0f;
static float s_press_pid_hold_ki = 8.0f;
static float s_press_pid_hold_kd = 0.0f;

static float s_press_pid_pulse_kp = 220.0f;
static float s_press_pid_pulse_ki = 0.0f;
static float s_press_pid_pulse_kd = 0.0f;
static uint32_t s_press_pid_version = 0U;

void WaveControl_GetPressurePidGains(wave_control_pid_stage_t stage,
                                     float *kp,
                                     float *ki,
                                     float *kd)
{
    switch (stage) {
    case WAVE_CONTROL_PID_STAGE_RISE:
        *kp = s_press_pid_rise_kp;
        *ki = s_press_pid_rise_ki;
        *kd = s_press_pid_rise_kd;
        break;
    case WAVE_CONTROL_PID_STAGE_HOLD:
        *kp = s_press_pid_hold_kp;
        *ki = s_press_pid_hold_ki;
        *kd = s_press_pid_hold_kd;
        break;
    case WAVE_CONTROL_PID_STAGE_PULSE:
    default:
        *kp = s_press_pid_pulse_kp;
        *ki = s_press_pid_pulse_ki;
        *kd = s_press_pid_pulse_kd;
        break;
    }
}

void WaveControl_SetPressurePidGains(wave_control_pid_stage_t stage,
                                     float kp,
                                     float ki,
                                     float kd)
{
    switch (stage) {
    case WAVE_CONTROL_PID_STAGE_RISE:
        s_press_pid_rise_kp = kp;
        s_press_pid_rise_ki = ki;
        s_press_pid_rise_kd = kd;
        break;
    case WAVE_CONTROL_PID_STAGE_HOLD:
        s_press_pid_hold_kp = kp;
        s_press_pid_hold_ki = ki;
        s_press_pid_hold_kd = kd;
        break;
    case WAVE_CONTROL_PID_STAGE_PULSE:
    default:
        s_press_pid_pulse_kp = kp;
        s_press_pid_pulse_ki = ki;
        s_press_pid_pulse_kd = kd;
        break;
    }

    /* ControlTask uses this version to reload the active PID on the next loop. */
    s_press_pid_version++;
}

uint32_t WaveControl_GetPressurePidVersion(void)
{
    return s_press_pid_version;
}

/*
 * 将配置中的 t1/t2/t3 按占比映射到固定 60 秒窗口。
 * 这样软件里可以继续调整三个阶段的相对时长，但每轮总时长始终严格保持 60 秒。
 */
void WaveControl_NormalizeStageMs(uint32_t *rise_ms, uint32_t *hold_ms, uint32_t *pulse_ms)
{
    uint32_t r = *rise_ms;
    uint32_t h = *hold_ms;
    uint32_t p = *pulse_ms;
    uint32_t total = r + h + p;

    if (total == 0U) {
        r = 20000U;
        h = 20000U;
        p = 20000U;
    } else {
        r = (uint32_t)(((uint64_t)r * WAVE_CONTROL_CYCLE_MS) / total);
        h = (uint32_t)(((uint64_t)h * WAVE_CONTROL_CYCLE_MS) / total);

        if (r + h >= WAVE_CONTROL_CYCLE_MS) {
            if (r >= WAVE_CONTROL_CYCLE_MS) {
                r = WAVE_CONTROL_CYCLE_MS;
                h = 0U;
            } else {
                h = WAVE_CONTROL_CYCLE_MS - r;
            }
            p = 0U;
        } else {
            p = WAVE_CONTROL_CYCLE_MS - r - h;
        }
    }

    *rise_ms = r;
    *hold_ms = h;
    *pulse_ms = p;
}

/*
 * 根据“总治疗已经走到哪个时刻”，计算当前处于 60 秒循环里的哪一个阶段。
 * 这里不直接驱动任何执行器，只输出一份描述性的快照给 ControlTask 使用。
 */
void WaveControl_ComputeSnapshot(const control_config_t *cfg,
                                 TickType_t wave_anchor_tick,
                                 wave_control_snapshot_t *snapshot)
{
    uint32_t rise_ms = (uint32_t)(cfg->t1_rise_s * 1000.0f);
    uint32_t hold_ms = (uint32_t)(cfg->t2_hold_s * 1000.0f);
    uint32_t pulse_ms = (uint32_t)(cfg->t3_pulse_s * 1000.0f);
    uint32_t elapsed_total_ms;
    uint32_t cycle_elapsed_ms;

    WaveControl_NormalizeStageMs(&rise_ms, &hold_ms, &pulse_ms);

    elapsed_total_ms = (uint32_t)((xTaskGetTickCount() - wave_anchor_tick) * portTICK_PERIOD_MS);
    cycle_elapsed_ms = elapsed_total_ms % WAVE_CONTROL_CYCLE_MS;

    snapshot->cycle_elapsed_ms = cycle_elapsed_ms;
    snapshot->rise_ms = rise_ms;
    snapshot->hold_ms = hold_ms;
    snapshot->pulse_ms = pulse_ms;
    snapshot->target_pressure_kpa = 0.0f;
    snapshot->inflating_phase = false;

    if (cycle_elapsed_ms < rise_ms) {
        float ratio = (rise_ms > 0U) ? ((float)cycle_elapsed_ms / (float)rise_ms) : 1.0f;
        if (ratio > 1.0f) {
            ratio = 1.0f;
        }

        snapshot->phase = WAVE_CONTROL_PHASE_RISE;
        snapshot->target_pressure_kpa = cfg->press_target_max * ratio;
        snapshot->inflating_phase = true;
    } else if (cycle_elapsed_ms < (rise_ms + hold_ms)) {
        snapshot->phase = WAVE_CONTROL_PHASE_HOLD;
        snapshot->target_pressure_kpa = cfg->press_target_max;
        snapshot->inflating_phase = true;
    } else {
        uint32_t pulse_elapsed_ms = cycle_elapsed_ms - rise_ms - hold_ms;
        bool on_phase =
            ((pulse_elapsed_ms % WAVE_CONTROL_PULSE_PERIOD_MS) < WAVE_CONTROL_PULSE_ON_MS);

        snapshot->phase = on_phase ? WAVE_CONTROL_PHASE_PULSE_ON : WAVE_CONTROL_PHASE_PULSE_OFF;
        snapshot->target_pressure_kpa = on_phase ? cfg->press_target_max : 0.0f;
        snapshot->inflating_phase = on_phase;
    }
}

/*
 * 生成当前这一拍的压力控制计划：
 * - 是否处于补压阶段
 * - 波形阀是否打开
 * - 气泵是否应该参与闭环
 *
 * 这里不直接操作任何硬件，只给出“本拍应该怎么控”的结果。
 */
void WaveControl_BuildPressurePlan(const control_config_t *cfg,
                                   TickType_t wave_anchor_tick,
                                   wave_control_pressure_plan_t *plan)
{
    WaveControl_ComputeSnapshot(cfg, wave_anchor_tick, &plan->snapshot);

    /*
     * 只有在需要补压的阶段，才打开波形阀并允许气泵输出。
     * 左右路由阀由上层根据单双眼配置决定，这里只管波形节奏。
     */
    plan->open_wave_valve = plan->snapshot.inflating_phase;
    plan->pump_enabled = plan->snapshot.inflating_phase;

    /*
     * PID 参数只按 rise / hold / pulse 三段切换。
     * 脉动阶段内部的 on/off 只改变阀和泵，不重新清 PID 历史量。
     */
    switch (plan->snapshot.phase) {
    case WAVE_CONTROL_PHASE_RISE:
        plan->pid_stage = WAVE_CONTROL_PID_STAGE_RISE;
        break;
    case WAVE_CONTROL_PHASE_HOLD:
        plan->pid_stage = WAVE_CONTROL_PID_STAGE_HOLD;
        break;
    case WAVE_CONTROL_PHASE_PULSE_ON:
    case WAVE_CONTROL_PHASE_PULSE_OFF:
    case WAVE_CONTROL_PHASE_IDLE:
    case WAVE_CONTROL_PHASE_PAUSE:
    default:
        plan->pid_stage = WAVE_CONTROL_PID_STAGE_PULSE;
        break;
    }
}

/*
 * 切换到对应阶段的压力 PID 参数，并清掉历史项。
 * 这样 rise / hold / pulse 三段可以完全独立调参，不会串积分。
 */
void WaveControl_ApplyPressurePidProfile(PID_TypeDef *pid, wave_control_pid_stage_t stage)
{
    float kp = 0.0f;
    float ki = 0.0f;
    float kd = 0.0f;

    WaveControl_GetPressurePidGains(stage, &kp, &ki, &kd);

    pid->Kp = kp;
    pid->Ki = ki;
    pid->Kd = kd;
    pid->integral = 0.0f;
    pid->previous_error = 0.0f;
    pid->previous_measured_value = 0.0f;
    pid->derivative_filtered = 0.0f;
    pid->debug.p = 0.0f;
    pid->debug.i = 0.0f;
    pid->debug.d = 0.0f;
    pid->debug.error = 0.0f;
    pid->debug.output = 0.0f;
}

/*
 * 根据统一目标压力和实时反馈压力，计算公共气泵 PWM。
 * 返回值已经按当前硬件能力限制到 0~255。
 */
uint16_t WaveControl_ComputePumpPwm(PID_TypeDef *pid,
                                    float target_pressure_kpa,
                                    float feedback_pressure_kpa,
                                    bool pump_enabled)
{
    int32_t pump_pwm;

    if (!pump_enabled) {
        return 0U;
    }

    pid->setpoint = target_pressure_kpa;
    pump_pwm = PID_Compute(pid, feedback_pressure_kpa);

    if (pump_pwm < 0) {
        pump_pwm = 0;
    }
    if (pump_pwm > 255) {
        pump_pwm = 255;
    }

    return (uint16_t)pump_pwm;
}

/*
 * 为日志和遥测提供阶段名。
 * ControlTask 继续沿用旧的字符串风格，这样上位机兼容性不需要改。
 */
const char *WaveControl_PhaseName(wave_control_phase_t phase)
{
    switch (phase) {
    case WAVE_CONTROL_PHASE_RISE:
        return "rise";
    case WAVE_CONTROL_PHASE_HOLD:
        return "hold";
    case WAVE_CONTROL_PHASE_PULSE_ON:
        return "pulse_on";
    case WAVE_CONTROL_PHASE_PULSE_OFF:
        return "pulse_off";
    case WAVE_CONTROL_PHASE_PAUSE:
        return "pause";
    case WAVE_CONTROL_PHASE_IDLE:
    default:
        return "idle";
    }
}
