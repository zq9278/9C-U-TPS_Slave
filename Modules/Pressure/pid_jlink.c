#include "pid_jlink.h"

#include <stddef.h>
#include "treatment_pressure_control.h"
#include "main.h"

#define PID_JLINK_MAGIC 0x5049444AU

volatile PidJlinkMailbox g_pid_jlink;
/* Host compares these bytes with the selected ELF before any RAM writes. */
_Alignas(4) const char g_pid_jlink_build_id[32] = __DATE__ " " __TIME__;

_Static_assert(sizeof(float) == 4U, "J-Link ABI requires float32");
_Static_assert(sizeof(PidJlinkMailbox) == 180U, "J-Link ABI size");
_Static_assert(offsetof(PidJlinkMailbox, request_seq) == 12U, "J-Link command ABI");
_Static_assert(offsetof(PidJlinkMailbox, profile) == 24U, "J-Link payload ABI");
_Static_assert(offsetof(PidJlinkMailbox, sample_seq) == 44U, "J-Link sample ABI");
_Static_assert(offsetof(PidJlinkMailbox, gains) == 108U, "J-Link gains ABI");

void PidJlink_Init(void)
{
    /* CMake retains the build ID even with --gc-sections and optimisations. */
    g_pid_jlink.version = 1U;
    g_pid_jlink.size = sizeof(g_pid_jlink);
    __DMB();
    g_pid_jlink.magic = PID_JLINK_MAGIC;
}

void PidJlink_ProcessCommand(void)
{
    uint32_t sequence = g_pid_jlink.request_seq;
    uint32_t profile;
    uint32_t stage;
    float kp, ki, kd;

    if (sequence == g_pid_jlink.ack_seq)
    {
        return;
    }
    __DMB();
    profile = g_pid_jlink.profile;
    stage = g_pid_jlink.stage;
    kp = g_pid_jlink.kp;
    ki = g_pid_jlink.ki;
    kd = g_pid_jlink.kd;
    __DMB();
    if (sequence != g_pid_jlink.request_seq)
    {
        return;
    }
    g_pid_jlink.command_status = TreatmentPressureControl_SetProfileGains(
        profile, stage, kp, ki, kd) ? 1U : 2U;
    __DMB();
    g_pid_jlink.ack_seq = sequence;
}

void PidJlink_Publish(const TreatmentAppController *controller,
                      const volatile sensor_data_t *sensor,
                      const TreatmentAppRuntime *runtime, uint32_t tick_ms)
{
    uint32_t minimum, maximum;
    uint32_t profile, stage, gain;
    float values[3];
    const PidControllerDebug *debug = &controller->pressure_pid.debug;
    uint8_t pid_active = (uint8_t)((runtime->session_active != 0U) &&
        (controller->paused == 0U) &&
        ((runtime->phase == TREATMENT_PHASE_RISE) ||
         (runtime->phase == TREATMENT_PHASE_HOLD) ||
         (runtime->phase == TREATMENT_PHASE_PULSE_ON)));

    ++g_pid_jlink.sample_seq;
    __DMB();
    g_pid_jlink.tick_ms = tick_ms;
    g_pid_jlink.phase = (uint32_t)runtime->phase;
    g_pid_jlink.active_profile = (controller->cfg.press_enable_L && controller->cfg.press_enable_R) ? 1U : 0U;
    g_pid_jlink.flags = (uint32_t)runtime->session_active |
        ((uint32_t)(controller->paused != 0U) << 1U) |
        ((uint32_t)(controller->emergency_stop != 0U) << 2U);
    g_pid_jlink.pwm = runtime->pump_pwm;
    TreatmentPressureControl_GetPwmLimits(&minimum, &maximum);
    g_pid_jlink.pwm_min = minimum;
    g_pid_jlink.pwm_max = maximum;
    g_pid_jlink.target = runtime->target_pressure_kpa;
    g_pid_jlink.left = sensor->pressL;
    g_pid_jlink.right = sensor->pressR;
    g_pid_jlink.feedback = runtime->feedback_pressure_kpa;
    g_pid_jlink.output = pid_active ? debug->output : 0.0f;
    g_pid_jlink.p = pid_active ? debug->p : 0.0f;
    g_pid_jlink.i = pid_active ? debug->i : 0.0f;
    g_pid_jlink.d = pid_active ? debug->d : 0.0f;
    for (profile = 0U; profile < 2U; ++profile)
    {
        for (stage = 0U; stage < 3U; ++stage)
        {
            TreatmentPressureControl_GetProfileGains(profile, stage, values);
            for (gain = 0U; gain < 3U; ++gain)
            {
                g_pid_jlink.gains[profile][stage][gain] = values[gain];
            }
        }
    }
    __DMB();
    ++g_pid_jlink.sample_seq;
}
