#ifndef PID_JLINK_H
#define PID_JLINK_H

#include "treatment_app_controller.h"

/* Little-endian, naturally aligned 32-bit words; mirrored by jlink_protocol.py.
 * Host owns request_seq/profile/stage/kp/ki/kd. Write payload first and commit
 * request_seq last, with only one outstanding request. Firmware owns the rest.
 * ack_seq is written last after command_status. sample_seq is a seqlock:
 * host accepts telemetry only if its before/after values match and are even.
 */
typedef struct
{
    uint32_t magic;
    uint32_t version;
    uint32_t size;
    uint32_t request_seq;
    uint32_t ack_seq;
    uint32_t command_status; /* 0=idle, 1=applied, 2=invalid */
    uint32_t profile;
    uint32_t stage;
    float kp;
    float ki;
    float kd;
    uint32_t sample_seq;
    uint32_t tick_ms;
    uint32_t phase;
    uint32_t active_profile;
    uint32_t flags; /* bit0=session, bit1=paused, bit2=emergency stop */
    uint32_t pwm;
    uint32_t pwm_min;
    uint32_t pwm_max;
    float target;
    float left;
    float right;
    float feedback;
    float output;
    float p;
    float i;
    float d;
    float gains[2][3][3]; /* profile, stage, Kp/Ki/Kd */
} PidJlinkMailbox;

extern volatile PidJlinkMailbox g_pid_jlink;
extern const char g_pid_jlink_build_id[32];

void PidJlink_Init(void);
void PidJlink_ProcessCommand(void);
void PidJlink_Publish(const TreatmentAppController *controller,
                      const volatile sensor_data_t *sensor,
                      const TreatmentAppRuntime *runtime, uint32_t tick_ms);

#endif
