/* Included by app_tasks.c. Bench state is owned by ControlTask. */
#define BENCH_DATA 0x06U
#define BENCH_START_AGE 0x10D0U
#define BENCH_START_LEAK 0x10D1U
#define BENCH_STOP 0x10D2U
#define BENCH_QUERY 0x10D3U
#define BENCH_ACK 0x11D0U
#define BENCH_STATUS 0x11D1U
#define BENCH_RESULT 0x11D2U

typedef struct {
    uint32_t id, duration_s, valid_ms, cycles, elapsed_s;
    TickType_t start, lease, report, last_sample;
    uint8_t active, leak, route, reason, phase;
    bool settled;
    float target, weighted_pwm, peak_pwm;
    uint32_t on_ms;
} RkBench;

static void BenchSend(uint16_t id, const uint8_t *bytes, uint8_t size)
{
    tx_frame_t tx = {.frame_id = id, .type = TX_DATA_BENCH, .length = size};
    memcpy(tx.v.text, bytes, size);
    (void)xQueueSend(gTxQueue, &tx, 0);
}

static void BenchAck(uint32_t id, uint16_t command, uint8_t status)
{
    uint8_t p[7];
    memcpy(p, &id, 4);
    memcpy(p + 4, &command, 2);
    p[6] = status;
    BenchSend(BENCH_ACK, p, sizeof(p));
}

static void BenchResult(const RkBench *b)
{
    uint8_t p[29];
    const float mean = b->valid_ms ? b->weighted_pwm / b->valid_ms : 0.0f;
    const float on = b->valid_ms ? 100.0f * b->on_ms / b->valid_ms : 0.0f;
    memcpy(p, &b->id, 4);
    p[4] = b->reason;
    memcpy(p + 5, &b->elapsed_s, 4);
    memcpy(p + 9, &b->cycles, 4);
    memcpy(p + 13, &b->valid_ms, 4);
    memcpy(p + 17, &mean, 4);
    memcpy(p + 21, &b->peak_pwm, 4);
    memcpy(p + 25, &on, 4);
    BenchSend(BENCH_RESULT, p, sizeof(p));
}

static void BenchFinish(RkBench *b, TreatmentAppController *controller,
                        TickType_t now, uint8_t reason)
{
    if (!b->active) return;
    const uint32_t ms = (uint32_t)(now - b->start) * portTICK_PERIOD_MS;
    b->elapsed_s = ms / 1000U;
    b->cycles = b->leak ? 0U : ms / TREATMENT_CYCLE_MS;
    b->reason = reason;
    b->active = 0U;
    controller->bench_hold_override = false;
    TreatmentAppController_SetSystemIdleOutputs();
    TreatmentPressureControl_ResetPid(controller);
    BenchResult(b);
}

static void BenchStatus(const RkBench *b, float pwm)
{
    uint8_t p[26];
    const float left = gSensorData.pressL, right = gSensorData.pressR;
    memcpy(p, &b->id, 4);
    p[4] = b->active ? 1 : 2;
    p[5] = b->phase;
    memcpy(p + 6, &b->elapsed_s, 4);
    memcpy(p + 10, &b->cycles, 4);
    memcpy(p + 14, &left, 4);
    memcpy(p + 18, &right, 4);
    memcpy(p + 22, &pwm, 4);
    BenchSend(BENCH_STATUS, p, sizeof(p));
}

static void BenchReceive(const CommunicationFrameView *frame)
{
    static const uint8_t lengths[] = {11, 23, 4, 4};
    const uint16_t id = frame->frame_id;
    ctrl_cmd_t cmd = {.id = CTRL_CMD_RK_BENCH, .bench_command = id};
    if (frame->payload_length >= 4U) memcpy(&cmd.request_id, frame->payload, 4);
    if (frame->data_type != BENCH_DATA || frame->payload_length != lengths[id - BENCH_START_AGE] ||
        !cmd.request_id) {
        BenchAck(cmd.request_id, id, 1);
        return;
    }
    memcpy(cmd.bench_payload, frame->payload, frame->payload_length);
    if (xQueueSend(gCtrlCmdQueue, &cmd, 0) != pdPASS) BenchAck(cmd.request_id, id, 2);
}

static void BenchCommand(RkBench *b, TreatmentAppController *controller,
                         const ctrl_cmd_t *cmd, TickType_t now, bool available)
{
    const uint32_t id = cmd->request_id;
    const uint16_t op = cmd->bench_command;
    if (op == BENCH_QUERY || op == BENCH_STOP) {
        if (b->id != id) {
            BenchAck(id, op, 4);
            return;
        }
        BenchAck(id, op, 0);
        if (op == BENCH_STOP && b->active) BenchFinish(b, controller, now, 1);
        else if (!b->active) BenchResult(b);
        else {
            b->lease = now + pdMS_TO_TICKS(3000U);
            BenchStatus(b, controller->pressure_pid.debug.mapped_output);
        }
        return;
    }
    const bool leak = op == BENCH_START_LEAK;
    if (id == b->id) {
        BenchAck(id, op, b->leak == leak ? 0 : 1);
        if (!b->active && b->leak == leak) BenchResult(b);
        return;
    }
    if (b->active) {
        BenchAck(id, op, 2);
        return;
    }
    const uint8_t *p = cmd->bench_payload;
    const uint8_t route = leak ? 3U : p[4];
    uint16_t target;
    uint32_t seconds;
    float gains[3];
    memcpy(&target, p + (leak ? 4 : 5), 2);
    memcpy(&seconds, p + (leak ? 6 : 7), 4);
    if (route < 1U || route > 3U || target < 1U || target > 550U ||
        seconds < 1U || seconds > 86400U || (leak && p[10] > 1U)) {
        BenchAck(id, op, 1);
        return;
    }
    if (leak && p[10]) {
        memcpy(gains, p + 11, sizeof(gains));
        for (unsigned i = 0; i < 3U; ++i)
            if (!(gains[i] >= 0.0f && gains[i] <= 1000.0f)) {
                BenchAck(id, op, 1);
                return;
            }
    } else if (leak) {
        TreatmentPressureControl_GetProfileGains(1, TREATMENT_PRESS_PID_STAGE_HOLD, gains);
    }
    if (!available) {
        BenchAck(id, op, 3);
        return;
    }
    *b = (RkBench){.id = id, .duration_s = seconds, .start = now,
        .lease = now + pdMS_TO_TICKS(3000U), .report = now,
        .active = 1, .leak = leak, .route = route, .target = target};
    TreatmentAppController_SetSystemIdleOutputs();
    TreatmentPressureControl_ResetPid(controller);
    controller->wave_anchor_tick = now;
    controller->bench_hold_override = leak;
    if (leak) memcpy(controller->bench_hold_gains, gains, sizeof(gains));
    BenchAck(id, op, 0);
}

static void BenchRun(RkBench *b, TreatmentAppController *controller, TickType_t now,
                     float dt, TreatmentAppRuntime *runtime)
{
    const uint32_t ms = (uint32_t)(now - b->start) * portTICK_PERIOD_MS;
    b->elapsed_s = ms / 1000U;
    b->cycles = b->leak ? 0U : ms / TREATMENT_CYCLE_MS;
    TreatmentPressurePlan plan = {.phase = TREATMENT_PHASE_HOLD,
        .pid_stage = TREATMENT_PRESS_PID_STAGE_HOLD, .inflating = 1,
        .target_pressure_kpa = b->target};
    const control_config_t saved = controller->cfg;
    controller->cfg.press_enable_L = (b->route & 1U) != 0;
    controller->cfg.press_enable_R = (b->route & 2U) != 0;
    controller->cfg.press_target_max = b->target;
    if (!b->leak) TreatmentPressureControl_BuildPlan(controller, now, &plan);
    memset(runtime, 0, sizeof(*runtime));
    TreatmentPressureControl_ApplyPlan(controller, &gSensorData, dt, &plan, runtime);
    controller->cfg = saved;
    runtime->session_active = 0;
    b->phase = plan.phase == TREATMENT_PHASE_CYCLE_VENT ? 5U : (uint8_t)plan.phase;
    const float pwm = controller->pressure_pid.debug.mapped_output;
    if (b->leak) {
        const bool in_band = fabsf(gSensorData.pressL - b->target) <= 5.0f &&
                             fabsf(gSensorData.pressR - b->target) <= 5.0f;
        if (b->settled && in_band) {
            const uint32_t step_ms = (uint32_t)(now - b->last_sample) * portTICK_PERIOD_MS;
            b->valid_ms += step_ms;
            b->weighted_pwm += step_ms * pwm;
            if (pwm > 0.0f) b->on_ms += step_ms;
            if (pwm > b->peak_pwm) b->peak_pwm = pwm;
        }
        b->settled = in_band;
        b->last_sample = now;
    }
    if ((int32_t)(now - b->report) >= 0) {
        b->report = now + pdMS_TO_TICKS(500U);
        BenchStatus(b, runtime->pump_pwm ? pwm : 0.0f);
    }
}
