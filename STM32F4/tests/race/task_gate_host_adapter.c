/* Host-only adapter built from ACTUAL STM32 RaceFsm QR route then RaceTaskPlan.
 * The bridge injects synthetic healthy feedback: this validates C state
 * transitions only, never physical motion or UART.
 */
#include "race_fsm.h"
#include "race_task_gate.h"
#include <stdint.h>
#include <string.h>

typedef struct {
    uint32_t cmd_id;
    uint8_t kind, batch, item, color, position;
    uint8_t state, error, completed;
    uint8_t hardware_authorized;
} HostTaskEvent;

typedef struct {
    uint8_t action_index, kind, state, error;
    uint8_t qr_confirmed, qr_valid;
    uint32_t scan_session;
    uint8_t hardware_authorized;
} HostQrEvent;

static RaceFsm qr;
static RaceFeedback feedback;
static RaceTaskPlan plan;
static RaceTaskGate gate;

int HostQr_Init(int zone)
{
    Race_Init(&qr);
    if (!Race_SelectZone(&qr, (uint8_t)zone)) return 0;
    memset(&feedback, 0, sizeof(feedback));
    feedback.imu_valid = 1U;
    feedback.x = 2.25f;
    feedback.y = zone == 1 ? 2.25f : 0.15f;
    if (!Race_StartQrRoute(&qr, &feedback, 1000U)) return 0;
    return qr.action_count;
}

int HostQr_Step(const char *code, HostQrEvent *output)
{
    uint8_t index;
    uint32_t first, settle;
    const RaceAction *action;
    if (output == 0 || qr.state != RACE_RUNNING ||
        qr.action_index >= qr.action_count) return 0;
    memset(output, 0, sizeof(*output));
    index = qr.action_index;
    action = &qr.actions[index];
    output->action_index = index;
    output->kind = (uint8_t)action->kind;
    output->hardware_authorized = 0U;
    feedback.x = qr.target_x;
    feedback.y = qr.target_y;
    feedback.yaw = qr.target_yaw;
    feedback.vx = feedback.vy = feedback.omega = 0.0f;
    if (action->kind == RACE_ACTION_SCAN) {
        if (code != 0 && Race_SubmitQr(&qr, qr.scan_session, code))
            output->qr_confirmed = 1U;
        else
            return 0; /* No valid QR: scan must NOT advance */
    }
    first = qr.entered_ms + 1U;
    Race_Tick(&qr, &feedback, first);
    if (qr.state == RACE_RUNNING && qr.action_index == index) {
        settle = first + action->hold_ms + 1U;
        Race_Tick(&qr, &feedback, settle);
    }
    output->state = (uint8_t)qr.state;
    output->error = (uint8_t)qr.error;
    output->qr_valid = qr.qr_valid;
    output->scan_session = qr.scan_session;
    return qr.action_index != index || qr.state == RACE_DONE;
}

int HostQr_Fail(int timeout)
{
    if (qr.state != RACE_RUNNING) return 0;
    Race_Fail(&qr, timeout ? RACE_ERROR_SCAN_TIMEOUT : RACE_ERROR_LINK);
    return 1;
}

int HostQr_State(void) { return (int)qr.state; }
int HostQr_Error(void) { return (int)qr.error; }
int HostQr_Completed(void) { return (int)qr.action_index; }
int HostQr_Session(void) { return (int)qr.scan_session; }
int HostQr_Valid(void) { return qr.qr_valid && qr.state == RACE_DONE; }

int HostTask_InitScanned(void)
{
    if (!HostQr_Valid()) return 0;
    memset(&plan, 0, sizeof(plan));
    memset(&gate, 0, sizeof(gate));
    if (!RaceTaskPlan_Build(&plan, qr.colors, qr.positions) ||
        !RaceTaskPlan_Start(&plan) ||
        !RaceTaskGate_Init(&gate, &plan)) return 0;
    return (int)plan.count;
}

int HostTask_Step(uint32_t now_ms, int fault_kind, HostTaskEvent *output)
{
    RaceTaskTestIntent intent;
    if (output == 0 || gate.state != TASK_GATE_READY) return 0;
    memset(output, 0, sizeof(*output));
    if (!RaceTaskGate_Issue(&gate, now_ms, &intent)) return 0;
    output->cmd_id = intent.cmd_id;
    output->kind = (uint8_t)intent.action.kind;
    output->batch = intent.action.batch;
    output->item = intent.action.item;
    output->color = intent.action.color;
    output->position = intent.action.position;
    output->hardware_authorized = intent.hardware_authorized;
    if (fault_kind == 1) {
        RaceTaskGate_Poll(&gate, now_ms + RACE_TASK_GATE_ACK_TIMEOUT_MS);
    } else {
        (void)RaceTaskGate_Ack(&gate, intent.cmd_id, 1U, now_ms + 1U);
        (void)RaceTaskGate_Result(&gate, intent.cmd_id,
                                  fault_kind == 2 ? 0U : 1U, now_ms + 2U);
    }
    output->state = (uint8_t)gate.state;
    output->error = (uint8_t)gate.error;
    output->completed = gate.completed;
    return 1;
}

int HostTask_State(void) { return (int)gate.state; }
int HostTask_Completed(void) { return (int)gate.completed; }
int HostTask_PlanIndex(void) { return (int)plan.index; }
int HostTask_PlanCount(void) { return (int)plan.count; }
