/* HOST-ONLY adapter for testing the actual STM32 RaceTaskPlan / RaceTaskGate C.
 * Compile as shared library on Linux; never link HAL, USART, motor or servo.
 */
#include "race_task_gate.h"
#include <stdint.h>
#include <string.h>

typedef struct {
    uint32_t cmd_id;
    uint8_t kind, batch, item, color, position;
    uint8_t state, error, completed;
    uint8_t hardware_authorized;
} HostTaskEvent;

static RaceTaskPlan plan;
static RaceTaskGate gate;

int HostTask_Init(const uint8_t *colors, const uint8_t *rings)
{
    uint8_t cc[2][3], rr[2][3];
    unsigned i;
    if (colors == 0 || rings == 0) return 0;
    memset(&plan, 0, sizeof(plan));
    memset(&gate, 0, sizeof(gate));
    for (i = 0; i < 6U; ++i) {
        cc[i / 3U][i % 3U] = colors[i];
        rr[i / 3U][i % 3U] = rings[i];
    }
    if (!RaceTaskPlan_Build(&plan, cc, rr) ||
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
        /* The requested ACK was never received. */
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
