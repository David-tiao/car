#include "race_task_gate.h"
#include <string.h>

uint8_t RaceTaskGate_Init(RaceTaskGate *gate, RaceTaskPlan *plan)
{
    if (gate == 0) return 0U;
    memset(gate, 0, sizeof(*gate));
    if (plan == 0 || !plan->valid || plan->index != 0U ||
        plan->count == 0U || plan->count > RACE_TASK_PLAN_MAX) {
        gate->state = TASK_GATE_FAULT;
        gate->error = TASK_GATE_ERROR_PLAN;
        return 0U;
    }
    gate->plan = plan;
    gate->state = TASK_GATE_READY;
    return 1U;
}

uint8_t RaceTaskGate_Issue(RaceTaskGate *gate, uint32_t now_ms,
                           RaceTaskTestIntent *intent)
{
    const TaskAction *step;
    if (gate == 0 || intent == 0 || gate->state != TASK_GATE_READY ||
        gate->cmd_id == UINT32_MAX) return 0U;
    step = RaceTaskPlan_Current(gate->plan);
    if (step == 0) {
        gate->state = TASK_GATE_FAULT;
        gate->error = TASK_GATE_ERROR_PLAN;
        return 0U;
    }
    gate->cmd_id++;
    gate->entered_ms = now_ms;
    gate->state = TASK_GATE_WAIT_ACK;
    intent->cmd_id = gate->cmd_id;
    intent->action = *step;
    intent->hardware_authorized = 0U;
    return 1U;
}

uint8_t RaceTaskGate_Ack(RaceTaskGate *gate, uint32_t cmd_id,
                         uint8_t accepted, uint32_t now_ms)
{
    if (gate == 0 || gate->state != TASK_GATE_WAIT_ACK ||
        cmd_id != gate->cmd_id) return 0U;
    if (!accepted) {
        gate->state = TASK_GATE_FAULT;
        gate->error = TASK_GATE_ERROR_REJECTED;
        return 1U;
    }
    gate->entered_ms = now_ms;
    gate->state = TASK_GATE_WAIT_RESULT;
    return 1U;
}

uint8_t RaceTaskGate_Result(RaceTaskGate *gate, uint32_t cmd_id,
                            uint8_t completed, uint32_t now_ms)
{
    (void)now_ms;
    if (gate == 0 || gate->state != TASK_GATE_WAIT_RESULT ||
        cmd_id != gate->cmd_id) return 0U;
    if (!completed) {
        gate->state = TASK_GATE_FAULT;
        gate->error = TASK_GATE_ERROR_FAILED;
        return 1U;
    }
    if (!RaceTaskPlan_Advance(gate->plan)) {
        gate->state = TASK_GATE_FAULT;
        gate->error = TASK_GATE_ERROR_PLAN;
        return 0U;
    }
    gate->completed++;
    gate->state = RaceTaskPlan_Done(gate->plan) ? TASK_GATE_DONE : TASK_GATE_READY;
    return 1U;
}

void RaceTaskGate_Poll(RaceTaskGate *gate, uint32_t now_ms)
{
    if (gate == 0) return;
    if (gate->state == TASK_GATE_WAIT_ACK &&
        (uint32_t)(now_ms - gate->entered_ms) >= RACE_TASK_GATE_ACK_TIMEOUT_MS) {
        gate->state = TASK_GATE_FAULT;
        gate->error = TASK_GATE_ERROR_ACK_TIMEOUT;
    } else if (gate->state == TASK_GATE_WAIT_RESULT &&
               (uint32_t)(now_ms - gate->entered_ms) >= RACE_TASK_GATE_RESULT_TIMEOUT_MS) {
        gate->state = TASK_GATE_FAULT;
        gate->error = TASK_GATE_ERROR_RESULT_TIMEOUT;
    }
}

void RaceTaskGate_Cancel(RaceTaskGate *gate)
{
    if (gate == 0 || gate->state == TASK_GATE_DONE ||
        gate->state == TASK_GATE_FAULT) return;
    gate->state = TASK_GATE_FAULT;
    gate->error = TASK_GATE_ERROR_CANCELLED;
}
