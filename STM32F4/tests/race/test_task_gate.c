#include "race_task_gate.h"
#include <assert.h>
#include <stdio.h>

static const uint8_t COLORS[2][3] = {{1U, 5U, 6U}, {5U, 1U, 6U}};
static const uint8_t RINGS[2][3] = {{1U, 2U, 3U}, {2U, 3U, 1U}};

static void init_plan(RaceTaskPlan *plan, RaceTaskGate *gate)
{
    assert(RaceTaskPlan_Build(plan, COLORS, RINGS));
    assert(plan->count == 30U);
    assert(RaceTaskPlan_Start(plan));
    assert(RaceTaskGate_Init(gate, plan));
}

static void test_all_actions_and_correlated_feedback(void)
{
    RaceTaskPlan plan;
    RaceTaskGate gate;
    RaceTaskTestIntent intent;
    uint32_t now = 50U;
    unsigned picks = 0U, rough_placements = 0U, retrieves = 0U;
    unsigned stacks = 0U, storage = 0U, returns = 0U;
    init_plan(&plan, &gate);
    while (gate.state != TASK_GATE_DONE) {
        assert(gate.state == TASK_GATE_READY);
        assert(RaceTaskGate_Issue(&gate, now, &intent));
        assert(intent.hardware_authorized == 0U);
        assert(intent.cmd_id == gate.completed + 1U);
        assert(!RaceTaskGate_Result(&gate, intent.cmd_id, 1U, now + 1U));
        assert(!RaceTaskGate_Ack(&gate, intent.cmd_id + 100U, 1U, now + 2U));
        assert(gate.state == TASK_GATE_WAIT_ACK);
        assert(RaceTaskGate_Ack(&gate, intent.cmd_id, 1U, now + 3U));
        assert(!RaceTaskGate_Ack(&gate, intent.cmd_id, 1U, now + 4U));
        assert(!RaceTaskGate_Result(&gate, intent.cmd_id + 100U, 1U, now + 5U));

        switch (intent.action.kind) {
        case TASK_ACTION_PICK: picks++; break;
        case TASK_ACTION_PLACE_ROUGH: rough_placements++; break;
        case TASK_ACTION_RETRIEVE_ROUGH: retrieves++; break;
        case TASK_ACTION_PLACE_STORAGE: storage++; break;
        case TASK_ACTION_STACK_STORAGE: {
            unsigned i;
            unsigned matched = 0U;
            stacks++;
            for (i = 0U; i < 3U; i++) {
                if (COLORS[0][i] == intent.action.color) {
                    assert(RINGS[0][i] == intent.action.position);
                    matched = 1U;
                }
            }
            assert(matched);
            break;
        }
        case TASK_ACTION_RETURN_START: returns++; break;
        default: break;
        }
        assert(RaceTaskGate_Result(&gate, intent.cmd_id, 1U, now + 6U));
        assert(!RaceTaskGate_Result(&gate, intent.cmd_id, 1U, now + 7U));
        now += 10U;
    }
    assert(picks == 6U && rough_placements == 6U && retrieves == 6U);
    assert(storage == 3U && stacks == 3U && returns == 1U);
    assert(gate.completed == 30U);
    assert(RaceTaskPlan_Done(&plan));
    assert(!RaceTaskGate_Issue(&gate, now, &intent));
    assert(gate.error == TASK_GATE_ERROR_NONE);
}

static void test_fault_and_timeout_guards(void)
{
    RaceTaskPlan plan;
    RaceTaskGate gate;
    RaceTaskTestIntent intent;
    uint32_t now;

    init_plan(&plan, &gate);
    assert(RaceTaskGate_Issue(&gate, 100U, &intent));
    assert(!RaceTaskGate_Ack(&gate, intent.cmd_id + 1U, 1U, 101U));
    RaceTaskGate_Poll(&gate, 100U + RACE_TASK_GATE_ACK_TIMEOUT_MS);
    assert(gate.state == TASK_GATE_FAULT);
    assert(gate.error == TASK_GATE_ERROR_ACK_TIMEOUT);
    assert(gate.completed == 0U && plan.index == 0U);
    assert(!RaceTaskGate_Issue(&gate, 200U, &intent));

    init_plan(&plan, &gate);
    assert(RaceTaskGate_Issue(&gate, 500U, &intent));
    assert(RaceTaskGate_Ack(&gate, intent.cmd_id, 1U, 501U));
    RaceTaskGate_Poll(&gate, 501U + RACE_TASK_GATE_RESULT_TIMEOUT_MS);
    assert(gate.state == TASK_GATE_FAULT);
    assert(gate.error == TASK_GATE_ERROR_RESULT_TIMEOUT);
    assert(plan.index == 0U);

    init_plan(&plan, &gate);
    assert(RaceTaskGate_Issue(&gate, 1U, &intent));
    assert(RaceTaskGate_Ack(&gate, intent.cmd_id, 1U, 2U));
    assert(RaceTaskGate_Result(&gate, intent.cmd_id, 0U, 3U));
    assert(gate.state == TASK_GATE_FAULT);
    assert(gate.error == TASK_GATE_ERROR_FAILED);
    assert(gate.completed == 0U && plan.index == 0U);

    init_plan(&plan, &gate);
    assert(RaceTaskGate_Issue(&gate, 1U, &intent));
    assert(RaceTaskGate_Ack(&gate, intent.cmd_id, 0U, 2U));
    assert(gate.error == TASK_GATE_ERROR_REJECTED);
    assert(plan.index == 0U);

    init_plan(&plan, &gate);
    assert(RaceTaskGate_Issue(&gate, 1U, &intent));
    RaceTaskGate_Cancel(&gate);
    assert(gate.state == TASK_GATE_FAULT);
    assert(gate.error == TASK_GATE_ERROR_CANCELLED);
    assert(!RaceTaskGate_Result(&gate, intent.cmd_id, 1U, 2U));

    /* unsigned tick wrap remains safe for bounded timeout intervals */
    init_plan(&plan, &gate);
    now = UINT32_MAX - 20U;
    assert(RaceTaskGate_Issue(&gate, now, &intent));
    RaceTaskGate_Poll(&gate, now + RACE_TASK_GATE_ACK_TIMEOUT_MS);
    assert(gate.error == TASK_GATE_ERROR_ACK_TIMEOUT);
}

static void test_invalid_plan_rejected(void)
{
    RaceTaskPlan plan = {0};
    RaceTaskGate gate;
    RaceTaskTestIntent intent;
    assert(!RaceTaskGate_Init(&gate, &plan));
    assert(gate.state == TASK_GATE_FAULT);
    assert(!RaceTaskGate_Issue(&gate, 0U, &intent));
}

int main(void)
{
    test_all_actions_and_correlated_feedback();
    test_fault_and_timeout_guards();
    test_invalid_plan_rejected();
    puts("PASS: real STM32 C RaceTaskPlan 30 actions, paired ACK/result, dedup, faults and same-color stacking");
    return 0;
}
