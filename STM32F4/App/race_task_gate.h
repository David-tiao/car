#ifndef GONGCHUANG_RACE_TASK_GATE_H
#define GONGCHUANG_RACE_TASK_GATE_H

/* Pure C, hardware-free task feedback gate for the existing RaceTaskPlan.
 * Does not drive actuators, UART or ROS. Can be tested with native GCC before
 * any later STM32 integration. Never treat its intent as a motor command. */
#include <stdint.h>
#include "race_task_plan.h"

#define RACE_TASK_GATE_ACK_TIMEOUT_MS       1600U
#define RACE_TASK_GATE_RESULT_TIMEOUT_MS  120000U

typedef enum {
    TASK_GATE_READY = 0,
    TASK_GATE_WAIT_ACK,
    TASK_GATE_WAIT_RESULT,
    TASK_GATE_DONE,
    TASK_GATE_FAULT
} RaceTaskGateState;

typedef enum {
    TASK_GATE_ERROR_NONE = 0,
    TASK_GATE_ERROR_REJECTED,
    TASK_GATE_ERROR_FAILED,
    TASK_GATE_ERROR_ACK_TIMEOUT,
    TASK_GATE_ERROR_RESULT_TIMEOUT,
    TASK_GATE_ERROR_CANCELLED,
    TASK_GATE_ERROR_PLAN
} RaceTaskGateError;

typedef struct {
    uint32_t cmd_id;
    TaskAction action;
    uint8_t hardware_authorized; /* Always 0 in this test-only layer. */
} RaceTaskTestIntent;

typedef struct {
    RaceTaskPlan *plan;
    RaceTaskGateState state;
    RaceTaskGateError error;
    uint32_t cmd_id;
    uint32_t entered_ms;
    uint8_t completed;
} RaceTaskGate;

uint8_t RaceTaskGate_Init(RaceTaskGate *gate, RaceTaskPlan *plan);
uint8_t RaceTaskGate_Issue(RaceTaskGate *gate, uint32_t now_ms,
                           RaceTaskTestIntent *intent);
uint8_t RaceTaskGate_Ack(RaceTaskGate *gate, uint32_t cmd_id,
                         uint8_t accepted, uint32_t now_ms);
uint8_t RaceTaskGate_Result(RaceTaskGate *gate, uint32_t cmd_id,
                            uint8_t completed, uint32_t now_ms);
void RaceTaskGate_Poll(RaceTaskGate *gate, uint32_t now_ms);
void RaceTaskGate_Cancel(RaceTaskGate *gate);
#endif
