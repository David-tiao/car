#ifndef RACE_FSM_H
#define RACE_FSM_H

#include <stdint.h>

#define RACE_MAX_ACTIONS 24U
#define RACE_TASK_CODE_SIZE 16U

typedef enum {
    RACE_READY = 0,
    RACE_RUNNING,
    RACE_DONE,
    RACE_FAULT,
    RACE_ABORTED
} RaceState;

typedef enum {
    RACE_ACTION_MOVE = 0,
    RACE_ACTION_TURN,
    RACE_ACTION_SETTLE,
    RACE_ACTION_SCAN
} RaceActionKind;

typedef enum {
    RACE_ERROR_NONE = 0,
    RACE_ERROR_CONFIG,
    RACE_ERROR_MOVE_TIMEOUT,
    RACE_ERROR_SCAN_TIMEOUT,
    RACE_ERROR_IMU,
    RACE_ERROR_LINK,
    RACE_ERROR_LOOP_STALL,
    RACE_ERROR_SCAN_MOTION
} RaceError;

typedef struct {
    float x, y, yaw;
    float vx, vy, omega;
    uint8_t imu_valid;
} RaceFeedback;

typedef struct {
    RaceActionKind kind;
    float x, y, yaw;
    uint32_t timeout_ms;
    uint32_t hold_ms;
} RaceAction;

typedef struct {
    RaceState state;
    RaceError error;
    uint8_t zone;
    uint8_t action_index;
    uint8_t action_count;
    uint8_t holding;
    uint8_t qr_valid;
    uint8_t qr_handoff_complete;
    uint8_t qr_handoff_required;
    uint32_t entered_ms;
    uint32_t hold_since_ms;
    uint32_t scan_session;
    RaceAction actions[RACE_MAX_ACTIONS];
    float origin_x, origin_y, origin_yaw;
    float target_x, target_y, target_yaw;
    char task_code[RACE_TASK_CODE_SIZE];
    uint8_t colors[2][3];
    uint8_t positions[2][3];
} RaceFsm;

/* Legacy trusted-code API is retained for host-only simulation adapters.
 * Firmware must use Race_InitWithQrHandoff to require the UART terminal gate. */
void Race_Init(RaceFsm *race);
void Race_InitWithQrHandoff(RaceFsm *race);
uint8_t Race_SelectZone(RaceFsm *race, uint8_t zone);
uint8_t Race_RunActions(RaceFsm *race, const RaceAction *actions, uint8_t count,
                       const RaceFeedback *feedback, uint32_t now);
uint8_t Race_StartQrRoute(RaceFsm *race, const RaceFeedback *feedback, uint32_t now);
void Race_Tick(RaceFsm *race, const RaceFeedback *feedback, uint32_t now);
void Race_Abort(RaceFsm *race);
void Race_Fail(RaceFsm *race, RaceError error);
uint8_t Race_SubmitQr(RaceFsm *race, uint32_t session, const char *code);
uint8_t Race_ScanStationary(const RaceFsm *race, const RaceFeedback *feedback);
uint8_t Race_CompleteQr(RaceFsm *race, uint32_t session,
                        const RaceFeedback *feedback, uint32_t now);
uint8_t Race_ParseTask(const char *code, uint8_t colors[2][3], uint8_t positions[2][3]);

#endif
