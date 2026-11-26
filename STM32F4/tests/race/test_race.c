#include "race_fsm.h"
#include "race_config.h"
#include "vision.h"
#include "race_task_plan.h"
#include <assert.h>
#include <math.h>
#include <stdio.h>
#include <string.h>

UART_HandleTypeDef huart3 = {USART3};
static uint32_t s_irq;
uint32_t __get_PRIMASK(void) { return s_irq; }
void __disable_irq(void) { s_irq = 1U; }
void __set_PRIMASK(uint32_t mask) { s_irq = mask; }
uint32_t HAL_GetTick(void) { return 0U; }
int HAL_UART_Receive_IT(UART_HandleTypeDef *uart, uint8_t *byte, uint16_t length)
{ (void)uart; (void)byte; (void)length; return HAL_OK; }
int HAL_UART_Transmit_IT(UART_HandleTypeDef *uart, uint8_t *byte, uint16_t length)
{ (void)uart; (void)byte; (void)length; return HAL_OK; }
void Mecanum_Stop(MecanumChassis_t *chassis) { (void)chassis; }
void Mecanum_SetPositionMode(MecanumChassis_t *chassis, float x, float y, float yaw)
{ (void)chassis; (void)x; (void)y; (void)yaw; }
void Mecanum_GetPosition(MecanumChassis_t *chassis, float *x, float *y, float *theta)
{ (void)chassis; if (x) *x = 0.0f; if (y) *y = 0.0f; if (theta) *theta = 0.0f; }
void Mecanum_GetVelocity(MecanumChassis_t *chassis, float *vx, float *vy, float *omega)
{ (void)chassis; if (vx) *vx = 0.0f; if (vy) *vy = 0.0f; if (omega) *omega = 0.0f; }
uint8_t Mecanum_IsTargetReached(MecanumChassis_t *chassis) { (void)chassis; return 1U; }

static RaceFeedback stationary(void)
{
    RaceFeedback value = {0};
    value.imu_valid = 1U;
    return value;
}

static void at_target(const RaceFsm *race, RaceFeedback *current)
{
    current->x = race->target_x;
    current->y = race->target_y;
    current->yaw = race->target_yaw;
    current->vx = current->vy = current->omega = 0.0f;
}

static void test_routes(void)
{
    uint8_t zone;
    for (zone = 1U; zone <= 2U; zone++) {
        RaceFsm race;
        RaceFeedback current = stationary();
        uint32_t now = 50U;
        Race_InitWithQrHandoff(&race);
        assert(Race_SelectZone(&race, zone));
        assert(Race_StartQrRoute(&race, &current, now));
        assert(fabsf(race.target_y - 0.125f) < 0.0001f);
        assert(fabsf(race.target_x) < 0.0001f);
        assert(fabsf(race.actions[1].x - (zone == 1U ? -1.05f : 1.05f)) < 0.0001f);
        assert(!Race_SelectZone(&race, 3U - zone));
        assert(!Race_SubmitQr(&race, race.scan_session, "452+321+254+312"));
        while (race.action_index < 4U) {
            uint8_t previous = race.action_index;
            at_target(&race, &current);
            Race_Tick(&race, &current, now);
            assert(race.action_index == previous);
            now += race.actions[previous].hold_ms;
            Race_Tick(&race, &current, now);
            assert(race.action_index == previous + 1U);
            now++;
        }
        assert(race.state == RACE_RUNNING);
        assert(race.scan_session != 0U);
        assert(!Race_SubmitQr(&race, race.scan_session + 1U, "452+321+254+312"));
        assert(!Race_SubmitQr(&race, race.scan_session, "452+321+253+312"));
        assert(Race_SubmitQr(&race, race.scan_session, "452+321+254+312"));
        assert(!Race_SubmitQr(&race, race.scan_session, "452+321+254+312"));
        assert(race.colors[1][0] == 2U && race.positions[0][0] == 3U);
        assert(race.action_index == 4U);
        current.vx = 0.1f;
        Race_Tick(&race, &current, now);
        assert(race.state == RACE_RUNNING);
        assert(!Race_CompleteQr(&race, race.scan_session, &current, now));
        current.vx = 0.0f;
        Race_Tick(&race, &current, now);
        assert(race.action_index == 4U);
        assert(!Race_CompleteQr(&race, race.scan_session + 1U, &current, now));
        assert(Race_CompleteQr(&race, race.scan_session, &current, now));
        assert(!Race_CompleteQr(&race, race.scan_session, &current, now));
        Race_Tick(&race, &current, now + 1U);
        assert(race.action_index == 5U && race.state == RACE_RUNNING);
        while (race.action_index < 7U) {
            uint8_t previous = race.action_index;
            at_target(&race, &current);
            Race_Tick(&race, &current, now + 2U);
            now += race.actions[previous].hold_ms;
            Race_Tick(&race, &current, now + 2U);
            assert(race.action_index == previous + 1U);
            now += 2U;
        }
        at_target(&race, &current);
        Race_Tick(&race, &current, now + 2U);
        now += race.actions[7].hold_ms;
        Race_Tick(&race, &current, now + 2U);
        assert(race.state == RACE_DONE);
        assert(!Race_StartQrRoute(&race, &current, now + 2U));
        current.imu_valid = 0U;
        Race_Tick(&race, &current, now + 100000U);
        assert(race.state == RACE_DONE);
    }
}

static void test_pending_qr_timeout(void)
{
    RaceFsm race;
    RaceFeedback current = stationary();
    RaceAction action = {RACE_ACTION_SCAN, 0.0f, 0.0f, 0.0f, 1000U, 0U};
    Race_InitWithQrHandoff(&race);
    assert(Race_RunActions(&race, &action, 1U, &current, 0U));
    assert(!Race_CompleteQr(&race, race.scan_session, &current, 1U));
    assert(Race_SubmitQr(&race, race.scan_session, "452+321+254+312"));
    Race_Tick(&race, &current, 999U);
    assert(race.state == RACE_RUNNING && race.action_index == 0U);
    assert(!Race_CompleteQr(&race, race.scan_session, &current, 1000U));
    Race_Tick(&race, &current, 1000U);
    assert(race.error == RACE_ERROR_SCAN_TIMEOUT && race.state == RACE_FAULT);
    assert(!Race_CompleteQr(&race, race.scan_session, &current, 1001U));
    /* The pre-existing host adapter injects already-trusted code, not UART. */
    Race_Init(&race);
    assert(Race_RunActions(&race, &action, 1U, &current, 0U));
    assert(Race_SubmitQr(&race, race.scan_session, "452+321+254+312"));
    Race_Tick(&race, &current, 1U);
    assert(race.state == RACE_DONE);
}

static void test_motion_guards(void)
{
    RaceFsm race;
    RaceFeedback current = stationary();
    RaceAction action = {RACE_ACTION_MOVE, 1.0f, 0.0f, 0.0f, 1000U, 200U};
    Race_Init(&race);
    assert(Race_RunActions(&race, &action, 1U, &current, 0U));
    at_target(&race, &current);
    current.vx = 0.1f;
    Race_Tick(&race, &current, 100U);
    assert(!race.holding);
    current.vx = 0.0f;
    Race_Tick(&race, &current, 200U);
    current.y = 0.1f;
    Race_Tick(&race, &current, 300U);
    assert(!race.holding);
    current.y = 0.0f;
    Race_Tick(&race, &current, 400U);
    Race_Tick(&race, &current, 599U);
    assert(race.state == RACE_RUNNING);
    Race_Tick(&race, &current, 600U);
    assert(race.state == RACE_DONE);
    Race_Init(&race);
    current = stationary();
    current.x = 2.0f;
    current.y = 3.0f;
    current.yaw = 1.57079632679f;
    assert(Race_RunActions(&race, &action, 1U, &current, 0U));
    assert(fabsf(race.target_x - 2.0f) < 0.0001f);
    assert(fabsf(race.target_y - 4.0f) < 0.0001f);
    Race_Abort(&race);
    Race_Tick(&race, &current, 2000U);
    assert(race.state == RACE_ABORTED);
    assert(!Race_StartQrRoute(&race, &current, 2000U));
}

static void test_faults(void)
{
    RaceFsm race;
    RaceFeedback current = stationary();
    RaceAction action = {RACE_ACTION_SCAN, 0.0f, 0.0f, 0.0f, 1000U, 0U};
    uint32_t start = 0xFFFFFF00U;
    Race_Init(&race);
    assert(Race_RunActions(&race, &action, 1U, &current, start));
    Race_Tick(&race, &current, start + 999U);
    assert(race.state == RACE_RUNNING);
    Race_Tick(&race, &current, start + 1000U);
    assert(race.error == RACE_ERROR_SCAN_TIMEOUT);
    Race_Init(&race);
    action.kind = RACE_ACTION_MOVE;
    action.x = 1.0f;
    assert(Race_RunActions(&race, &action, 1U, &current, 0U));
    Race_Tick(&race, &current, 1000U);
    assert(race.error == RACE_ERROR_MOVE_TIMEOUT);
    Race_Init(&race);
    assert(Race_StartQrRoute(&race, &current, 0U));
    current.imu_valid = 0U;
    Race_Tick(&race, &current, 1U);
    assert(race.error == RACE_ERROR_IMU);
    Race_Init(&race);
    current = stationary();
    current.x = NAN;
    assert(!Race_StartQrRoute(&race, &current, 0U));
    assert(race.error == RACE_ERROR_IMU);
    Race_Init(&race);
    current = stationary();
    current.vx = 0.1f;
    assert(!Race_StartQrRoute(&race, &current, 0U));
    assert(race.state == RACE_READY);
    current.vx = 0.0f;
    action.hold_ms = action.timeout_ms;
    assert(!Race_RunActions(&race, &action, 1U, &current, 0U));
    assert(race.error == RACE_ERROR_CONFIG);
}

static void test_protocol(void)
{
    static const char *const invalid[] = {
        "#SEQ,3,2,1,2,1,3,3,1,2",
        "#SEQ,452,321,254,312",
        "#SEQ,4,5,2,3,2,1,2,5,4,3,1,2,1",
        "#SEQ,4,5,2,3,2,1,2,5,4,3,1,",
        "#SEQ,4,5,2,3,2,1,2,5,4,3,1,2junk",
        "#SEQ,4,5,2,3,2,1,2,5,4,3,1,22",
        "#SEQ,4,5,2,3,2,1,2,5,4,3,1,-2",
        "#SEQ,4,5,2,3,2,1,2,5,4,3,1,0",
        "#SEQ,4,5,2,3,2,1,2,5,4,3,1,6",
        "#SEQ,4,5,2,3,2,1,2,5,3,3,1,2",
        "#SEQ,4,4,2,3,2,1,2,4,4,3,1,2",
        "#DONE", "#OBJ,1,160,120,40,35,1400,215", ""
    };
    const char *valid = "#SEQ,4,5,2,3,2,1,2,5,4,3,1,2\r\n";
    char code[RACE_TASK_CODE_SIZE] = "unchanged";
    RaceVisionDecoder decoder;
    unsigned index, accepted = 0U;
    for (index = 0U; index < sizeof(invalid) / sizeof(invalid[0]); index++) {
        assert(!RaceVision_ParseSeq(invalid[index], code));
        assert(strcmp(code, "unchanged") == 0);
    }
    assert(RaceVision_ParseSeq("#SEQ, 4,5,2,3,2,1,2,5,4,3,1,2 ", code));
    assert(strcmp(code, "452+321+254+312") == 0);
    RaceVisionDecoder_Reset(&decoder);
    for (index = 0U; valid[index]; index++) accepted += RaceVisionDecoder_Push(&decoder, valid[index], code);
    assert(accepted == 1U);
    RaceVisionDecoder_Reset(&decoder);
    assert(!RaceVisionDecoder_Push(&decoder, '#', code));
    for (index = 0U; index < 200U; index++) assert(!RaceVisionDecoder_Push(&decoder, 'X', code));
    for (index = 0U; valid[index]; index++) assert(!RaceVisionDecoder_Push(&decoder, valid[index], code));
    accepted = 0U;
    for (index = 0U; valid[index]; index++) accepted += RaceVisionDecoder_Push(&decoder, valid[index], code);
    assert(accepted == 1U);
    RaceVisionDecoder_Reset(&decoder);
    assert(!RaceVisionDecoder_Push(&decoder, '#', code));
    assert(!RaceVisionDecoder_Push(&decoder, 0U, code));
    for (index = 0U; valid[index]; index++) assert(!RaceVisionDecoder_Push(&decoder, valid[index], code));
}

static void test_task_plan(void)
{
    static const uint8_t colors[2][3] = {{4U, 5U, 2U}, {2U, 5U, 4U}};
    static const uint8_t positions[2][3] = {{3U, 2U, 1U}, {2U, 1U, 3U}};
    RaceTaskPlan plan;
    const TaskAction *action;
    uint8_t pick_count = 0U;
    assert(RaceTaskPlan_Build(&plan, colors, positions));
    assert(plan.count == 34U);
    assert(RaceTaskPlan_Start(&plan));
    action = RaceTaskPlan_Current(&plan);
    assert(action->kind == TASK_ACTION_PICK && action->batch == 0U &&
           action->color == 4U);
    while (!RaceTaskPlan_Done(&plan)) {
        action = RaceTaskPlan_Current(&plan);
        if (action->kind == TASK_ACTION_PICK) pick_count++;
        if (action->kind == TASK_ACTION_STACK_STORAGE) {
            if (action->color == 2U) assert(action->position == 1U);
            if (action->color == 5U) assert(action->position == 2U);
            if (action->color == 4U) assert(action->position == 3U);
        }
        assert(RaceTaskPlan_Advance(&plan));
    }
    assert(pick_count == 6U && RaceTaskPlan_Current(&plan) == 0);
}


/* Test the production Hardware/vision.c parser, not the web contract model. */
static char v4_tx[192];
static void capture_v4_tx(const uint8_t *data, size_t size, void *user)
{
    (void)user;
    assert(size < sizeof(v4_tx));
    memcpy(v4_tx, data, size);
    v4_tx[size] = '\0';
}
static void inject_v4(VisionContext *vision, unsigned seq, const char *body,
                      unsigned now_ms, int corrupt)
{
    char header[160], wire[192];
    unsigned char xor_value = 0U;
    int length, size;
    unsigned i;
    length = snprintf(header, sizeof(header), "%u,%u,%s",
                      seq, (unsigned)strlen(body), body);
    assert(length > 0 && (size_t)length < sizeof(header));
    for (i = 0U; i < (unsigned)length; ++i)
        xor_value ^= (unsigned char)header[i];
    if (corrupt) xor_value ^= 1U;
    size = snprintf(wire, sizeof(wire), "@%s*%02X\n", header, (unsigned)xor_value);
    assert(size > 0 && (size_t)size < sizeof(wire));
    for (i = 0U; i < (unsigned)size; ++i)
        Vision_RxByte(vision, (uint8_t)wire[i], now_ms);
}
static void test_v4_qr_session_contract(void)
{
    VisionContext vision;
    char code[VISION_TASK_CODE_SIZE];
    static const char *legacy = "#SEQ,1,5,6,1,2,3,5,1,6,2,3,1\n";
    unsigned i;

    Vision_Init(&vision, capture_v4_tx, NULL, NULL);
    vision.ready = 1U;
    assert(!Vision_StartQrJob(&vision, 65536U, 0U));
    assert(Vision_StartQrJob(&vision, 23U, 0U));
    assert(strstr(v4_tx, "JOB,1,23*") != NULL);
    assert(vision.qr_session == 23U);
    assert(vision.phase == VISION_WAIT_ACK);
    inject_v4(&vision, 0U, "ACK,JOB,1", 10U, 0);
    assert(vision.phase == VISION_RUNNING);

    /* A tagged V4 session must never accept the old no-session syntax. */
    for (i = 0U; legacy[i] != '\0'; ++i)
        Vision_RxByte(&vision, (uint8_t)legacy[i], 20U);
    assert(!Vision_TakeQr(&vision, code));
    inject_v4(&vision, 1U, "QR,156+123+516+231", 20U, 0);
    assert(!Vision_TakeQr(&vision, code));
    inject_v4(&vision, 2U, "QR,156+123+516+231,22", 20U, 0);
    assert(!Vision_TakeQr(&vision, code));
    inject_v4(&vision, 3U, "QR,156+123+516+231,23", 20U, 1);
    assert(!Vision_TakeQr(&vision, code));
    assert(vision.frames_bad >= 1U);
    inject_v4(&vision, 4U, "QR,156+112+516+231,23", 20U, 0);
    assert(!Vision_TakeQr(&vision, code));
    inject_v4(&vision, 5U, "QR,156+123+516+231,23", 20U, 0);
    assert(Vision_TakeQr(&vision, code));
    assert(strcmp(code, "156+123+516+231") == 0);
    inject_v4(&vision, 5U, "QR,156+123+516+231,23", 20U, 0);
    assert(!Vision_TakeQr(&vision, code));
    assert(!Vision_QrHandoffComplete(&vision, 23U));
    assert(!Vision_AckQr(&vision, 22U));
    assert(Vision_AckQr(&vision, 23U));
    assert(strstr(v4_tx, "ACK,QR,23*") != NULL);
    inject_v4(&vision, 6U, "DONE,QR", 20U, 0);
    assert(vision.result == VISION_RESULT_QR_DONE);
    assert(!Vision_QrHandoffComplete(&vision, 23U));
    inject_v4(&vision, 7U, "IDLE", 20U, 0);
    assert(vision.phase == VISION_IDLE);
    assert(Vision_QrHandoffComplete(&vision, 23U));
    assert(!Vision_QrHandoffComplete(&vision, 24U));
    assert(Vision_StartQrJob(&vision, 24U, 30U));
    assert(vision.qr_session == 24U);
    /* Stale QR from scan 23 cannot activate the new scan. */
    inject_v4(&vision, 8U, "ACK,JOB,1", 31U, 0);
    inject_v4(&vision, 9U, "QR,156+123+516+231,23", 32U, 0);
    assert(!Vision_TakeQr(&vision, code));
    inject_v4(&vision, 10U, "QR,156+123+516+231,24", 32U, 0);
    assert(Vision_TakeQr(&vision, code));
}

static void test_qr_terminal_guards(void)
{
    VisionContext v;
    unsigned scenario;
    for (scenario = 0U; scenario < 6U; scenario++) {
        Vision_Init(&v, capture_v4_tx, NULL, NULL);
        v.ready = 1U;
        assert(Vision_StartQrJob(&v, 23U, 0U));
        inject_v4(&v, 10U, "ACK,JOB,1", 1U, 0);
        assert(!Vision_AckQr(&v, 23U));
        inject_v4(&v, 11U, "QR,156+123+516+231,23", 2U, 0);
        if (scenario != 0U) assert(Vision_AckQr(&v, 23U));
        if (scenario == 0U) inject_v4(&v, 12U, "DONE,QR", 3U, 0);
        if (scenario == 1U) inject_v4(&v, 12U, "IDLE", 3U, 0);
        if (scenario == 2U) {
            inject_v4(&v, 12U, "DONE,QR", 3U, 0);
            inject_v4(&v, 14U, "IDLE", 4U, 0);
        }
        if (scenario == 3U) inject_v4(&v, 0U, "READY,IDLE", 3U, 0);
        if (scenario == 4U) assert(Vision_Cancel(&v));
        if (scenario == 5U) assert(Vision_Reset(&v));
        assert(!Vision_QrHandoffComplete(&v, 23U));
        assert(v.qr_stage == VISION_QR_FAILED);
        inject_v4(&v, 15U, "DONE,QR", 5U, 0);
        inject_v4(&v, 16U, "IDLE", 6U, 0);
        assert(!Vision_QrHandoffComplete(&v, 23U));
    }
}

int main(void)
{
    test_qr_terminal_guards();
    test_v4_qr_session_contract();
    test_routes();
    test_pending_qr_timeout();
    test_motion_guards();
    test_faults();
    test_protocol();
    test_task_plan();
    puts("PASS: both routes, pose transform, dwell/speed, abort/latch, time wrap, faults, strict protocol, stream recovery");
    return 0;
}
