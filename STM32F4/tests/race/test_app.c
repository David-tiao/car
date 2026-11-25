#include "board.h"
#include "app_race.h"
#include "app_ring_align.h"
#include "app_stepper_openloop.h"
#include "race_config.h"
#include "vision.h"
#include <assert.h>
#include <stdio.h>
#include <string.h>
#include <stdlib.h>

uint8_t OLED_GRAM[128][8];
static Motor_t s_motors[4];
static MecanumChassis_t s_chassis;
static uint32_t s_now, s_frames, s_irq, s_position_calls, s_stops, s_scans;
static uint8_t s_key_down, s_imu_ok = 1U, s_bias_ready = 1U, s_link_ok = 1U, s_qr;
static VisionContext s_vision;

uint32_t __get_PRIMASK(void) { return s_irq; }
void __disable_irq(void) { s_irq = 1U; }
void __set_PRIMASK(uint32_t mask) { s_irq = mask; }
uint32_t HAL_GetTick(void) { return s_now; }
int HAL_GPIO_ReadPin(void *port, uint16_t pin) { (void)port; (void)pin; return !s_key_down; }
bool app_imu_ok(void) { return s_imu_ok; }
bool app_imu_bias_ready(void) { return s_bias_ready; }
uint32_t app_imu_frames(void) { return s_frames; }
void Motor_Stop(Motor_t *motor) { motor->output = 0.0f; s_stops++; }
void Mecanum_Stop(MecanumChassis_t *chassis) { chassis->mode = 0U; }
void Mecanum_SetSpeedLimit(MecanumChassis_t *chassis, float linear, float angular)
{ chassis->max_speed = linear; chassis->max_omega = angular; }
void Mecanum_SetTolerance(MecanumChassis_t *chassis, float position, float yaw)
{ chassis->position_tol = position; chassis->theta_tol = yaw; }
void Mecanum_SetPositionMode(MecanumChassis_t *chassis, float x, float y, float yaw)
{
    assert(s_irq == 1U && chassis->body_frame == 0U);
    chassis->x_target = x;
    chassis->y_target = y;
    chassis->theta_target = yaw;
    chassis->mode = 1U;
    s_position_calls++;
}
void OLED_ShowChar(uint8_t x, uint8_t y, uint8_t chr, uint8_t size, uint8_t mode)
{ (void)x; (void)y; (void)chr; (void)size; (void)mode; }
void OLED_WR_Byte(uint8_t value, uint8_t command) { (void)value; (void)command; }
uint8_t RaceVision_Init(void) { return s_link_ok; }
uint8_t RaceVision_Healthy(void) { return s_link_ok; }
uint8_t RaceVision_BeginScan(void) { s_scans++; return s_link_ok; }
uint8_t RaceVision_BeginScanSession(uint32_t session)
{ assert(session != 0U); return RaceVision_BeginScan(); }
uint8_t RaceVision_AckQr(uint32_t session)
{ return session != 0U && s_link_ok; }
uint8_t RaceVision_ScanFailed(void) { return 0U; }
void RaceVision_EndScan(void) { s_qr = 0U; }
uint8_t RaceVision_Poll(char code[RACE_TASK_CODE_SIZE])
{
    if (!s_qr) return 0U;
    s_qr = 0U;
    memcpy(code, "452+321+254+312", RACE_TASK_CODE_SIZE);
    return 1U;
}
VisionContext *Vision_GetDefault(void) { return &s_vision; }
void Vision_SetStop(VisionContext *vision, VisionStop stop, void *user)
{ (void)vision; (void)stop; (void)user; }
uint8_t Vision_RingActive(const VisionContext *vision) { (void)vision; return 0U; }
uint8_t Vision_RingFailed(const VisionContext *vision) { (void)vision; return 0U; }
uint8_t Vision_RingSucceeded(const VisionContext *vision) { (void)vision; return 0U; }
int Vision_StartRingAlign(VisionContext *vision, uint8_t target, uint32_t now)
{ (void)vision; (void)target; (void)now; return 0; }
void AppRingAlign_Init(AppRingAlign *align, VisionContext *vision,
                       MecanumChassis_t *chassis, const RingAlignConfig *config)
{
    (void)chassis; (void)config;
    memset(align, 0, sizeof(*align));
    align->vision = vision;
#ifdef TEST_TASK_MISSION
    align->calibration_valid = 1U;
#endif
}
uint8_t AppRingAlign_Start(AppRingAlign *align, uint8_t target, uint32_t now)
{ assert(target >= 1U && target <= 3U); (void)now; align->succeeded = 0U; return 1U; }
void AppRingAlign_Poll(AppRingAlign *align, uint32_t now)
{ align->succeeded = 1U; (void)now; }
void AppRingAlign_Cancel(AppRingAlign *align) { (void)align; }
uint8_t AppRingAlign_IsActive(const AppRingAlign *align)
{ (void)align; return 0U; }
uint8_t AppRingAlign_Succeeded(const AppRingAlign *align)
{ return align->succeeded; }
uint8_t AppRingAlign_Failed(const AppRingAlign *align)
{ (void)align; return 0U; }
int Vision_Cancel(VisionContext *vision) { (void)vision; return 0; }
int Vision_RequestCheck(VisionContext *v, uint16_t session, uint8_t step,
                        char phase, uint8_t color, uint8_t ring)
{
    const TaskAction *action = RaceTaskPlan_Current(AppRace_TaskPlan());
    assert(action && color == action->color);
    assert(ring == (action->kind == TASK_ACTION_RETURN_START ?
                    AppRace_Status()->zone : action->position));
    assert(session == AppRace_Status()->scan_session);
    v->check_step = step;
    v->check_phase = phase;
    v->check_received = 0U;
    return 1;
}
float Servo_GetAngle(uint8_t id) { (void)id; return 0.0f; }
void Servo_SetAngleImmediate(uint8_t id, float angle) { (void)id; (void)angle; }
void Servo_Init(void) {}
void Servo_SetAngle(uint8_t id, float angle) { (void)id; (void)angle; }
void Servo_SetAngleRelativeLong(uint8_t id, float angle) { (void)id; (void)angle; }
void Servo_SetProfile(uint8_t id, float speed, float accel)
{ (void)id; (void)speed; (void)accel; }
void Servo_SetSpeedLimit(float speed) { (void)speed; }
void Servo_SetAccel(float accel) { (void)accel; }
void Servo_Disable(uint8_t id) { (void)id; }
void Servo_Poll(void) {}
uint8_t Servo_IsBusy(uint8_t id) { (void)id; return 0U; }
void AppStepper_Init(void) {}
HAL_StatusTypeDef AppStepper_MoveSteps(StepperAxis_t axis, int32_t steps, uint32_t frequency)
{ (void)axis; (void)steps; (void)frequency; return 0; }
HAL_StatusTypeDef AppStepper_MoveStepsProfile(StepperAxis_t axis, int32_t steps,
                                              uint32_t frequency, float accel)
{ (void)axis; (void)steps; (void)frequency; (void)accel; return 0; }
void AppStepper_Poll(void) {}
void AppStepper_Stop(StepperAxis_t axis) { (void)axis; }
void AppStepper_SetBrakeMode(StepperAxis_t axis, uint8_t trap_brake)
{ (void)axis; (void)trap_brake; }
uint8_t AppStepper_IsBusy(StepperAxis_t axis) { (void)axis; return 0U; }
uint32_t AppStepper_GetRemainingSteps(StepperAxis_t axis)
{ (void)axis; return 0U; }

static void tick(uint32_t duration)
{
    while (duration--) {
        s_now++;
        if (s_imu_ok) s_frames++;
        (void)AppRace_ControlTick();
        AppRace_Poll();
    }
}

static void init(uint8_t held)
{
    memset(&s_chassis, 0, sizeof(s_chassis));
    s_chassis.motor_lf = &s_motors[0];
    s_chassis.motor_rf = &s_motors[1];
    s_chassis.motor_lb = &s_motors[2];
    s_chassis.motor_rb = &s_motors[3];
    s_chassis.body_frame = 1U;
    s_key_down = held;
    s_now = s_frames = s_irq = s_position_calls = s_scans = 0U;
    s_bias_ready = s_imu_ok = s_link_ok = 1U;
    AppRace_Init(&s_chassis);
}

static void click(void)
{
    s_key_down = 1U;
    tick(40U);
    s_key_down = 0U;
    tick(40U);
}

static void start(void)
{
    init(0U);
    tick(RACE_WARMUP_MS + 1U);
    click();
    assert(AppRace_Status()->state == RACE_RUNNING);
}

int main(void)
{
    uint32_t calls, stops;
    init(1U);
    tick(RACE_WARMUP_MS + 1U);
    s_key_down = 0U;
    tick(40U);
    assert(AppRace_Status()->state == RACE_READY && s_position_calls == 0U);
    s_key_down = 1U;
    tick(750U);
    s_key_down = 0U;
    tick(40U);
    assert(AppRace_Status()->zone == 2U && s_position_calls == 0U);
    click();
    assert(AppRace_Status()->state == RACE_RUNNING && s_position_calls == 1U);
    tick(50U);
    assert(s_position_calls == 1U);
    click();
    assert(AppRace_Status()->state == RACE_ABORTED && !AppRace_ControlTick());
    click();
    assert(AppRace_Status()->state == RACE_ABORTED);
    init(0U);
    tick(40U);
    click();
    assert(AppRace_Status()->state == RACE_READY);
    s_bias_ready = 0U;
    tick(RACE_WARMUP_MS);
    click();
    assert(AppRace_Status()->state == RACE_READY);
    start();
    calls = s_position_calls;
    stops = s_stops;
    s_motors[0].output = 100.0f;
    s_now += RACE_IMU_STALE_MS;
    assert(!AppRace_ControlTick());
    assert(s_stops == stops + 4U && s_motors[0].output == 0.0f);
    AppRace_Poll();
    assert(AppRace_Status()->error == RACE_ERROR_LOOP_STALL && s_position_calls == calls);
    start();
    s_imu_ok = 0U;
    tick(1U);
    assert(AppRace_Status()->error == RACE_ERROR_IMU && !AppRace_ControlTick());
    start();
    s_link_ok = 0U;
    tick(1U);
    assert(AppRace_Status()->error == RACE_ERROR_LINK && !AppRace_ControlTick());
    start();
    while (AppRace_Status()->action_index < 4U) {
        uint8_t index = AppRace_Status()->action_index;
        const RaceFsm *race = AppRace_Status();
        s_chassis.x_pos = race->target_x;
        s_chassis.y_pos = race->target_y;
        s_chassis.theta_pos = race->target_yaw;
        tick(race->actions[index].hold_ms + 2U);
        assert(AppRace_Status()->action_index == index + 1U);
    }
    /* Complete the scan action before exercising the material route. */
    {
        const RaceFsm *race = AppRace_Status();
        s_chassis.x_pos = race->target_x;
        s_chassis.y_pos = race->target_y;
        s_chassis.theta_pos = race->target_yaw;
        s_qr = 1U;
        tick(1U);
        assert(AppRace_Status()->action_index == 5U);
    }
    while (AppRace_Status()->action_index < 7U) {
        uint8_t index = AppRace_Status()->action_index;
        const RaceFsm *race = AppRace_Status();
        s_chassis.x_pos = race->target_x;
        s_chassis.y_pos = race->target_y;
        s_chassis.theta_pos = race->target_yaw;
        tick(race->actions[index].hold_ms + 2U);
        assert(AppRace_Status()->action_index == index + 1U);
    }
    {
        const RaceFsm *race = AppRace_Status();
        s_chassis.x_pos = race->target_x;
        s_chassis.y_pos = race->target_y;
        s_chassis.theta_pos = race->target_yaw;
        tick(race->actions[7].hold_ms + 2U);
    }
    assert(s_scans == 1U && s_position_calls == 5U && s_chassis.mode == 0U);
#ifdef TEST_TASK_MISSION
    assert(AppRace_Status()->state == RACE_TASK && !AppRace_LogFinished());
    assert(s_vision.check_step == 1U && s_vision.check_phase == 'B');
    tick(RACE_CHECK_RETRY_MS + 1U);
    assert(AppRace_TaskPlan()->index == 0U);
    if (getenv("TEST_DROP_CONFIRMATION")) {
        s_vision.check_received = 1U;
        tick(10000U);
        assert(s_vision.check_phase == 'A' && AppRace_TaskPlan()->index == 0U);
        tick(RACE_CHECK_TIMEOUT_MS);
        assert(AppRace_Status()->state == RACE_FAULT);
        assert(AppRace_TaskPlan()->index == 0U && !AppRace_ControlTick());
        return 0;
    }
    if (getenv("TEST_STOP_TRANSFER")) {
        s_vision.check_received = 1U;
        tick(1U);
        click();
        assert(AppRace_Status()->state == RACE_ABORTED);
        assert(AppRace_TaskPlan()->index == 0U && !AppRace_ControlTick());
        return 0;
    }
    for (unsigned step = 1U; step <= 25U; ++step) {
        uint32_t limit;
        for (limit = 0; limit < 10000U; ++limit) {
            const RaceFsm *race = AppRace_Status();
            if (race->state == RACE_RUNNING) {
                s_chassis.x_pos = race->target_x;
                s_chassis.y_pos = race->target_y;
                s_chassis.theta_pos = race->target_yaw;
            }
            if (s_vision.check_step == step && s_vision.check_phase == 'B') break;
            tick(1U);
        }
        assert(limit < 10000U);
        s_vision.check_received = 1;
        tick(1U);
        for (limit = 0; limit < 12000U && s_vision.check_phase != 'A'; ++limit) tick(1U);
        assert(s_vision.check_phase == 'A');
        /* The arm can finish but no transfer counts without a new DID. */
        uint8_t index = AppRace_TaskPlan()->index;
        tick(RACE_CHECK_RETRY_MS + 1U);
        assert(AppRace_TaskPlan()->index == index);
        s_vision.check_received = 1;
        tick(1U);
        assert(AppRace_TaskPlan()->index == index + 1U);
    }
    assert(RaceTaskPlan_Done(AppRace_TaskPlan()) && AppRace_LogFinished());
    puts("PASS: 30 task entries, 24 physical-evidence handshakes, selected home confirmed");
#endif
    assert(AppRace_Status()->state == RACE_DONE && !AppRace_ControlTick());
    assert(strcmp(AppRace_Status()->task_code, "452+321+254+312") == 0);
    puts("PASS: boot-held/warmup keys, zone select, one-shot start/abort, PID transition ownership, stall/IMU/link stops, scan completion");
    return 0;
}
