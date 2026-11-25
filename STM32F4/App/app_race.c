#include "app_mode.h"
#include "app_race.h"
#include "app_ring_align.h"
#include "race_config.h"
#include "vision.h"
#include "app_imu.h"
#include "app_pick_fsm.h"
#include "app_stepper_openloop.h"
#include "bsp_servo.h"
#include "race_task_plan.h"
#include "race_ui.h"
#include <string.h>

#if APP_RUN_MODE == APP_MODE_RACE
static RaceFsm s_race;
static MecanumChassis_t *s_chassis;
static uint32_t s_boot, s_frames, s_last_imu;
static volatile uint32_t s_last_poll;
static volatile uint8_t s_active, s_control, s_stalled;
static RaceInput s_input;
static PickFsm s_pick;
static RaceTaskPlan s_plan;
static AppRingAlign s_ring;
static uint8_t s_direct_cycle_done;
static uint8_t s_ring_control;
typedef enum { TASK_NEXT, TASK_TRAVEL, TASK_DOCK, TASK_ALIGN, TASK_BEFORE, TASK_ARM,
               TASK_AFTER, TASK_RESET } TaskStage;
static TaskStage s_stage;
static uint8_t s_mission, s_step;
static uint32_t s_check_since, s_check_sent;
#if (RACE_PICK_DIRECT_KEY_TEST != 0U)
static void start_direct_pick(uint32_t now, uint8_t reverse)
{
    if (PickFsm_IsReady(&s_pick)) {
        if (reverse) (void)PickFsm_StartReverse(&s_pick, now);
        else (void)PickFsm_Start(&s_pick, now);
    }
}
#endif


static const PickConfig s_pick_config = {
    RACE_PICK_ROTATE_HOME_DEG,
    RACE_PICK_ROTATE_PLACE_DEG,
    RACE_PICK_GRIPPER_OPEN_DEG,
    RACE_PICK_GRIPPER_PRECLOSE_DEG,
    RACE_PICK_GRIPPER_CLOSE_DEG,
    RACE_PICK_LIFT_APPROACH_STEPS,
    RACE_PICK_LIFT_DROP_TO_GRAB_STEPS,
    RACE_PICK_FORWARD_TO_170_STEPS,
    RACE_PICK_FORWARD_RESET_LIFT_STEPS,
    RACE_PICK_REVERSE_TO_175_STEPS,
    RACE_PICK_REVERSE_TO_141_STEPS,
    RACE_PICK_REVERSE_TO_175_AFTER_CLOSE_STEPS,
    RACE_PICK_REVERSE_TO_GROUND_STEPS,
    RACE_PICK_REVERSE_RESET_LIFT_STEPS,
    RACE_PICK_LIFT_FREQUENCY_HZ,
    RACE_PICK_LIFT_ACCEL_RPMS,
    RACE_PICK_ROTATE_SPEED_DPS,
    RACE_PICK_ROTATE_ACCEL_DPS2,
    RACE_PICK_GRIPPER_SPEED_DPS,
    RACE_PICK_GRIPPER_ACCEL_DPS2,
    RACE_PICK_LIFT_BEFORE_ROTATE_DONE_MS,
    RACE_PICK_SERVO_TIMEOUT_MS,
    RACE_PICK_LIFT_TIMEOUT_MS,
    RACE_PICK_SETTLE_MS,
    RACE_PICK_DIRECTION_SETTLE_MS,
    RACE_PICK_ROTATE_TRIGGER_STEPS,
    RACE_PICK_REVERSE_ROTATE_TRIGGER_STEPS,
    RACE_PICK_REVERSE_HOME_ROTATE_TRIGGER_STEPS,
    RACE_PICK_FORWARD_CYCLES,
    RACE_PICK_TURNTABLE_SERVO_SPEED_DPS,
    RACE_PICK_TURNTABLE_SERVO_ACCEL_DPS2
};

static void stop_outputs(void)
{
    Mecanum_Stop(s_chassis);
    Motor_Stop(s_chassis->motor_lf);
    Motor_Stop(s_chassis->motor_rf);
    Motor_Stop(s_chassis->motor_lb);
    Motor_Stop(s_chassis->motor_rb);
}

static void stop_vision_chassis(void *user)
{
    MecanumChassis_t *chassis = (MecanumChassis_t *)user;
    if (chassis != 0) Mecanum_Stop(chassis);
}

static const RingAlignConfig s_ring_config = {
    RACE_RING_J00_PX_PER_M,
    RACE_RING_J01_PX_PER_M,
    RACE_RING_J10_PX_PER_M,
    RACE_RING_J11_PX_PER_M,
    RACE_RING_GAIN,
    RACE_RING_MAX_STEP_M,
    RACE_RING_MIN_STEP_M,
    RACE_RING_MAX_SPEED_MPS,
    RACE_RING_MAX_OMEGA_RADPS,
    8.0f,
    8.0f
};

static RaceFeedback feedback(uint32_t now)
{
    RaceFeedback result;
    uint32_t saved_irq = __get_PRIMASK();
    __disable_irq();
    result.x = s_chassis->x_pos;
    result.y = s_chassis->y_pos;
    result.yaw = s_chassis->theta_pos;
    result.vx = s_chassis->vx_actual;
    result.vy = s_chassis->vy_actual;
    result.omega = s_chassis->omega_actual;
    __set_PRIMASK(saved_irq);
    result.imu_valid = app_imu_ok() && s_frames != 0U &&
                       (uint32_t)(now - s_last_imu) < RACE_IMU_STALE_MS;
    return result;
}

static uint8_t warmed_up(uint32_t now)
{
    return app_imu_bias_ready() && (uint32_t)(now - s_boot) >= RACE_WARMUP_MS;
}

static uint8_t travel_leg(TaskActionKind kind)
{
    switch (kind) {
    case TASK_ACTION_GO_ROUGH: return 1U;
    case TASK_ACTION_GO_STORAGE: return 2U;
    case TASK_ACTION_GO_MATERIAL: return 3U;
    case TASK_ACTION_RETURN_START: return 4U;
    default: return 0U;
    }
}

static uint8_t start_transfer(const TaskAction *action, uint32_t now)
{
    static const float tray_angles[] = {90.0f, 0.0f, 270.0f};
    if (action->item >= 3U || !PickFsm_IsReady(&s_pick)) return 0U;
    if (action->kind == TASK_ACTION_PICK || action->kind == TASK_ACTION_RETRIEVE_ROUGH) {
        s_pick.forward_cycle_index = action->item;
        return PickFsm_Start(&s_pick, now);
    }
    s_pick.config.reverse_to_ground_steps = s_pick_config.reverse_to_ground_steps;
    s_pick.config.reverse_reset_steps = s_pick_config.reverse_reset_steps;
    if (action->kind == TASK_ACTION_STACK_STORAGE) {
        const int32_t extra = RACE_STACK_EXTRA_STEPS;
        if (extra <= 0 || extra >= s_pick.config.reverse_to_ground_steps) return 0U;
        s_pick.config.reverse_to_ground_steps -= extra;
        s_pick.config.reverse_reset_steps += extra;
    }
    return PickFsm_SetReverseTurntableAngle(&s_pick, tray_angles[action->item]) &&
           PickFsm_StartReverse(&s_pick, now);
}

static void request_check(const TaskAction *action, TaskStage stage, uint32_t now)
{
    VisionContext *vision = Vision_GetDefault();
    if (s_stage != stage) s_check_since = now;
    s_stage = stage;
    s_check_sent = now;
    if (!Vision_RequestCheck(vision, (uint16_t)s_race.scan_session, s_step,
                             stage == TASK_BEFORE ? 'B' : 'A', action->color,
                             action->kind == TASK_ACTION_RETURN_START ? s_race.zone : action->position))
        Race_Fail(&s_race, RACE_ERROR_LINK);
}

static void poll_mission(const RaceFeedback *current, uint32_t now)
{
    const TaskAction *action = RaceTaskPlan_Current(&s_plan);
    VisionContext *vision = Vision_GetDefault();
    uint8_t leg;
    if (!action || !current->imu_valid || !RaceVision_Healthy() ||
        vision->result == VISION_RESULT_ERROR || vision->result == VISION_RESULT_TIMEOUT) {
        Race_Fail(&s_race, !current->imu_valid ? RACE_ERROR_IMU : RACE_ERROR_VISION_CHECK);
        return;
    }
    switch (s_stage) {
    case TASK_NEXT:
        leg = travel_leg(action->kind);
        if (leg || action->position) {
            s_stage = leg ? TASK_TRAVEL : TASK_DOCK;
            if (!leg) leg = (action->kind == TASK_ACTION_PLACE_ROUGH ||
                             action->kind == TASK_ACTION_RETRIEVE_ROUGH) ? 5U : 6U;
            if (!Race_StartTaskRoute(&s_race, leg, action->position, current, now))
                Race_Fail(&s_race, RACE_ERROR_ROUTE_UNTAUGHT);
        } else {
            request_check(action, TASK_BEFORE, now);
        }
        break;
    case TASK_TRAVEL:
        if (action->kind != TASK_ACTION_RETURN_START) {
            (void)RaceTaskPlan_Advance(&s_plan);
            s_stage = TASK_NEXT;
            break;
        }
        /* Home also needs fresh blue-zone evidence after the odometry stop. */
        request_check(action, TASK_BEFORE, now);
        break;
    case TASK_DOCK:
        if (s_ring.calibration_valid && (action->kind == TASK_ACTION_PLACE_ROUGH ||
                                         action->kind == TASK_ACTION_PLACE_STORAGE)) {
            if (AppRingAlign_Start(&s_ring, action->position, now))
                s_stage = TASK_ALIGN;
            else
                Race_Fail(&s_race, RACE_ERROR_VISION_CHECK);
        } else {
            request_check(action, TASK_BEFORE, now);
        }
        break;
    case TASK_ALIGN:
        AppRingAlign_Poll(&s_ring, now);
        if (AppRingAlign_Failed(&s_ring)) {
            Race_Fail(&s_race, RACE_ERROR_VISION_CHECK);
        } else if (AppRingAlign_Succeeded(&s_ring)) {
            Mecanum_Stop(s_chassis);
            request_check(action, TASK_BEFORE, now);
        }
        break;
    case TASK_BEFORE:
    case TASK_AFTER:
        if ((uint32_t)(now - s_check_since) >= RACE_CHECK_TIMEOUT_MS) {
            Race_Fail(&s_race, RACE_ERROR_VISION_CHECK);
        } else if (vision->check_received) {
            vision->check_received = 0U;
            if (s_stage == TASK_BEFORE) {
                if (action->kind == TASK_ACTION_RETURN_START) {
                    request_check(action, TASK_AFTER, now);
                } else if (start_transfer(action, now)) {
                    s_stage = TASK_ARM;
                } else {
                    Race_Fail(&s_race, RACE_ERROR_ARM);
                }
            } else {
                (void)RaceTaskPlan_Advance(&s_plan);
                s_step++;
                vision->check_phase = 0;
                if (RaceTaskPlan_Done(&s_plan)) {
                    s_mission = 0U;
                    s_race.state = RACE_DONE;
                } else {
                    PickFsm_Reset(&s_pick, now);
                    s_stage = TASK_RESET;
                }
            }
        } else if ((uint32_t)(now - s_check_sent) >= RACE_CHECK_RETRY_MS) {
            request_check(action, s_stage, now);
        }
        break;
    case TASK_ARM:
        if (PickFsm_IsDone(&s_pick)) request_check(action, TASK_AFTER, now);
        else if (!PickFsm_IsActive(&s_pick)) Race_Fail(&s_race, RACE_ERROR_ARM);
        break;
    case TASK_RESET:
        if (PickFsm_IsReady(&s_pick)) s_stage = TASK_NEXT;
        else if (!PickFsm_IsActive(&s_pick)) Race_Fail(&s_race, RACE_ERROR_ARM);
        break;
    }
}

void AppRace_Init(MecanumChassis_t *chassis)
{
    memset(&s_plan, 0, sizeof(s_plan));
    s_mission = 0U;
    s_stage = TASK_NEXT;
    s_step = 1U;
    s_chassis = chassis;
    s_direct_cycle_done = 0U;
    s_ring_control = 0U;
    Race_Init(&s_race);
    s_boot = HAL_GetTick();
    s_last_poll = s_boot;
    s_last_imu = s_boot;
    s_frames = app_imu_frames();
    s_active = 0U;
    s_stalled = 0U;
    s_control = 1U;
    RaceInput_Init(&s_input, s_boot);
    RaceUi_Init();
    if (RACE_PICK_ENABLE != 0U) {
        AppStepper_Init();
        Servo_Init();
#if (RACE_PICK_TURNTABLE_ENABLE == 0U)
        Servo_Disable(SERVO_3);
#else
        /* Hold PC7 output off at boot; the first tray command enables it. */
        Servo_Disable(SERVO_3);
#endif
    }
    PickFsm_Init(&s_pick, &s_pick_config);
    (void)PickFsm_SetReverseTurntableAngle(
        &s_pick, RACE_PICK_REVERSE_TURNTABLE_ANGLE_DEG);
    if (RACE_PICK_DIRECT_KEY_TEST != 0U) {
        PickFsm_Reset(&s_pick, s_boot);
    }
    stop_outputs();
    Mecanum_SetSpeedLimit(chassis, RACE_MOVE_SPEED_MPS, RACE_TURN_SPEED_RADPS);
    Mecanum_SetTolerance(chassis, RACE_POSITION_TOL_M * 0.5f, RACE_YAW_TOL_RAD * 0.5f);
    if (!RaceVision_Init()) Race_Fail(&s_race, RACE_ERROR_LINK);
    Vision_SetStop(Vision_GetDefault(), stop_vision_chassis, chassis);
    AppRingAlign_Init(&s_ring, Vision_GetDefault(), chassis, &s_ring_config);
    if (!app_imu_ok()) Race_Fail(&s_race, RACE_ERROR_IMU);
}

uint8_t AppRace_ControlTick(void)
{
    if (s_chassis == NULL) return 0U;
    if ((s_active || s_ring_control) &&
        (uint32_t)(HAL_GetTick() - s_last_poll) >= RACE_IMU_STALE_MS) {
        s_stalled = 1U;
        s_control = 0U;
        s_active = 0U;
        stop_outputs();
        PickFsm_Abort(&s_pick);
    }
    return s_control;
}

void AppRace_Poll(void)
{
    uint32_t now = HAL_GetTick();
    uint32_t frames = app_imu_frames();
    RaceState previous_state = s_race.state;
    uint8_t previous_index = s_race.action_index;
    uint32_t saved_irq;
    RaceFeedback current;
    char code[RACE_TASK_CODE_SIZE];
    uint8_t input_event;
    uint8_t can_start;
    if (frames != s_frames) {
        s_frames = frames;
        s_last_imu = now;
    }
    current = feedback(now);
    if (s_stalled) Race_Fail(&s_race, RACE_ERROR_LOOP_STALL);
    if (!RaceVision_Healthy() && (s_race.state == RACE_READY || s_race.state == RACE_RUNNING)) {
        Race_Fail(&s_race, RACE_ERROR_LINK);
    }
    if (RaceVision_Poll(code)) {
        if (Race_SubmitQr(&s_race, s_race.scan_session, code)) {
            /* Exactly once: validate and build the plan before moving on. */
            if (!RaceVision_AckQr(s_race.scan_session)) {
                Race_Fail(&s_race, RACE_ERROR_LINK);
            } else if (!RaceTaskPlan_Build(&s_plan,
                                           (const uint8_t (*)[3])s_race.colors,
                                           (const uint8_t (*)[3])s_race.positions) ||
                       !RaceTaskPlan_Start(&s_plan)) {
                Race_Fail(&s_race, RACE_ERROR_CONFIG);
            }
        } else if (s_race.state == RACE_RUNNING && s_race.qr_valid &&
                   strcmp(s_race.task_code, code) == 0) {
            /* OpenMV retries QR until ACK, which itself may be lost.
             * Re-ACK identical data without advancing the race plan twice. */
            if (!RaceVision_AckQr(s_race.scan_session))
                Race_Fail(&s_race, RACE_ERROR_LINK);
        }
    }
    if (s_race.state == RACE_RUNNING &&
        s_race.actions[s_race.action_index].kind == RACE_ACTION_SCAN &&
        RaceVision_ScanFailed()) {
        Race_Fail(&s_race, RaceVision_Healthy() ?
                  RACE_ERROR_SCAN_TIMEOUT : RACE_ERROR_LINK);
    }
    if (!s_mission && (s_ring_control || Vision_RingActive(Vision_GetDefault()))) {
        if (s_stalled && Vision_RingActive(Vision_GetDefault()))
            (void)Vision_Cancel(Vision_GetDefault());
        AppRingAlign_Poll(&s_ring, now);
        if (Vision_RingFailed(Vision_GetDefault())) {
            s_ring_control = 0U;
            stop_outputs();
            s_control = 0U;
        } else if (Vision_RingSucceeded(Vision_GetDefault())) {
            s_ring_control = 0U;
            Mecanum_Stop(s_chassis);
        } else {
            s_ring_control = 1U;
        }
        s_active = s_ring_control;
        s_last_poll = now;
        RaceUi_Poll(&s_race, now, warmed_up(now));
        return;
    }
 #if (RACE_PICK_DIRECT_KEY_TEST != 0U)
    can_start = PickFsm_IsReady(&s_pick);
 #else
    can_start = (s_race.state == RACE_READY) && warmed_up(now) && current.imu_valid;
 #endif
    input_event = RaceInput_Poll(&s_input, now,
                                 s_race.state == RACE_RUNNING || s_mission, can_start);
    if ((input_event & RACE_INPUT_PRESS) != 0U &&
        (s_race.state == RACE_RUNNING || s_mission)) {
        Race_Abort(&s_race);
    }
    if ((input_event & RACE_INPUT_LONG) != 0U &&
        s_race.state == RACE_READY) {
        (void)Race_SelectZone(&s_race, s_race.zone == 1U ? 2U : 1U);
    }
    if ((input_event & RACE_INPUT_SHORT_RELEASE) != 0U) {
 #if (RACE_PICK_DIRECT_KEY_TEST != 0U)
        start_direct_pick(now, 0U);
 #else
        if (s_race.state == RACE_READY) {
            (void)Race_StartQrRoute(&s_race, &current, now);
        }
 #endif
    }
    Race_Tick(&s_race, &current, now);
    if (previous_state == RACE_RUNNING && s_race.state == RACE_DONE &&
        RACE_PICK_ENABLE && !RACE_PICK_DIRECT_KEY_TEST && s_plan.valid) {
        Mecanum_Stop(s_chassis);
        if (!s_mission) RaceVision_EndScan();
        s_mission = 1U;
        s_race.state = RACE_TASK;
    }
    if (s_mission && s_race.state == RACE_TASK)
        poll_mission(&current, now);
    if ((s_race.state == RACE_FAULT || s_race.state == RACE_ABORTED) && s_mission) {
        s_mission = 0U;
        if (s_stage == TASK_ALIGN) AppRingAlign_Cancel(&s_ring);
        Vision_GetDefault()->check_phase = 0;
        (void)Vision_Cancel(Vision_GetDefault());
        PickFsm_Abort(&s_pick);
    }
    if (PickFsm_IsActive(&s_pick) || s_pick.state == PICK_RESETTING) PickFsm_Poll(&s_pick, now);
#if (RACE_PICK_DIRECT_KEY_TEST != 0U)
    if (PickFsm_IsDone(&s_pick)) {
        s_direct_cycle_done = 1U;
        PickFsm_Reset(&s_pick, now);
    }
#endif
    if (s_race.state == RACE_RUNNING &&
        (previous_state != RACE_RUNNING || previous_index != s_race.action_index)) {
        const RaceAction *action = &s_race.actions[s_race.action_index];
        saved_irq = __get_PRIMASK();
        __disable_irq();
        if (!s_stalled && (action->kind == RACE_ACTION_MOVE || action->kind == RACE_ACTION_TURN)) {
            s_chassis->body_frame = 0U;
            Mecanum_SetPositionMode(s_chassis, s_race.target_x, s_race.target_y, s_race.target_yaw);
        }
        __set_PRIMASK(saved_irq);
        if (action->kind == RACE_ACTION_SCAN &&
            !RaceVision_BeginScanSession(s_race.scan_session)) {
            Race_Fail(&s_race, RACE_ERROR_LINK);
        }
    }
    saved_irq = __get_PRIMASK();
    __disable_irq();
    if (s_stalled) Race_Fail(&s_race, RACE_ERROR_LOOP_STALL);
    s_active = s_mission || (s_race.state == RACE_RUNNING) || PickFsm_IsActive(&s_pick) ||
               s_ring_control;
    s_last_poll = now;
    if (!s_mission && s_race.state != RACE_READY && s_race.state != RACE_RUNNING &&
        !PickFsm_IsActive(&s_pick)) {
        if (s_control) stop_outputs();
        s_control = 0U;
    }
    __set_PRIMASK(saved_irq);
    if (!s_active) RaceVision_EndScan();
    RaceUi_Poll(&s_race, now, warmed_up(now));
}

const RaceFsm *AppRace_Status(void)
{
    return &s_race;
}

const RaceTaskPlan *AppRace_TaskPlan(void)
{
    return &s_plan;
}

uint8_t AppRace_StartRingAlign(uint8_t target)
{
    if (s_chassis == NULL || s_race.state != RACE_READY || s_ring_control ||
        PickFsm_IsActive(&s_pick)) return 0U;
    s_ring_control = AppRingAlign_Start(&s_ring, target, HAL_GetTick());
    return s_ring_control;
}

uint8_t AppRace_LogFinished(void)
{
    if (s_race.state == RACE_FAULT || s_race.state == RACE_ABORTED ||
        s_pick.state == PICK_FAULT || s_pick.state == PICK_ABORTED) return 1U;
    if (PickFsm_IsActive(&s_pick) ||
        AppStepper_IsBusy(STEPPER_AXIS_1) ||
        Servo_IsBusy(SERVO_1) || Servo_IsBusy(SERVO_2) ||
        Servo_IsBusy(SERVO_3)) return 0U;
#if (RACE_PICK_DIRECT_KEY_TEST != 0U)
    /* Boot reset is not a completed test. Wait for the post-test reset. */
    return s_direct_cycle_done && PickFsm_IsReady(&s_pick);
#else
    return s_race.state == RACE_DONE &&
           (RACE_PICK_ENABLE == 0U || RaceTaskPlan_Done(&s_plan));
#endif
}
#endif
