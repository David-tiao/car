#include "app_mode.h"
#include "app_race.h"
#include "race_config.h"
#include "race_vision.h"
#include "app_imu.h"
#include "bsp_oled.h"
#include "app_pick_fsm.h"
#include "app_stepper_openloop.h"
#include "bsp_servo.h"
#include "race_task_plan.h"
#include <stdio.h>
#include <math.h>
#include <string.h>

#if APP_RUN_MODE == APP_MODE_RACE
static RaceFsm s_race;
static MecanumChassis_t *s_chassis;
static uint32_t s_boot, s_frames, s_last_imu;
static volatile uint32_t s_last_poll;
static volatile uint8_t s_active, s_control, s_stalled;
static uint8_t s_key_raw, s_key_stable, s_key_armed, s_key_long, s_key_start;
static uint32_t s_key_changed, s_key_pressed;
static uint32_t s_screen_time, s_page_time;
static uint8_t s_page;
static PickFsm s_pick;
static RaceTaskPlan s_plan;

typedef enum {
    PICK_FLOW_ROUTE = 0,
    PICK_FLOW_ALIGN_WAIT,
    PICK_FLOW_ALIGN_MOVING,
    PICK_FLOW_ARMING,
    PICK_FLOW_WAIT_ITEM,
    PICK_FLOW_PICKING,
    PICK_FLOW_CONFIRM,
    PICK_FLOW_GROUP_DONE,
    PICK_FLOW_COMPLETE
} PickFlow;

static PickFlow s_flow;
static uint8_t s_disk_ok, s_slot_valid, s_arm_ok, s_take_ok;
static int16_t s_slot_x, s_slot_y;
static uint32_t s_slot_ms, s_slot_number, s_handled_slot, s_phase_since;
static uint32_t s_align_reached_ms, s_arm_sent_ms, s_take_sent_ms;
static float s_align_target_x, s_align_target_y, s_align_distance;
static uint8_t s_align_hits;

static uint8_t pick_flow_running(void)
{
    return (s_flow != PICK_FLOW_ROUTE && s_flow != PICK_FLOW_COMPLETE &&
            s_race.state == RACE_DONE);
}

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
    RACE_PICK_TURNTABLE_CCW_SIGN,
    RACE_PICK_TURNTABLE_FIRST_CCW_STEPS,
    RACE_PICK_TURNTABLE_CW_STEPS,
    RACE_PICK_TURNTABLE_RETURN_CCW_STEPS,
    RACE_PICK_TURNTABLE_FREQUENCY_HZ,
    RACE_PICK_TURNTABLE_START_FREQUENCY_HZ,
    RACE_PICK_TURNTABLE_END_FREQUENCY_HZ,
    RACE_PICK_TURNTABLE_ACCEL_STEPS,
    RACE_PICK_TURNTABLE_DECEL_STEPS,
    RACE_PICK_TURNTABLE_SETTLE_MS,
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

static void poll_key(const RaceFeedback *current, uint32_t now)
{
    uint8_t raw = HAL_GPIO_ReadPin(GPIOE, GPIO_PIN_0) == GPIO_PIN_RESET;
    if (raw != s_key_raw) {
        s_key_raw = raw;
        s_key_changed = now;
    }
    if ((uint32_t)(now - s_key_changed) < 20U) return;
    if (!s_key_armed) {
        s_key_stable = raw;
        if (!raw) s_key_armed = 1U;
        return;
    }
    if (raw != s_key_stable) {
        s_key_stable = raw;
        if (raw) {
            s_key_pressed = now;
            s_key_long = 0U;
            s_key_start = s_race.state == RACE_READY &&
                          warmed_up(now) && current->imu_valid;
            if (s_race.state == RACE_RUNNING || pick_flow_running()) {
                Race_Abort(&s_race);
                PickFsm_Abort(&s_pick);
            }
        } else if (!s_key_long && s_key_start && s_race.state == RACE_READY) {
            (void)Race_StartQrRoute(&s_race, current, now);
        }
    }
    if (raw && !s_key_long && (uint32_t)(now - s_key_pressed) >= 700U) {
        s_key_long = 1U;
        if (s_race.state == RACE_READY)
            (void)Race_SelectZone(&s_race, s_race.zone == 1U ? 2U : 1U);
    }
}

static void text_line(uint8_t row, const char *text)
{
    uint8_t column = 0U;
    while (*text && column < 21U) {
        OLED_ShowChar((uint8_t)(column * 6U), row, (uint8_t)*text++, 12U, 1U);
        column++;
    }
}

static void poll_display(uint32_t now)
{
    static const char *const states[] = {"READY", "RUNNING", "MATERIAL READY", "FAULT", "STOPPED"};
    static const char *const actions[] = {"MOVE", "TURN", "SETTLE", "SCAN"};
    char line[32];
    uint8_t column;
    if (s_page == 0U && (uint32_t)(now - s_screen_time) >= 200U) {
        s_screen_time = now;
        memset(OLED_GRAM, 0, sizeof(OLED_GRAM));
        (void)snprintf(line, sizeof(line), "RACE Z%u %s", s_race.zone, states[s_race.state]);
        text_line(0U, line);
        if (s_race.state == RACE_READY) {
            text_line(12U, warmed_up(now) ? "SHORT:START" : "IMU WARMUP: KEEP STILL");
            text_line(24U, "LONG:ZONE 1/2");
        } else if (s_race.state == RACE_RUNNING) {
            (void)snprintf(line, sizeof(line), "%u/%u %s", s_race.action_index + 1U,
                           s_race.action_count, actions[s_race.actions[s_race.action_index].kind]);
            text_line(12U, line);
            text_line(24U, "PRESS:STOP");
        } else if (s_race.state == RACE_DONE) {
            text_line(12U, s_flow == PICK_FLOW_COMPLETE ? "BATCH1 COMPLETE" : "ALIGN / WAIT PICK");
            if (s_flow == PICK_FLOW_WAIT_ITEM) text_line(24U, "WAIT QR COLOR");
            else if (s_flow == PICK_FLOW_PICKING || s_flow == PICK_FLOW_CONFIRM)
                text_line(24U, "SINGLE PICK BUSY");
        } else {
            (void)snprintf(line, sizeof(line), "ERROR:%u RESET TO ARM", (unsigned)s_race.error);
            text_line(12U, line);
        }
        text_line(36U, s_race.task_code);
        text_line(48U, RaceVision_Healthy() ? "USART3:RX READY" : "USART3:FAULT");
    }
    if ((uint32_t)(now - s_page_time) < 2U) return;
    s_page_time = now;
    OLED_WR_Byte((uint8_t)(0xb0U + s_page), OLED_CMD);
    OLED_WR_Byte(0x00U, OLED_CMD);
    OLED_WR_Byte(0x10U, OLED_CMD);
    for (column = 0U; column < 128U; column++) OLED_WR_Byte(OLED_GRAM[column][s_page], OLED_DATA);
    s_page = (uint8_t)((s_page + 1U) & 7U);
}

void AppRace_Init(MecanumChassis_t *chassis)
{
    s_chassis = chassis;
    s_flow = PICK_FLOW_ROUTE;
    s_disk_ok = 0U;
    s_slot_valid = 0U;
    s_arm_ok = 0U;
    s_take_ok = 0U;
    s_slot_number = s_handled_slot = 0U;
    s_align_hits = 0U;
    s_align_distance = 0.0f;
    memset(&s_plan, 0, sizeof(s_plan));
    Race_Init(&s_race);
    s_boot = HAL_GetTick();
    s_last_poll = s_boot;
    s_last_imu = s_boot;
    s_frames = app_imu_frames();
    s_active = 0U;
    s_stalled = 0U;
    s_control = 1U;
    s_key_armed = 0U;
    s_key_raw = HAL_GPIO_ReadPin(GPIOE, GPIO_PIN_0) == GPIO_PIN_RESET;
    s_key_stable = s_key_raw;
    s_key_changed = s_boot;
    if (RACE_PICK_ENABLE != 0U) {
        AppStepper_Init();
        Servo_Init();
    }
    PickFsm_Init(&s_pick, &s_pick_config);
    if (RACE_PICK_ENABLE != 0U) {
        PickFsm_Reset(&s_pick, s_boot);
    }
    stop_outputs();
    Mecanum_SetSpeedLimit(chassis, RACE_MOVE_SPEED_MPS, RACE_TURN_SPEED_RADPS);
    Mecanum_SetTolerance(chassis, RACE_POSITION_TOL_M * 0.5f, RACE_YAW_TOL_RAD * 0.5f);
    if (!RaceVision_Init()) Race_Fail(&s_race, RACE_ERROR_LINK);
    if (!app_imu_ok()) Race_Fail(&s_race, RACE_ERROR_IMU);
}

uint8_t AppRace_ControlTick(void)
{
    if (s_chassis == NULL) return 0U;
    if (s_active && (uint32_t)(HAL_GetTick() - s_last_poll) >= RACE_IMU_STALE_MS) {
        s_stalled = 1U;
        s_control = 0U;
        s_active = 0U;
        stop_outputs();
        PickFsm_Abort(&s_pick);
    }
    return s_control;
}

/* Local image target -> small absolute odometry target. The matrix is
 * deliberately NOT inferred from arbitrary camera orientation. */
static uint8_t align_step(const RaceFeedback *f, uint32_t now)
{
    float ex, ey, bx, by, step, c, s, wx, wy;
    ex = RACE_ALIGN_TARGET_X_PX - (float)s_slot_x;
    ey = RACE_ALIGN_TARGET_Y_PX - (float)s_slot_y;
    if (fabsf(ex) <= RACE_ALIGN_PIXEL_TOL && fabsf(ey) <= RACE_ALIGN_PIXEL_TOL) {
        if (++s_align_hits >= 3U &&
            hypotf(f->vx, f->vy) < RACE_STILL_SPEED_MPS &&
            fabsf(f->omega) < RACE_STILL_OMEGA_RADPS) {
            s_flow = PICK_FLOW_ARMING;
            s_phase_since = now;
            s_arm_sent_ms = now - RACE_ARM_RETRY_MS;
            stop_outputs();
        }
        return 1U;
    }
    s_align_hits = 0U;
    /* Position already correct: safe to arm without a motion calibration.
     * Any actual motor correction requires an explicitly calibrated map. */
    if (!RACE_ALIGN_CALIBRATED ||
        (RACE_ALIGN_M_PER_PX_XX == 0.0f && RACE_ALIGN_M_PER_PX_XY == 0.0f &&
         RACE_ALIGN_M_PER_PX_YX == 0.0f && RACE_ALIGN_M_PER_PX_YY == 0.0f))
        return 0U;
    bx = RACE_ALIGN_M_PER_PX_XX * ex + RACE_ALIGN_M_PER_PX_XY * ey;
    by = RACE_ALIGN_M_PER_PX_YX * ex + RACE_ALIGN_M_PER_PX_YY * ey;
    step = hypotf(bx, by);
    if (!isfinite(step) || step < 0.0001f) return 0U;
    if (step > RACE_ALIGN_MAX_STEP_M) {
        bx *= RACE_ALIGN_MAX_STEP_M / step;
        by *= RACE_ALIGN_MAX_STEP_M / step;
        step = RACE_ALIGN_MAX_STEP_M;
    }
    if (s_align_distance + step > RACE_ALIGN_MAX_TOTAL_M) return 0U;
    s_align_distance += step;
    c = cosf(f->yaw); s = sinf(f->yaw);
    wx = c * bx - s * by;
    wy = s * bx + c * by;
    s_align_target_x = f->x + wx;
    s_align_target_y = f->y + wy;
    /* Route-following and visual alignment share the same ISR-owned chassis. */
    {
        uint32_t irq = __get_PRIMASK();
        __disable_irq();
        s_chassis->body_frame = 0U;
        Mecanum_SetPositionMode(s_chassis, s_align_target_x,
                                s_align_target_y, f->yaw);
        __set_PRIMASK(irq);
    }
    s_flow = PICK_FLOW_ALIGN_MOVING;
    s_phase_since = now;
    s_align_reached_ms = 0U;
    return 1U;
}

static void accept_vision_event(const RaceVisionEvent *event, uint32_t now)
{
    const TaskAction *item;
    if (event->kind == VISION_SEQ) {
        if (Race_SubmitQr(&s_race, s_race.scan_session, event->task_code)) {
            if (!RaceTaskPlan_Build(&s_plan,
                                    (const uint8_t (*)[3])s_race.colors,
                                    (const uint8_t (*)[3])s_race.positions) ||
                !RaceTaskPlan_Start(&s_plan)) Race_Fail(&s_race, RACE_ERROR_CONFIG);
        }
        return;
    }
    if (!s_plan.valid || s_race.state == RACE_FAULT || s_race.state == RACE_ABORTED) return;
    if (event->kind == VISION_DISK) {
        s_disk_ok = event->disk_locked;
        if (!s_disk_ok) {
            s_slot_valid = 0U;
            s_arm_ok = 0U;
            if (s_flow == PICK_FLOW_WAIT_ITEM || s_flow == PICK_FLOW_ARMING ||
                s_flow == PICK_FLOW_ALIGN_MOVING) {
                stop_outputs();
                s_flow = PICK_FLOW_ALIGN_WAIT;
                s_phase_since = now;
                (void)RaceVision_Send("#ARM,0\n");
            }
        }
    } else if (event->kind == VISION_SLOT) {
        s_slot_valid = 1U;
        s_disk_ok = 1U;
        s_slot_x = event->x;
        s_slot_y = event->y;
        s_slot_ms = now;
        s_slot_number++;
    } else if (event->kind == VISION_ARM_ACK) {
        if (s_flow == PICK_FLOW_ARMING) s_arm_ok = event->arm_enabled;
        else if (s_flow == PICK_FLOW_WAIT_ITEM && !event->arm_enabled) {
            s_flow = PICK_FLOW_ARMING;
            s_phase_since = now;
            s_arm_ok = 0U;
            s_arm_sent_ms = now - RACE_ARM_RETRY_MS;
        }
    } else if (event->kind == VISION_TAKE_ACK) {
        if (s_flow == PICK_FLOW_PICKING || s_flow == PICK_FLOW_CONFIRM)
            s_take_ok = 1U;
    } else if (event->kind == VISION_OBJ) {
        float dx, dy;
        item = RaceTaskPlan_Current(&s_plan);
        if (s_flow != PICK_FLOW_WAIT_ITEM || !s_arm_ok || !s_disk_ok ||
            !s_slot_valid || (uint32_t)(now - s_slot_ms) > 750U ||
            item == 0 || item->kind != TASK_ACTION_PICK ||
            event->color != item->color) return;
        dx = (float)event->x - (float)s_slot_x;
        dy = (float)event->y - (float)s_slot_y;
        if (dx*dx + dy*dy > RACE_PICK_PIXEL_TOL*RACE_PICK_PIXEL_TOL) return;
        if (hypotf(s_chassis->vx_actual, s_chassis->vy_actual) >= RACE_STILL_SPEED_MPS ||
            fabsf(s_chassis->omega_actual) >= RACE_STILL_OMEGA_RADPS) return;
        /* Only start the arm ONCE and only for the requested color. */
        if (!RaceVision_Send("#ARM,0\n")) return;
        if ((RACE_PICK_ENABLE == 0U) ||
            !PickFsm_IsReady(&s_pick) ||
            !PickFsm_StartSingle(&s_pick, now, item->item)) {
            Race_Fail(&s_race, RACE_ERROR_CONFIG);
            return;
        }
        s_arm_ok = 0U;
        s_take_ok = 0U;
        s_take_sent_ms = now - RACE_TAKE_RETRY_MS;
        s_flow = PICK_FLOW_PICKING;
        s_phase_since = now;
    }
}

/* Retried handshake: without TAKE_OK, never advance a finished pick. */
static void retry_take(uint32_t now)
{
    if (!s_take_ok && (uint32_t)(now - s_take_sent_ms) >= RACE_TAKE_RETRY_MS &&
        RaceVision_Send("#TAKE\n")) s_take_sent_ms = now;
}

static void poll_material_flow(const RaceFeedback *f, uint32_t now)
{
    if (s_race.state != RACE_DONE || !s_plan.valid || s_flow == PICK_FLOW_COMPLETE) return;
    if (s_pick.state == PICK_FAULT || s_pick.state == PICK_ABORTED) {
        Race_Fail(&s_race, RACE_ERROR_CONFIG);
        return;
    }
    if (s_flow == PICK_FLOW_ROUTE) {
        s_flow = PICK_FLOW_ALIGN_WAIT;
        s_phase_since = now;
        s_handled_slot = s_slot_number;
        stop_outputs();
    }
    if (s_flow == PICK_FLOW_ALIGN_WAIT) {
        if ((uint32_t)(now - s_phase_since) > RACE_ALIGN_SLOT_TIMEOUT_MS) {
            Race_Fail(&s_race, RACE_ERROR_MOVE_TIMEOUT);
        } else if (s_slot_valid && s_slot_number != s_handled_slot &&
                   (uint32_t)(now - s_slot_ms) < 650U) {
            s_handled_slot = s_slot_number;
            if (!align_step(f, now)) Race_Fail(&s_race, RACE_ERROR_CONFIG);
        }
    } else if (s_flow == PICK_FLOW_ALIGN_MOVING) {
        if ((uint32_t)(now - s_phase_since) > RACE_MOVE_TIMEOUT_MS) {
            Race_Fail(&s_race, RACE_ERROR_MOVE_TIMEOUT);
            return;
        }
        if (hypotf(f->x - s_align_target_x, f->y - s_align_target_y) < RACE_POSITION_TOL_M &&
            hypotf(f->vx, f->vy) < RACE_STILL_SPEED_MPS &&
            fabsf(f->omega) < RACE_STILL_OMEGA_RADPS) {
            if (!s_align_reached_ms) s_align_reached_ms = now;
            if ((uint32_t)(now - s_align_reached_ms) >= RACE_ALIGN_HOLD_MS) {
                s_flow = PICK_FLOW_ALIGN_WAIT;
                s_phase_since = now;
                s_handled_slot = s_slot_number;
                stop_outputs();
            }
        } else s_align_reached_ms = 0U;
    } else if (s_flow == PICK_FLOW_ARMING) {
        if ((uint32_t)(now - s_phase_since) > RACE_ALIGN_ARM_TIMEOUT_MS) {
            Race_Fail(&s_race, RACE_ERROR_LINK);
            return;
        }
        if (s_arm_ok) {
            s_flow = PICK_FLOW_WAIT_ITEM;
            s_phase_since = now;
        } else if ((uint32_t)(now - s_arm_sent_ms) >= RACE_ARM_RETRY_MS) {
            if (RaceVision_Send("#ARM,1\n")) s_arm_sent_ms = now;
        }
    } else if (s_flow == PICK_FLOW_PICKING) {
        retry_take(now);
        if (s_pick.state == PICK_FAULT || s_pick.state == PICK_ABORTED) {
            Race_Fail(&s_race, RACE_ERROR_CONFIG);
        } else if (PickFsm_IsDone(&s_pick)) {
            s_flow = PICK_FLOW_CONFIRM;
            s_phase_since = now;
        }
    } else if (s_flow == PICK_FLOW_CONFIRM) {
        if (!s_take_ok) {
            if ((uint32_t)(now - s_phase_since) > RACE_ALIGN_ARM_TIMEOUT_MS) {
                Race_Fail(&s_race, RACE_ERROR_LINK);
                return;
            }
            retry_take(now);
            return;
        }
        const TaskAction *item = RaceTaskPlan_Current(&s_plan);
        if (item && item->kind == TASK_ACTION_PICK && RaceVision_Send("K\n")) {
            (void)RaceTaskPlan_Advance(&s_plan);
            PickFsm_Init(&s_pick, &s_pick_config);  /* finished, so ready for next */
            if (RaceTaskPlan_Current(&s_plan) &&
                RaceTaskPlan_Current(&s_plan)->kind == TASK_ACTION_PICK) {
                s_arm_ok = 0U;
                s_flow = PICK_FLOW_ARMING;
                s_arm_sent_ms = now - RACE_ARM_RETRY_MS;
            } else {
                s_flow = PICK_FLOW_GROUP_DONE;
            }
        }
    } else if (s_flow == PICK_FLOW_GROUP_DONE) {
        if (RaceVision_Send("#GROUP_DONE,1\n")) {
            s_flow = PICK_FLOW_COMPLETE;
            stop_outputs();
        }
    }
}

void AppRace_Poll(void)
{
    uint32_t now = HAL_GetTick();
    uint32_t frames = app_imu_frames();
    RaceState previous_state = s_race.state;
    uint8_t previous_index = s_race.action_index;
    uint32_t saved_irq;
    RaceFeedback current;
    RaceVisionEvent event;
    uint8_t budget = 16U;
    if (frames != s_frames) { s_frames = frames; s_last_imu = now; }
    current = feedback(now);
    if (s_stalled) Race_Fail(&s_race, RACE_ERROR_LOOP_STALL);
    if (!RaceVision_Healthy() && (s_race.state == RACE_READY ||
        s_race.state == RACE_RUNNING || pick_flow_running())) {
        Race_Fail(&s_race, RACE_ERROR_LINK);
    }
    while (budget-- && RaceVision_PollEvent(&event)) {
        accept_vision_event(&event, now);
    }
    poll_key(&current, now);
    Race_Tick(&s_race, &current, now);
    if (PickFsm_IsActive(&s_pick) || s_pick.state == PICK_RESETTING)
        PickFsm_Poll(&s_pick, now);
    if (s_race.state == RACE_DONE) poll_material_flow(&current, now);
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
        if (action->kind == RACE_ACTION_SCAN && !RaceVision_BeginScan())
            Race_Fail(&s_race, RACE_ERROR_LINK);
    }
    saved_irq = __get_PRIMASK();
    __disable_irq();
    if (s_stalled) Race_Fail(&s_race, RACE_ERROR_LOOP_STALL);
    s_active = (s_race.state == RACE_RUNNING) ||
               pick_flow_running() || PickFsm_IsActive(&s_pick);
    s_last_poll = now;
    if (s_race.state == RACE_FAULT || s_race.state == RACE_ABORTED ||
        (s_race.state == RACE_DONE && s_flow == PICK_FLOW_COMPLETE)) {
        if (s_control) stop_outputs();
        s_control = 0U;
        s_active = 0U;
        if (PickFsm_IsActive(&s_pick)) PickFsm_Abort(&s_pick);
    }
    __set_PRIMASK(saved_irq);
    if (s_race.state == RACE_FAULT || s_race.state == RACE_ABORTED)
        RaceVision_EndScan();
    poll_display(now);
}

const RaceFsm *AppRace_Status(void)
{
    return &s_race;
}

const RaceTaskPlan *AppRace_TaskPlan(void)
{
    return &s_plan;
}

uint8_t AppRace_LogFinished(void)
{
    if (s_race.state == RACE_FAULT || s_race.state == RACE_ABORTED ||
        s_pick.state == PICK_FAULT || s_pick.state == PICK_ABORTED) return 1U;
    if (PickFsm_IsActive(&s_pick) ||
        AppStepper_IsBusy(STEPPER_AXIS_1) ||
        Servo_IsBusy(SERVO_1) || Servo_IsBusy(SERVO_2) ||
        Servo_IsBusy(SERVO_3)) return 0U;
    return s_race.state == RACE_DONE && s_flow == PICK_FLOW_COMPLETE;
}
#endif
