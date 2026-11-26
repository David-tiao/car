#include "app_runtime.h"
#include "app_mode.h"
#include "app_imu.h"
#include "app_race.h"
#include "app_servo_key.h"
#include "app_square.h"
#include "app_stepper_drive.h"
#include "app_stepper_openloop.h"
#include "bsp_d36_cmd.h"
#include "bsp_oled.h"
#include "bsp_servo.h"
#include "bsp_ulog.h"
#include "bsp_vbat.h"
#include "display_nav.h"
#include "mecanum_chassis.h"
#include "Motor.h"
#include "tim.h"
#include "vofa_debug.h"
#include <math.h>
#if APP_RUN_MODE == APP_MODE_PI_MOTION
#include "pi_uart2_port.h"
#endif
#define APP_RUNTIME_LEGACY (APP_RUN_MODE != APP_MODE_RACE && APP_RUN_MODE != APP_MODE_PI_MOTION)

Motor_t motor_left_front;
Motor_t motor_right_front;
Motor_t motor_left_back;
Motor_t motor_right_back;
MecanumChassis_t chassis;
#if APP_RUN_MODE == APP_MODE_PI_MOTION
static uint8_t s_pi_ready, s_pi_have_frame;
static uint32_t s_pi_frames, s_pi_sample_ms;

/* Physical authorization and independently provisioned session are not wired.
 * Even an explicitly compiled Pi mode cannot grant motion in this revision.
 * PE0 is NOT a verified hardware e-stop; never infer authorization from UART. */
static uint8_t pi_local_enable(void) { return 0U; }
#endif
#if APP_RUNTIME_LEGACY
static uint8_t s_square_seen_running;
#endif

#if APP_RUNTIME_LEGACY
static uint8_t d36_stepper_busy(uint8_t axis, void *user)
{ (void)user; return AppStepper_IsBusy((StepperAxis_t)axis); }
static uint32_t d36_stepper_remaining(uint8_t axis, void *user)
{ (void)user; return AppStepper_GetRemainingSteps((StepperAxis_t)axis); }
static HAL_StatusTypeDef d36_stepper_start(uint8_t axis, int8_t direction,
                                           uint32_t frequency_hz, void *user)
{ (void)user; return AppStepper_StartContinuous((StepperAxis_t)axis, direction, frequency_hz); }
static HAL_StatusTypeDef d36_stepper_move(uint8_t axis, int32_t steps,
                                          uint32_t frequency_hz, void *user)
{ (void)user; return AppStepper_MoveSteps((StepperAxis_t)axis, steps, frequency_hz); }
static void d36_stepper_stop(uint8_t axis, void *user)
{ (void)user; AppStepper_Stop((StepperAxis_t)axis); }
static void d36_stepper_stop_all(void *user)
{ (void)user; AppStepper_StopAll(); }
static void d36_stepper_enable(uint8_t axis, uint8_t enable, void *user)
{ (void)user; AppStepper_SetEnable((StepperAxis_t)axis, enable); }
static void d36_stepper_enable_all(uint8_t enable, void *user)
{ (void)user; AppStepper_SetEnableAll(enable); }

static const D36CmdStepperOps s_d36_stepper_ops = {
    d36_stepper_busy, d36_stepper_remaining, d36_stepper_start,
    d36_stepper_move, d36_stepper_stop, d36_stepper_stop_all,
    d36_stepper_enable, d36_stepper_enable_all
};
#endif

static void stop_chassis_outputs(void)
{
    Mecanum_Stop(&chassis);
    Motor_Stop(chassis.motor_lf);
    Motor_Stop(chassis.motor_rf);
    Motor_Stop(chassis.motor_lb);
    Motor_Stop(chassis.motor_rb);
}

static void update_chassis_imu_snapshot(void)
{
    NavImuSnapshot_t snapshot;

    snapshot.yaw_rad = app_imu_yaw_rad();
    snapshot.valid = app_imu_ok() ? 1u : 0u;
    MecanumChassis_SetImuSnapshot(&chassis, &snapshot);
}

#if APP_RUN_MODE != APP_MODE_PI_MOTION
static void update_ulog_snapshot(void)
{
    ULogAppSnapshot snapshot = {0};

    snapshot.imu_yaw_deg = app_imu_yaw_deg();
#if APP_RUN_MODE == APP_MODE_RACE
    {
        const RaceFsm *race = AppRace_Status();
        const RaceTaskPlan *plan = AppRace_TaskPlan();
        snapshot.status = (uint8_t)race->state;
        snapshot.action_index = race->action_index;
        snapshot.error = (uint8_t)race->error;
        snapshot.plan_index = plan->index;
        snapshot.plan_done = RaceTaskPlan_Done(plan);
        snapshot.finished = AppRace_LogFinished();
        if (snapshot.finished) snapshot.phase = 4U;
        else if (race->state != RACE_RUNNING ||
                 race->action_index >= race->action_count) snapshot.phase = 0U;
        else if (race->actions[race->action_index].kind == RACE_ACTION_TURN) snapshot.phase = 2U;
        else if (race->actions[race->action_index].kind == RACE_ACTION_SETTLE ||
                 race->actions[race->action_index].kind == RACE_ACTION_SCAN) snapshot.phase = 3U;
        else if (race->actions[race->action_index].kind == RACE_ACTION_MOVE) snapshot.phase = 1U;
    }
#else
    snapshot.status = Square_GetState();
    snapshot.action_index = Square_GetStep();
    if (snapshot.status == SQUARE_RUNNING) s_square_seen_running = 1U;
    snapshot.finished = (snapshot.status == SQUARE_DONE ||
                         snapshot.status == SQUARE_TIMEOUT ||
                         (s_square_seen_running && snapshot.status == SQUARE_IDLE));
    if (snapshot.status == SQUARE_DONE) snapshot.phase = 4U;
    else if (snapshot.status == SQUARE_RUNNING) {
        if (chassis.mode != 0U || chassis.target_reached != 0U ||
            fabsf(chassis.x_target) >= 1e-4f || fabsf(chassis.y_target) >= 1e-4f ||
            fabsf(chassis.vx_target) >= 1e-4f)
            snapshot.phase = ((snapshot.action_index & 1U) == 0U) ? 1U : 2U;
    }
#endif
    ULog_SetAppSnapshot(&snapshot);
}
#endif
static void init_motors_and_chassis(void)
{
    Motor_Init(&motor_left_front,
               &htim10, TIM_CHANNEL_1, 4199,
               &htim11, TIM_CHANNEL_1, 4199,
               &htim2, 2.5f, 1.60f, 0.0f, 1, 1);
    Motor_Init(&motor_right_front,
               &htim9, TIM_CHANNEL_1, 4199,
               &htim9, TIM_CHANNEL_2, 4199,
               &htim3, 2.5f, 1.60f, 0.0f, 1, -1);
    Motor_Init(&motor_left_back,
               &htim1, TIM_CHANNEL_1, 8399,
               &htim1, TIM_CHANNEL_2, 8399,
               &htim4, 2.5f, 1.60f, 0.0f, -1, 1);
    Motor_Init(&motor_right_back,
               &htim1, TIM_CHANNEL_3, 8399,
               &htim1, TIM_CHANNEL_4, 8399,
               &htim5, 2.5f, 1.60f, 0.0f, -1, -1);

    MecanumChassis_Init(&chassis,
                        &motor_left_front, &motor_right_front,
                        &motor_left_back, &motor_right_back,
                        0.04f, 0.093f, 0.11f, 1.0f, 440.0f);

    chassis.pid_x.kp = 1.6f;
    chassis.pid_x.ki = 0.05f;
    chassis.pid_x.kd = 0.12f;
    chassis.pid_x.integral_max = 0.20f;
    chassis.pid_x.output_max = 1.0f;
    chassis.pid_y.kp = 1.6f;
    chassis.pid_y.ki = 0.05f;
    chassis.pid_y.kd = 0.12f;
    chassis.pid_y.integral_max = 0.20f;
    chassis.pid_y.output_max = 1.0f;
    chassis.pid_theta.kp = 3.2f;
    chassis.pid_theta.ki = 0.08f;
    chassis.pid_theta.kd = 0.18f;
    chassis.pid_theta.integral_max = 0.50f;
    chassis.pid_theta.output_max = PI;
}

void AppRuntime_Init(void)
{
#if APP_RUN_MODE == APP_MODE_SERVO
    __HAL_TIM_MOE_DISABLE(&htim1);
    (void)HAL_TIM_PWM_Stop(&htim1, TIM_CHANNEL_1);
    (void)HAL_TIM_PWM_Stop(&htim1, TIM_CHANNEL_2);
    (void)HAL_TIM_PWM_Stop(&htim1, TIM_CHANNEL_3);
    (void)HAL_TIM_PWM_Stop(&htim1, TIM_CHANNEL_4);
    (void)HAL_TIM_PWM_Stop(&htim9, TIM_CHANNEL_1);
    (void)HAL_TIM_PWM_Stop(&htim9, TIM_CHANNEL_2);
    (void)HAL_TIM_PWM_Stop(&htim10, TIM_CHANNEL_1);
    (void)HAL_TIM_PWM_Stop(&htim11, TIM_CHANNEL_1);
    Servo_Init();
    ServoKey_Init();
#else
#if APP_RUNTIME_LEGACY
    AppStepper_Init();
    Servo_Init();
    Servo_SetBothAngle(30.0f, 30.0f);
    OLED_Init();
    D36Cmd_SetStepperOps(&s_d36_stepper_ops, 0);
    D36Cmd_Init();
#else
    OLED_Init();
#endif
    __HAL_TIM_MOE_ENABLE(&htim1);
    init_motors_and_chassis();
#if APP_RUNTIME_LEGACY
    DisplayNav_Init();
    VOFA_Init();
#endif
    Vbat_Init();
    if (app_imu_init() != 0) {
#if APP_RUN_MODE == APP_MODE_PI_MOTION
        stop_chassis_outputs();
        s_pi_ready = 0U;
        return;
#elif APP_RUNTIME_LEGACY
        Error_Handler();
#endif
    }
    update_chassis_imu_snapshot();
#if APP_RUN_MODE == APP_MODE_RACE
    ULog_Init();
    AppRace_Init(&chassis);
#elif APP_RUN_MODE == APP_MODE_PI_MOTION
    /* Dedicated USART2 owner; do not start AppRace, ULog, direct pick or Square. */
    s_pi_frames = app_imu_frames();
    s_pi_sample_ms = HAL_GetTick();
    s_pi_have_frame = 0U;
    s_pi_ready = (uint8_t)PiUart2_Init(&chassis, 1U, s_pi_sample_ms);
#endif
    stop_chassis_outputs();
#if APP_RUNTIME_LEGACY
#if (MOTOR_TUNE != 0)
    Tune_Start(&motor_left_front);
#else
    Square_Start(&chassis);
#endif
#endif
#endif
}

void AppRuntime_Poll(void)
{
#if APP_RUN_MODE == APP_MODE_SERVO
    ServoKey_Poll();
#elif APP_RUN_MODE == APP_MODE_RACE
    app_imu_poll();
    update_chassis_imu_snapshot();
    AppRace_Poll();
    Vbat_Poll();
    update_ulog_snapshot();
    ULog_Poll(&chassis);
#elif APP_RUN_MODE == APP_MODE_PI_MOTION
    uint32_t frames, now;
    app_imu_poll();
    update_chassis_imu_snapshot();
    frames = app_imu_frames();
    now = HAL_GetTick();
    if (frames != s_pi_frames) {
        s_pi_frames = frames;
        s_pi_sample_ms = now;
        s_pi_have_frame = 1U;
    }
    if (s_pi_ready)
        PiMotion_Poll(now, pi_local_enable(),
                      (uint8_t)(s_pi_have_frame && app_imu_ok() && app_imu_bias_ready()),
                      s_pi_sample_ms);
    Vbat_Poll();
#else
#if (MOTOR_TUNE != 0)
    Tune_Poll();
#else
    Square_Poll(&chassis);
#endif
    app_imu_poll();
    update_chassis_imu_snapshot();
    AppStepper_Poll();
    D36Cmd_Poll();
    Vbat_Poll();
    DisplayNav_Poll(&chassis);
    update_ulog_snapshot();
    ULog_Poll(&chassis);
#endif
}

uint8_t AppRuntime_ControlTick(void)
{
#if APP_RUN_MODE == APP_MODE_RACE
    return AppRace_ControlTick();
#elif APP_RUN_MODE == APP_MODE_PI_MOTION
    if (!s_pi_ready) return 0U;
    /* Run independent guard even while permission is off. No motor update or
     * odometry sampling until a verified local permission path is integrated. */
    if (!PiMotion_ControlTick(HAL_GetTick(), pi_local_enable())) return 0U;
    return pi_local_enable();
#else
    return 1U;
#endif
}

void AppRuntime_MotorTick(void)
{
    Motor_Update(&motor_left_front, 0.001f);
    Motor_Update(&motor_right_front, 0.001f);
    Motor_Update(&motor_left_back, 0.001f);
    Motor_Update(&motor_right_back, 0.001f);
}

void AppRuntime_ChassisTick(void)
{
#if APP_RUN_MODE == APP_MODE_PI_MOTION
    if (s_pi_ready && pi_local_enable()) {
        MecanumChassis_Loop(&chassis, 0.01f);
        PiMotion_Capture(HAL_GetTick());
    }
#else
    if (!g_tune_mode) MecanumChassis_Loop(&chassis, 0.01f);
#endif
}
