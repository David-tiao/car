#include "pi_motion_adapter.h"
#include "race_config.h"
#include <math.h>
#include <string.h>

#define RX_CAP 256u
#define POLL_BUDGET 128u
#define MAIN_STALL_MS 100u
#define DEG_MILLI_PER_RAD 57295.77951308232f
#define RAD_PER_DEG_MILLI 0.0000174532925199433f
static MecanumChassis_t *s_chassis;
static PiMotionTx s_tx;
static MLCore s_core;
static MLStream s_parser;
static MLFeedback s_sample;
static volatile uint8_t s_rx[RX_CAP];
static volatile uint16_t s_head, s_tail;
static volatile uint8_t s_ready, s_io_fault, s_latched, s_imu_valid;
static volatile uint32_t s_poll_ms, s_imu_ms;
static uint32_t s_issued; /* last applied action, not serial sequence */
static uint8_t s_output_active;

static void stop_locked(void)
{
    Mecanum_Stop(s_chassis);
    Motor_Stop(s_chassis->motor_lf); Motor_Stop(s_chassis->motor_rf);
    Motor_Stop(s_chassis->motor_lb); Motor_Stop(s_chassis->motor_rb);
    s_output_active = 0u;
}
static void fault_locked(uint8_t why)
{
    if (!s_latched) {
        s_latched = 1u;
        s_core.state = ML_FAULT; s_core.reason = why; s_core.count = 0u;
        stop_locked();
    }
}
static uint8_t fresh_imu(uint32_t now)
{
    return s_imu_valid && (uint32_t)(now-s_imu_ms) < RACE_IMU_STALE_MS;
}
static void guard_locked(uint32_t now, uint8_t enable)
{
    if (s_latched) return;
    if (s_io_fault) { fault_locked(ML_INTERLOCK); return; }
    if (!s_core.session) return;
    if (enable != 1u || !s_chassis->enable) { fault_locked(ML_INTERLOCK); return; }
    if ((uint32_t)(now-s_poll_ms) >= MAIN_STALL_MS ||
        (uint32_t)(now-s_core.last_rx) >= ML_WATCHDOG_MS) { fault_locked(ML_TIMEOUT); return; }
    if (!fresh_imu(now) || !s_sample.valid || (uint32_t)(now-s_sample.sample_ms)>ML_FEEDBACK_MS)
        fault_locked(ML_FEEDBACK);
}
static void sync_locked(void)
{
    int32_t goal[3];
    if (s_core.state == ML_FAULT) { fault_locked(s_core.reason); return; }
    if (!ml_intent(&s_core, goal)) {
        if (s_output_active) stop_locked();
        return;
    }
    if (s_latched || s_core.action_seq == s_issued) return;
    /* Exactly once per action; position controller uses odom frame. */
    s_chassis->body_frame = 0u;
    Mecanum_SetPositionMode(s_chassis, goal[0]*0.001f, goal[1]*0.001f,
                            goal[2]*RAD_PER_DEG_MILLI);
    s_issued = s_core.action_seq; s_output_active = 1u;
}
int PiMotion_Init(MecanumChassis_t *chassis, PiMotionTx tx, uint32_t now)
{
    if (!PI_MOTION_ENABLE || s_ready || !chassis || !tx || !chassis->motor_lf ||
        !chassis->motor_rf || !chassis->motor_lb || !chassis->motor_rb) return 0;
    uint32_t irq=__get_PRIMASK(); __disable_irq();
    s_chassis=chassis; s_tx=tx; ml_init(&s_core); ml_stream_init(&s_parser);
    memset(&s_sample,0,sizeof(s_sample)); s_head=s_tail=0u; s_io_fault=s_latched=s_imu_valid=0u;
    s_poll_ms=s_imu_ms=now; s_issued=0u; s_output_active=0u;
    Mecanum_SetSpeedLimit(chassis,RACE_MOVE_SPEED_MPS,RACE_TURN_SPEED_RADPS);
    /* PID tolerance must not be looser than the independent completion gate. */
    Mecanum_SetTolerance(chassis, ML_POS_TOL_MM*0.0005f, ML_YAW_TOL_MDEG*RAD_PER_DEG_MILLI*0.5f);
    stop_locked(); s_ready=1u; __set_PRIMASK(irq); return 1;
}
int PiMotion_LocalGrant(uint32_t session, uint8_t enable, uint32_t now)
{
    if (!s_ready) return 0;
    uint32_t irq=__get_PRIMASK(); __disable_irq();
    int granted=0;
    if (!s_latched && !s_io_fault && !s_core.session && enable==1u && s_chassis->enable &&
        fresh_imu(now) && s_sample.valid && s_sample.stationary &&
        (uint32_t)(now-s_sample.sample_ms)<=ML_FEEDBACK_MS &&
        (uint32_t)(now-s_poll_ms)<MAIN_STALL_MS) {
        /* Discard pre-grant bytes and parser state. A remote frame cannot grant. */
        s_tail=s_head; ml_stream_init(&s_parser);
        granted=ml_grant(&s_core,session,enable,now);
    }
    __set_PRIMASK(irq); return granted;
}
void PiMotion_Capture(uint32_t now)
{
    if (!s_ready || s_latched) return;
    /* ISR is sole writer of feedback. Main copies under IRQ mask. */
    MLFeedback fb={0}; fb.sample_ms=now;
    float x=s_chassis->x_pos, y=s_chassis->y_pos, h=s_chassis->theta_pos;
    float vx=s_chassis->vx_actual, vy=s_chassis->vy_actual, w=s_chassis->omega_actual;
    float wheel[4]={s_chassis->motor_lf->actual_speed,s_chassis->motor_rf->actual_speed,
                    s_chassis->motor_lb->actual_speed,s_chassis->motor_rb->actual_speed};
    uint8_t wheels_still=1u;
    for(unsigned i=0;i<4u;i++){
        if(!isfinite(wheel[i])) { s_sample=fb; return; }
        if(fabsf(wheel[i])*s_chassis->pulse_to_meter>=RACE_STILL_SPEED_MPS)wheels_still=0u;
    }
    if (fresh_imu(now) && s_chassis->enable && s_chassis->imu_snapshot.valid &&
        isfinite(s_chassis->pulse_to_meter) && s_chassis->pulse_to_meter>0.0f &&
        isfinite(x)&&isfinite(y)&&isfinite(h)&&isfinite(vx)&&isfinite(vy)&&isfinite(w)&&
        fabsf(x)<=10.0f&&fabsf(y)<=10.0f&&fabsf(h)<=3.142f) {
        fb.x=(int32_t)lroundf(x*1000.0f); fb.y=(int32_t)lroundf(y*1000.0f);
        fb.yaw=(int32_t)lroundf(h*DEG_MILLI_PER_RAD);
        if(fb.yaw>=180000)fb.yaw-=360000;
        if(fb.yaw< -180000)fb.yaw+=360000;
        fb.valid=1u;
        fb.stationary=wheels_still&&hypotf(vx,vy)<RACE_STILL_SPEED_MPS&&fabsf(w)<RACE_STILL_OMEGA_RADPS;
    }
    s_sample=fb;
}
uint8_t PiMotion_ControlTick(uint32_t now, uint8_t enable)
{
    if (!s_ready) return 0u;
    /* Also guard in TIM6: main-loop stall or RX error cannot keep old targets. */
    guard_locked(now,enable);
    return !s_latched;
}
void PiMotion_RxByte(uint8_t b)
{
    if (!s_ready) return;
    uint16_t next=(uint16_t)((s_head+1u)%RX_CAP);
    if(next==s_tail){s_io_fault=1u;return;}
    s_rx[s_head]=b; s_head=next;
}
void PiMotion_IoFault(void){if(s_ready)s_io_fault=1u;}
void PiMotion_Poll(uint32_t now,uint8_t enable,uint8_t imu_valid,uint32_t imu_sample_ms)
{
    if(!s_ready)return;
    uint32_t irq=__get_PRIMASK();__disable_irq();
    /* Guard the previous poll timestamp BEFORE refreshing it. */
    guard_locked(now,enable);
    s_poll_ms=now;s_imu_valid=imu_valid==1u;s_imu_ms=imu_sample_ms;
    guard_locked(now,enable);
    if(s_core.session&&!s_latched){ml_tick(&s_core,&s_sample,enable,now);sync_locked();}
    __set_PRIMASK(irq);
    for(unsigned i=0;i<POLL_BUDGET;i++){
        irq=__get_PRIMASK();__disable_irq();
        if(s_tail==s_head){__set_PRIMASK(irq);break;}
        uint8_t b=s_rx[s_tail];s_tail=(uint16_t)((s_tail+1u)%RX_CAP);
        __set_PRIMASK(irq);
        MLFrame f,r;
        if(!ml_stream_push(&s_parser,b,&f))continue;
        irq=__get_PRIMASK();__disable_irq();
        /* Use fresh wall-clock at dispatch; timer may have latched a stop. */
        uint32_t dispatch=HAL_GetTick();
        guard_locked(dispatch,enable);
        int reply=ml_receive(&s_core,&f,&s_sample,enable,dispatch,&r);
        sync_locked();
        if(reply&&s_latched){r.a[0]=ML_FAULT;r.a[5]=s_core.reason;}
        __set_PRIMASK(irq);
        if(reply){
            uint8_t wire[ML_SIZE];
            if(!ml_encode(&r,wire)||!s_tx(wire,ML_SIZE)){
                irq=__get_PRIMASK();__disable_irq();fault_locked(ML_INTERLOCK);__set_PRIMASK(irq);
                break;
            }
        }
    }
}
uint8_t PiMotion_State(void)
{
    uint32_t irq=__get_PRIMASK();__disable_irq();uint8_t v=s_core.state;__set_PRIMASK(irq);return v;
}
uint8_t PiMotion_FaultReason(void)
{
    uint32_t irq=__get_PRIMASK();__disable_irq();uint8_t v=s_core.reason;__set_PRIMASK(irq);return v;
}
