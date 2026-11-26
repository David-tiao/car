#ifndef CAR_MOTION_LINK_REVIEW_H
#define CAR_MOTION_LINK_REVIEW_H
#include <stdint.h>
#include <stddef.h>
/* Host-test-only reference. No HAL, serial, motor or actuator calls. */
#define ML_SIZE 38u
#define ML_ARM 1u
#define ML_HEARTBEAT 2u
#define ML_TRANSLATE 3u
#define ML_TURN 4u
#define ML_STOP 5u
#define ML_STATUS 128u
#define ML_DISARMED 0u
#define ML_IDLE 1u
#define ML_MOVING 2u
#define ML_DONE 3u
#define ML_FAULT 4u
#define ML_OK 0u
#define ML_REJECT 1u
#define ML_TIMEOUT 2u
#define ML_FEEDBACK 3u
#define ML_INTERLOCK 4u
#define ML_STOPPED 5u
#define ML_BUSY 6u
#define ML_BADSEQ 7u
#define ML_BADARG 8u
#define ML_MOVE_TIMEOUT 9u
/* Engineering test limits, NOT real chassis calibration. */
#define ML_WATCHDOG_MS 500u
#define ML_MOVE_MS 8000u
#define ML_FEEDBACK_MS 100u
#define ML_MAX_STEP_MM 300
#define ML_POS_TOL_MM 8
#define ML_YAW_TOL_MDEG 3000
#define ML_STILL_MS 150u
#define ML_STILL_COUNT 4u

typedef struct { uint8_t kind; uint32_t session, seq; int32_t a[6]; } MLFrame;
typedef struct {
    int32_t x, y, yaw; /* odom mm, millidegrees; yaw [-180000,180000) */
    uint32_t sample_ms; /* local MCU clock */
    uint8_t valid, stationary; /* must come from measured feedback, not command */
} MLFeedback;
typedef struct {
    uint32_t session, last_session, last_seq, action_seq, last_rx, started, still_since, last_sample;
    uint8_t state, reason, count, have_sample, last_reason;
    int32_t gx, gy, gyaw;
    MLFrame last_command;
} MLCore;
typedef struct { uint8_t bytes[ML_SIZE]; uint8_t n; } MLStream;
void ml_stream_init(MLStream *s);
int ml_stream_push(MLStream *s,uint8_t b,MLFrame *out);
uint16_t ml_crc(const uint8_t *p, size_t n);
int ml_encode(const MLFrame *f, uint8_t out[ML_SIZE]);
int ml_decode(const uint8_t in[ML_SIZE], MLFrame *f);
void ml_init(MLCore *c);
/* Local, physical enable required. A fresh nonzero session must be provisioned
 * independently of incoming serial bytes. No remote re-arm after fault. */
int ml_grant(MLCore *c, uint32_t session, uint8_t local_enable, uint32_t now);
void ml_tick(MLCore *c, const MLFeedback *fb, uint8_t local_enable, uint32_t now);
/* Returns 1 with a status reply, 0 for invalid / unrelated session. */
int ml_receive(MLCore *c, const MLFrame *f, const MLFeedback *fb,
               uint8_t local_enable, uint32_t now, MLFrame *reply);
/* Pure intent only: caller must NOT wire to motors without separate review. */
int ml_intent(const MLCore *c, int32_t goal[3]);
#endif
