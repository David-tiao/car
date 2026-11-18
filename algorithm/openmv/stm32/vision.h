#ifndef PROJECT_VISION_H
#define PROJECT_VISION_H

#include <stdint.h>
#include <stddef.h>

#ifdef __cplusplus
extern "C" {
#endif

#define VISION_MAX_LINE 192
#define VISION_MAX_BODY 128
#define VISION_TARGET_TIMEOUT_MS 450u
#define VISION_CMD_TIMEOUT_MS 300u
#define VISION_MAX_TRACKS 3

typedef enum { VISION_PORT_OPENMV = 0, VISION_PORT_HOST = 1 } VisionPort;

typedef struct {
    uint16_t id;                 /* OpenMV Kalman track id */
    uint8_t color;               /* 1 red, 2 yellow, 3 blue, 4 green, 5 black, 6 lightblue */
    int32_t pixel_x, pixel_y, cap_radius_px;
    int32_t x_mm, y_mm, z_mm, distance_mm; /* camera optical frame */
    char state;                  /* 'U' uncertain; 'M' moving; 'S' still */
    uint8_t quality;            /* heuristic, NOT a probability */
    uint8_t metric_valid;       /* only 1 when calibrated on OpenMV */
    uint8_t seen;
    uint32_t updated_ms;
} VisionTarget;

typedef struct {
    uint8_t digit;              /* ring number 1, 2 or 3 */
    int32_t pixel_x, pixel_y;   /* 320x240 image coordinates */
    int32_t x_mm, y_mm, z_mm, distance_mm; /* camera optical frame */
    int32_t quality;            /* detector score, not a probability */
    uint8_t metric_valid;
    uint8_t seen;
    uint32_t updated_ms;
} VisionRing;

typedef struct {
    uint8_t locked;
    int32_t x_px, y_px, radius_px;
    uint32_t updated_ms;
} VisionDisk;

typedef struct {
    char line[VISION_MAX_LINE];
    uint16_t count;
    uint32_t bad_frames;
    uint32_t good_frames;
} VisionLineParser;

typedef void (*VisionTxFn)(const uint8_t *bytes, size_t n, void *user);
typedef void (*VisionVelocityFn)(int32_t vx_mm_s, int32_t vy_mm_s,
                                  int32_t wz_mrad_s, void *user);
typedef void (*VisionStopFn)(void *user);

typedef struct {
    VisionTxFn tx_openmv;
    VisionTxFn tx_host;
    VisionVelocityFn apply_velocity;  /* optional; no actual motor code supplied */
    VisionStopFn stop_motor;          /* REQUIRED for real vehicle safety */
    void *user;
    VisionLineParser rx_openmv, rx_host;
    uint16_t seq_openmv, seq_host;
    VisionDisk disk;
    VisionTarget tracks[VISION_MAX_TRACKS];
    VisionTarget target;
    VisionRing rings[3];
    uint8_t target_present;
    char task_code[20];
    uint8_t task_present;
    char current_mode[12];
    uint8_t estop_latched;
    uint8_t vel_active;
    uint32_t last_velocity_ms;
} VisionContext;

void Vision_Init(VisionContext *v, VisionTxFn to_openmv, VisionTxFn to_host,
                 VisionVelocityFn apply_velocity, VisionStopFn stop_motor,
                 void *user);
/* Call from main loop with bytes dequeued from UART RX IRQ, NOT from blocking ISR logic. */
void Vision_FeedByte(VisionContext *v, VisionPort port, uint8_t ch, uint32_t now_ms);
void Vision_Poll(VisionContext *v, uint32_t now_ms);

/* NULL when the target was not recently observed; coordinates are camera optical frame. */
const VisionTarget *Vision_GetTarget(const VisionContext *v, uint32_t now_ms);
/* NULL when metric calibration is missing. Do not issue metric moves without this. */
const VisionTarget *Vision_GetMetricTarget(const VisionContext *v, uint32_t now_ms);
/* digit must be 1..3; returns NULL if that numbered ring is stale. */
const VisionRing *Vision_GetRing(const VisionContext *v, uint8_t digit, uint32_t now_ms);
const VisionRing *Vision_GetMetricRing(const VisionContext *v, uint8_t digit, uint32_t now_ms);

/* Call ONLY after physically verifying the emergency-stop cause is cleared. */
void Vision_ClearEmergency(VisionContext *v);
/* Actual encoder odometry must be computed elsewhere; never send a fabricated odometry value. */
void Vision_SendOdom(VisionContext *v, int32_t x_mm, int32_t y_mm,
                     int32_t yaw_mrad, int32_t vx_mm_s, int32_t vy_mm_s,
                     int32_t wz_mrad_s);
/* For test/telemetry. Forwarding commands from ROS2 is already handled internally. */
int Vision_SendBody(VisionContext *v, VisionPort destination, const char *body);

#ifdef __cplusplus
}
#endif
#endif
