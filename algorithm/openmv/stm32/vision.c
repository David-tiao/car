#include "vision.h"
#include <string.h>
#include <stdio.h>
#include <stdlib.h>
#include <limits.h>
#include <ctype.h>

static uint8_t checksum(const char *s, size_t n) {
    uint8_t x = 0;
    size_t i;
    for (i = 0; i < n; ++i) x ^= (uint8_t)s[i];
    return x;
}

int Vision_SendBody(VisionContext *v, VisionPort to, const char *body) {
    char prefix[VISION_MAX_LINE], frame[VISION_MAX_LINE];
    size_t body_len;
    int n, f;
    uint16_t *seq;
    VisionTxFn tx;
    if (!v || !body) return 0;
    body_len = strlen(body);
    if (body_len == 0 || body_len > VISION_MAX_BODY) return 0;
    if (strchr(body, '*') || strchr(body, '@') || strchr(body, '\n') ||
        strchr(body, '\r')) return 0;
    if (to == VISION_PORT_OPENMV) {
        seq = &v->seq_openmv;
        tx = v->tx_openmv;
    } else {
        seq = &v->seq_host;
        tx = v->tx_host;
    }
    if (!tx) return 0;
    n = snprintf(prefix, sizeof(prefix), "%u,%u,%s", (unsigned)*seq,
                 (unsigned)body_len, body);
    if (n <= 0 || n >= (int)sizeof(prefix)) return 0;
    f = snprintf(frame, sizeof(frame), "@%s*%02X\n", prefix,
                 (unsigned)checksum(prefix, (size_t)n));
    if (f <= 0 || f >= (int)sizeof(frame)) return 0;
    *seq = (uint16_t)(*seq + 1u);
    tx((const uint8_t *)frame, (size_t)f, v->user);
    return 1;
}

static int parse_u32(const char *s, uint32_t *out) {
    char *end;
    unsigned long n;
    if (!s || !*s || !out || *s == '-' || *s == '+') return 0;
    n = strtoul(s, &end, 10);
    if (*end || n > UINT32_MAX) return 0;
    *out = (uint32_t)n;
    return 1;
}

static int parse_i32(const char *s, int32_t *out) {
    char *end;
    long n;
    if (!s || !*s || !out) return 0;
    n = strtol(s, &end, 10);
    if (*end || n < INT32_MIN || n > INT32_MAX) return 0;
    *out = (int32_t)n;
    return 1;
}

/* Mutates buf, returns 0 if checksum/length/shape wrong. */
static int decode_frame(char *buf, char **body) {
    char *star, *comma1, *comma2, *p;
    uint32_t seq, nlen;
    unsigned chk;
    size_t actual;
    if (buf[0] != '@') return 0;
    star = strchr(buf, '*');
    if (!star || strlen(star + 1) != 2 || strchr(star + 1, '*')) return 0;
    if (!isxdigit((unsigned char)star[1]) || !isxdigit((unsigned char)star[2]) ||
        sscanf(star + 1, "%2x", &chk) != 1 || chk > 255) return 0;
    if (checksum(buf + 1, (size_t)(star - (buf + 1))) != (uint8_t)chk) return 0;
    *star = '\0';
    comma1 = strchr(buf + 1, ',');
    if (!comma1) return 0;
    *comma1 = '\0';
    if (!parse_u32(buf + 1, &seq) || seq > 65535u) return 0;
    comma2 = strchr(comma1 + 1, ',');
    if (!comma2) return 0;
    *comma2 = '\0';
    if (!parse_u32(comma1 + 1, &nlen) || nlen > VISION_MAX_BODY) return 0;
    p = comma2 + 1;
    actual = strlen(p);
    if (actual == 0 || actual != nlen) return 0;
    *body = p;
    return 1;
}

static int split_csv(char *body, char **fields, int max) {
    int n = 0;
    char *p = body, *q;
    while (p && n < max) {
        if (!*p) return 0; /* reject empty fields */
        fields[n++] = p;
        q = strchr(p, ',');
        if (!q) return n;
        *q = '\0';
        p = q + 1;
    }
    return 0; /* too many fields */
}

static int parse_target(char **f, int n, VisionTarget *t, uint32_t now) {
    int32_t tmp[9];
    int i;
    uint32_t id, color;
    if (n != 13 || !t || !parse_u32(f[1], &id) || id > 65535u ||
        !parse_u32(f[2], &color) || color < 1 || color > 6) return 0;
    /* tokens 3..9 are px,py,r,x,y,z,d. */
    for (i = 0; i < 7; ++i) if (!parse_i32(f[i + 3], &tmp[i])) return 0;
    if (tmp[0] < 0 || tmp[0] >= 320 || tmp[1] < 0 || tmp[1] >= 240 ||
        tmp[2] < 1 || tmp[2] > 120) return 0;
    if (strlen(f[10]) != 1 || !strchr("UMS", f[10][0]) ||
        !parse_i32(f[11], &tmp[7]) || tmp[7] < 0 || tmp[7] > 100 ||
        !parse_i32(f[12], &tmp[8]) || (tmp[8] != 0 && tmp[8] != 1)) return 0;
    if (tmp[8] == 1 && (tmp[5] <= 0 || tmp[6] <= 0)) return 0;
    memset(t, 0, sizeof(*t));
    t->id = (uint16_t)id;
    t->color = (uint8_t)color;
    t->pixel_x = tmp[0]; t->pixel_y = tmp[1]; t->cap_radius_px = tmp[2];
    t->x_mm = tmp[3]; t->y_mm = tmp[4]; t->z_mm = tmp[5];
    t->distance_mm = tmp[6]; t->state = f[10][0];
    t->quality = (uint8_t)tmp[7]; t->metric_valid = (uint8_t)tmp[8];
    t->seen = 1;
    t->updated_ms = now;
    return 1;
}

static int parse_ring(char **f, int n, VisionRing *ring, uint32_t now) {
    int32_t a[9];
    int i;
    if (!ring || n != 10) return 0;
    for (i = 0; i < 9; ++i)
        if (!parse_i32(f[i + 1], &a[i])) return 0;
    if (a[0] < 1 || a[0] > 3 || a[1] < 0 || a[1] >= 320 ||
        a[2] < 0 || a[2] >= 240 || a[7] < 0 || a[7] > 10000 ||
        (a[8] != 0 && a[8] != 1)) return 0;
    if (a[8] == 1 && (a[5] <= 0 || a[6] <= 0)) return 0;
    memset(ring, 0, sizeof(*ring));
    ring->digit = (uint8_t)a[0];
    ring->pixel_x = a[1]; ring->pixel_y = a[2];
    ring->x_mm = a[3]; ring->y_mm = a[4];
    ring->z_mm = a[5]; ring->distance_mm = a[6];
    ring->quality = a[7]; ring->metric_valid = (uint8_t)a[8];
    ring->seen = 1; ring->updated_ms = now;
    return 1;
}

static int valid_task_code(const char *code) {
    int g, j;
    if (!code || strlen(code) != 15) return 0;
    for (g = 0; g < 4; ++g) {
        int base = g * 4;
        if (g != 3 && code[base + 3] != '+') return 0;
        for (j = 0; j < 3; ++j) {
            char c = code[base + j];
            if (c < '1' || c > (g % 2 ? '3' : '6')) return 0;
            if (j && (c == code[base] || (j == 2 && c == code[base + 1])))
                return 0;
        }
    }
    return 1;
}

static void emergency_stop(VisionContext *v) {
    v->estop_latched = 1;
    v->vel_active = 0;
    if (v->stop_motor) v->stop_motor(v->user);
}

static void from_openmv(VisionContext *v, char *body, uint32_t now_ms) {
    char copy[VISION_MAX_BODY + 1], forward[VISION_MAX_BODY + 1];
    char *f[20];
    int n, i;
    VisionTarget temp;
    VisionRing temp_ring;
    int32_t x, y, r;
    strncpy(copy, body, sizeof(copy) - 1);
    copy[sizeof(copy) - 1] = '\0';
    n = split_csv(copy, f, 20);
    if (!n) return;
    if (strcmp(f[0], "DISK") == 0 && n == 5) {
        if (strcmp(f[1], "0") == 0) {
            v->disk.locked = 0;
            v->target_present = 0;
        } else if (strcmp(f[1], "1") == 0 && parse_i32(f[2], &x) &&
                   parse_i32(f[3], &y) && parse_i32(f[4], &r) && r > 0) {
            v->disk.locked = 1;
            v->disk.x_px = x; v->disk.y_px = y; v->disk.radius_px = r;
        } else return;
        v->disk.updated_ms = now_ms;
    } else if (strcmp(f[0], "TARGET") == 0) {
        if (v->disk.locked && parse_target(f, n, &temp, now_ms)) {
            v->target = temp;
            v->target_present = 1;
        } else return; /* prevent forwarding unverified target */
    } else if (strcmp(f[0], "MAT") == 0) {
        if (v->disk.locked && parse_target(f, n, &temp, now_ms)) {
            for (i = 0; i < VISION_MAX_TRACKS; ++i) {
                if (!v->tracks[i].seen || v->tracks[i].id == temp.id) break;
            }
            if (i == VISION_MAX_TRACKS) {
                /* Replace the oldest track when three IDs already exist. */
                i = 0;
                if ((uint32_t)(now_ms - v->tracks[1].updated_ms) >
                    (uint32_t)(now_ms - v->tracks[i].updated_ms)) i = 1;
                if ((uint32_t)(now_ms - v->tracks[2].updated_ms) >
                    (uint32_t)(now_ms - v->tracks[i].updated_ms)) i = 2;
            }
            v->tracks[i] = temp;
        } else return;
    } else if (strcmp(f[0], "RING") == 0) {
        if (!parse_ring(f, n, &temp_ring, now_ms)) return;
        v->rings[temp_ring.digit - 1] = temp_ring;
    } else if (strcmp(f[0], "NONE") == 0 && n == 2 &&
               strcmp(f[1], "RING") == 0) {
        for (i = 0; i < 3; ++i) v->rings[i].seen = 0;
    } else if (strcmp(f[0], "NONE") == 0 && n == 2 &&
               strcmp(f[1], "TARGET") == 0) {
        v->target_present = 0;
    } else if (strcmp(f[0], "QR") == 0 && n == 2) {
        /* Prior OpenMV parser validates the task code 4x3. Still bound length. */
        if (valid_task_code(f[1])) {
            strcpy(v->task_code, f[1]);
            v->task_present = 1;
        } else return;
    } else if (strcmp(f[0], "STOP") == 0 && n == 2) {
        emergency_stop(v);
        Vision_SendBody(v, VISION_PORT_HOST, "ESTOP,EDGE");
    } else if (strcmp(f[0], "STATUS") == 0 && n == 6) {
        strncpy(v->current_mode, f[1], sizeof(v->current_mode)-1);
        v->current_mode[sizeof(v->current_mode)-1] = '\0';
    }
    /* Forward a validated recognized target, and informational updates. */
    if (strlen(body) + 4 <= VISION_MAX_BODY) {
        snprintf(forward, sizeof(forward), "VIS,%s", body);
        Vision_SendBody(v, VISION_PORT_HOST, forward);
    }
}

static void from_host(VisionContext *v, char *body, uint32_t now_ms) {
    char copy[VISION_MAX_BODY+1], *f[12];
    int n;
    int32_t vx, vy, wz, id;
    strncpy(copy, body, sizeof(copy)-1);
    copy[sizeof(copy)-1] = '\0';
    n = split_csv(copy, f, 12);
    if (n == 4 && strcmp(f[0], "VEL") == 0) {
        if (!parse_i32(f[1], &vx) || !parse_i32(f[2], &vy) ||
            !parse_i32(f[3], &wz) || vx < -800 || vx > 800 ||
            vy < -800 || vy > 800 || wz < -3000 || wz > 3000) {
            Vision_SendBody(v, VISION_PORT_HOST, "ERR,BAD_VEL");
            return;
        }
        if (v->estop_latched) {
            Vision_SendBody(v, VISION_PORT_HOST, "ERR,ESTOP_LATCHED");
            return;
        }
        v->last_velocity_ms = now_ms;
        v->vel_active = 1;
        if (v->apply_velocity) v->apply_velocity(vx, vy, wz, v->user);
    } else if (n == 1 && strcmp(f[0], "ESTOP") == 0) {
        emergency_stop(v);
        Vision_SendBody(v, VISION_PORT_HOST, "ACK,ESTOP");
    } else if (n == 2 && strcmp(f[0], "MODE") == 0 &&
               (strcmp(f[1], "QR") == 0 || strcmp(f[1], "MATERIAL") == 0 ||
                strcmp(f[1], "RING") == 0 || strcmp(f[1], "EDGE") == 0 ||
                strcmp(f[1], "BLUE") == 0)) {
        Vision_SendBody(v, VISION_PORT_OPENMV, body);
    } else if (n == 2 &&
               (strcmp(f[0], "COLOR") == 0 || strcmp(f[0], "RINGID") == 0 ||
                strcmp(f[0], "DONE") == 0) && parse_i32(f[1], &id) &&
               ((strcmp(f[0], "COLOR") == 0 && id >= 0 && id <= 6) ||
                (strcmp(f[0], "RINGID") == 0 && id >= 0 && id <= 3) ||
                (strcmp(f[0], "DONE") == 0 && id >= 1 && id <= 4))) {
        Vision_SendBody(v, VISION_PORT_OPENMV, body);
    } else if (n == 1 && (strcmp(f[0], "RESET") == 0 ||
                          strcmp(f[0], "PING") == 0)) {
        Vision_SendBody(v, VISION_PORT_OPENMV, body);
    } else {
        Vision_SendBody(v, VISION_PORT_HOST, "ERR,BAD_COMMAND");
    }
}

void Vision_Init(VisionContext *v, VisionTxFn to_openmv, VisionTxFn to_host,
                 VisionVelocityFn apply_velocity, VisionStopFn stop_motor,
                 void *user) {
    memset(v, 0, sizeof(*v));
    v->tx_openmv = to_openmv;
    v->tx_host = to_host;
    v->apply_velocity = apply_velocity;
    v->stop_motor = stop_motor;
    v->user = user;
    strcpy(v->current_mode, "UNKNOWN");
    if (stop_motor) stop_motor(user);
}

void Vision_FeedByte(VisionContext *v, VisionPort port, uint8_t ch, uint32_t now_ms) {
    VisionLineParser *p;
    char *body;
    if (!v) return;
    p = (port == VISION_PORT_OPENMV) ? &v->rx_openmv : &v->rx_host;
    if (ch == '@') {
        p->line[0] = '@';
        p->count = 1;
    } else if (ch == '\n') {
        if (p->count && p->count < VISION_MAX_LINE) {
            p->line[p->count] = '\0';
            if (decode_frame(p->line, &body)) {
                ++p->good_frames;
                if (port == VISION_PORT_OPENMV) from_openmv(v, body, now_ms);
                else from_host(v, body, now_ms);
            } else ++p->bad_frames;
        }
        p->count = 0;
    } else if (ch == '\r') {
        /* ignore CR */
    } else if (p->count) {
        if (ch < 32 || ch > 126 || p->count >= VISION_MAX_LINE - 1) {
            p->count = 0;
            ++p->bad_frames;
        } else p->line[p->count++] = (char)ch;
    }
}

const VisionTarget *Vision_GetTarget(const VisionContext *v, uint32_t now_ms) {
    if (!v || !v->disk.locked || !v->target_present ||
        (uint32_t)(now_ms - v->target.updated_ms) > VISION_TARGET_TIMEOUT_MS)
        return NULL;
    return &v->target;
}

const VisionTarget *Vision_GetMetricTarget(const VisionContext *v, uint32_t now_ms) {
    const VisionTarget *t = Vision_GetTarget(v, now_ms);
    return (t && t->metric_valid) ? t : NULL;
}

const VisionRing *Vision_GetRing(const VisionContext *v, uint8_t digit, uint32_t now_ms) {
    const VisionRing *r;
    if (!v || digit < 1 || digit > 3) return NULL;
    r = &v->rings[digit - 1];
    if (!r->seen || (uint32_t)(now_ms - r->updated_ms) > VISION_TARGET_TIMEOUT_MS)
        return NULL;
    return r;
}

const VisionRing *Vision_GetMetricRing(const VisionContext *v, uint8_t digit, uint32_t now_ms) {
    const VisionRing *r = Vision_GetRing(v, digit, now_ms);
    return (r && r->metric_valid) ? r : NULL;
}

void Vision_Poll(VisionContext *v, uint32_t now_ms) {
    size_t i;
    if (!v) return;
    if (v->vel_active &&
        (uint32_t)(now_ms - v->last_velocity_ms) > VISION_CMD_TIMEOUT_MS) {
        v->vel_active = 0;
        if (v->stop_motor) v->stop_motor(v->user);
        Vision_SendBody(v, VISION_PORT_HOST, "WATCHDOG,STOP");
    }
    if ((uint32_t)(now_ms - v->target.updated_ms) > VISION_TARGET_TIMEOUT_MS)
        v->target_present = 0;
    if (v->disk.locked &&
        (uint32_t)(now_ms - v->disk.updated_ms) > 1800u) {
        v->disk.locked = 0;
        v->target_present = 0;
    }
    for (i = 0; i < VISION_MAX_TRACKS; ++i)
        if (v->tracks[i].seen &&
            (uint32_t)(now_ms - v->tracks[i].updated_ms) > VISION_TARGET_TIMEOUT_MS)
            v->tracks[i].seen = 0;
    for (i = 0; i < 3; ++i)
        if (v->rings[i].seen &&
            (uint32_t)(now_ms - v->rings[i].updated_ms) > VISION_TARGET_TIMEOUT_MS)
            v->rings[i].seen = 0;
}

void Vision_ClearEmergency(VisionContext *v) {
    if (!v) return;
    if (v->stop_motor) v->stop_motor(v->user);
    v->estop_latched = 0;
    v->vel_active = 0;
}

void Vision_SendOdom(VisionContext *v, int32_t x_mm, int32_t y_mm,
                     int32_t yaw_mrad, int32_t vx_mm_s, int32_t vy_mm_s,
                     int32_t wz_mrad_s) {
    char body[VISION_MAX_BODY+1];
    int n = snprintf(body, sizeof(body), "ODOM,%ld,%ld,%ld,%ld,%ld,%ld",
                     (long)x_mm, (long)y_mm, (long)yaw_mrad,
                     (long)vx_mm_s, (long)vy_mm_s, (long)wz_mrad_s);
    if (n > 0 && n < (int)sizeof(body)) Vision_SendBody(v, VISION_PORT_HOST, body);
}
