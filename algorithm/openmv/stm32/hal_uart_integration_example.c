/* STM32F4 <-> OpenMV H7 Plus: single-UART, no ROS 2 / host serial.
 * EXAMPLE for CubeMX HAL. Adapt USART and motor hooks to your board.
 * This is an integration template: do not compile alongside another
 * HAL_UART_RxCpltCallback; merge callback branches in your own firmware.
 *
 * OpenMV UART3 P4(TX) -> STM32 USART3 RX
 * OpenMV UART3 P5(RX) <- STM32 USART3 TX
 * GND -> GND; 3.3V IO; UART 115200 8N1.
 */
#include "main.h"
#include "usart.h"
#include "app_vision.h"
#include <string.h>
#include <stdio.h>

extern UART_HandleTypeDef huart3;  /* Adapt to actual CubeMX UART handle. */

/* Optional: implement a real hardware stop that sets motor drive to zero.
 * For bench-only (motors disconnected), this can be a documented no-op.
 * NEVER use an empty hook on a moving robot.
 */
extern void Robot_EmergencyStop(void);

#define MV_FIFO_SIZE 512u
static uint8_t mv_rx_byte;
static struct {
    volatile uint16_t head, tail;
    uint8_t data[MV_FIFO_SIZE];
} mv_fifo;
static VisionContext vision_ctx;

static void mv_tx(const uint8_t *bytes, size_t n, void *user) {
    (void)user;
    (void)HAL_UART_Transmit(&huart3, (uint8_t *)bytes, (uint16_t)n, 25);
}
static void stop_motor(void *user) {
    (void)user;
    Robot_EmergencyStop();
}
static void rx_put(uint8_t b) {
    uint16_t next = (uint16_t)((mv_fifo.head + 1u) & (MV_FIFO_SIZE - 1u));
    if (next == mv_fifo.tail) return;  /* resync at next '@' after overflow */
    mv_fifo.data[mv_fifo.head] = b;
    mv_fifo.head = next;
}
static int rx_get(void) {
    uint8_t b;
    if (mv_fifo.head == mv_fifo.tail) return -1;
    b = mv_fifo.data[mv_fifo.tail];
    mv_fifo.tail = (uint16_t)((mv_fifo.tail + 1u) & (MV_FIFO_SIZE - 1u));
    return (int)b;
}

/* Call after MX_USART3_UART_Init(). OpenMV starts in QR mode. */
void App_VisionInit(void) {
    memset(&mv_fifo, 0, sizeof(mv_fifo));
    Vision_Init(&vision_ctx, mv_tx, NULL, NULL, stop_motor, NULL);
    HAL_UART_Receive_IT(&huart3, &mv_rx_byte, 1);
    /* Can be called again after OpenMV power-cycles. */
    Vision_SendBody(&vision_ctx, VISION_PORT_OPENMV, "MODE,QR");
}

/* Merge with the existing HAL callback if your app already defines it. */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) {
    if (huart == &huart3) {
        rx_put(mv_rx_byte);
        HAL_UART_Receive_IT(&huart3, &mv_rx_byte, 1);
    }
}
void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart) {
    if (huart == &huart3) HAL_UART_Receive_IT(&huart3, &mv_rx_byte, 1);
}

/* Call every iteration of main while(1), ideally at least every 10–20 ms. */
void App_VisionPoll(void) {
    int b;
    uint32_t now = HAL_GetTick();
    while ((b = rx_get()) != -1)
        Vision_FeedByte(&vision_ctx, VISION_PORT_OPENMV, (uint8_t)b, now);
    Vision_Poll(&vision_ctx, now);
}

/* The STM32 decides when to change stages: QR -> MATERIAL -> RING etc. */
int App_VisionSetMode(const char *mode) {
    const char *body = NULL;
    if (!mode) return 0;
    if      (strcmp(mode, "QR") == 0)       body = "MODE,QR";
    else if (strcmp(mode, "MATERIAL") == 0) body = "MODE,MATERIAL";
    else if (strcmp(mode, "RING") == 0)     body = "MODE,RING";
    else if (strcmp(mode, "EDGE") == 0)     body = "MODE,EDGE";
    else if (strcmp(mode, "BLUE") == 0)     body = "MODE,BLUE";
    if (!body) return 0;
    return Vision_SendBody(&vision_ctx, VISION_PORT_OPENMV, body);
}
int App_VisionSetColor(uint8_t color) {
    char b[24];
    if (color > 6) return 0;
    (void)snprintf(b, sizeof(b), "COLOR,%u", (unsigned)color);
    return Vision_SendBody(&vision_ctx, VISION_PORT_OPENMV, b);
}
int App_VisionSetRing(uint8_t ring) {
    char b[24];
    if (ring > 3) return 0;
    (void)snprintf(b, sizeof(b), "RINGID,%u", (unsigned)ring);
    return Vision_SendBody(&vision_ctx, VISION_PORT_OPENMV, b);
}
int App_VisionGroupDone(uint8_t group) {
    char b[24];
    if (group < 1 || group > 4) return 0;
    (void)snprintf(b, sizeof(b), "DONE,%u", (unsigned)group);
    return Vision_SendBody(&vision_ctx, VISION_PORT_OPENMV, b);
}
int App_VisionPing(void) {
    return Vision_SendBody(&vision_ctx, VISION_PORT_OPENMV, "PING");
}
int App_VisionReset(void) {
    return Vision_SendBody(&vision_ctx, VISION_PORT_OPENMV, "RESET");
}

/* Read image pixel target even before camera calibration. NULL if stale/invalid. */
const VisionTarget *App_VisionGetTarget(void) {
    return Vision_GetTarget(&vision_ctx, HAL_GetTick());
}
/* Read 3D mm only after calibrating fx/fy and real material top diameter. */
const VisionTarget *App_VisionGetMetricTarget(void) {
    return Vision_GetMetricTarget(&vision_ctx, HAL_GetTick());
}
/* Ring digit 1..3: ring center px and calibrated mm when valid. */
const VisionRing *App_VisionGetRing(uint8_t digit) {
    return Vision_GetRing(&vision_ctx, digit, HAL_GetTick());
}
const VisionRing *App_VisionGetMetricRing(uint8_t digit) {
    return Vision_GetMetricRing(&vision_ctx, digit, HAL_GetTick());
}
/* Returns whether latest confirmed QR task is available and copies it. */
int App_VisionGetTask(char out[20]) {
    if (!out || !vision_ctx.task_present) return 0;
    memcpy(out, vision_ctx.task_code, 20);
    return 1;
}
int App_VisionDiskLocked(void) {
    return vision_ctx.disk.locked != 0;
}

/* Example main.c usage, insert at appropriate points (DO NOT copy whole main):
 *
 * MX_USART3_UART_Init();
 * App_VisionInit();
 * for (;;) {
 *     App_VisionPoll();
 *     // Once QR task received and robot has arrived at the material station:
 *     // App_VisionSetMode("MATERIAL");  // call ONCE on stage transition
 *     // App_VisionSetColor(0);          // 0=nearest target, 1..6=color
 *     const VisionTarget *t = App_VisionGetTarget();
 *     if (t) {
 *         int32_t px = t->pixel_x, py = t->pixel_y;
 *         // Valid pixels. Only use metric coordinates when t->metric_valid.
 *     }
 *     const VisionTarget *m = App_VisionGetMetricTarget();
 *     if (m && m->state == 'S') {
 *         int32_t cam_x_mm = m->x_mm;
 *         int32_t cam_y_mm = m->y_mm;
 *         int32_t cam_z_mm = m->z_mm;
 *         int32_t slant_mm = m->distance_mm;
 *         // Still requires camera->base calibration and gripper planning.
 *     }
 * }
 */
