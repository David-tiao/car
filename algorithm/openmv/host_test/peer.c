/* Linux HAL shim: all protocol encoding, decoding and state belong to upstream vision.c. */
#include <stdint.h>
#include <string.h>
#include "vision.h"

static unsigned char tx[4096];
static unsigned used;
static uint32_t tick;
UART_HandleTypeDef huart3 = { USART3 };
uint32_t HAL_GetTick(void) { return tick; }
uint32_t __get_PRIMASK(void) { return 0; }
void __disable_irq(void) {}
void __set_PRIMASK(uint32_t mask) { (void)mask; }
int HAL_UART_Receive_IT(UART_HandleTypeDef *port, uint8_t *p, uint16_t n)
{ (void)port; (void)p; (void)n; return HAL_OK; }
int HAL_UART_Transmit_IT(UART_HandleTypeDef *port, uint8_t *p, uint16_t n)
{
    (void)port;
    if (n + used > sizeof(tx)) return HAL_ERROR;
    memcpy(tx + used, p, n);
    used += n;
    return HAL_OK;
}
void peer_boot(void) { used = 0; tick = 0; RaceVision_Init(); }
int peer_take(char *out) {
    while (Vision_GetDefault()->tx_active) Vision_TxCpltCallback(&huart3);
    memcpy(out, tx, used);
    int n = (int)used; used = 0; return n;
}
void peer_feed(const char *in, int size) {
    tick++;
    for (int i = 0; i < size; i++) Vision_RxByte(Vision_GetDefault(), (uint8_t)in[i], tick);
}
int peer_job(int code) { return Vision_StartJob(Vision_GetDefault(), code, tick); }
int peer_ring(int target) { return Vision_StartRingAlign(Vision_GetDefault(), target, tick); }
int peer_qr(char *out) { return RaceVision_Poll(out); }
int peer_idle(void) { return Vision_GetDefault()->phase == VISION_IDLE; }
int peer_running(void) { return Vision_GetDefault()->phase == VISION_RUNNING; }
int peer_at_target(void) { return Vision_GetResult(Vision_GetDefault()) == VISION_RESULT_ARRIVED; }
int peer_area(void) {
    const VisionArea *a = Vision_GetArea(Vision_GetDefault(), tick);
    return a && a->cx == 160 && a->cy == 120 && a->radius == 105;
}
int peer_pose(int target, int dx, int dy, int cx, int cy) {
    const VisionPosition *p = Vision_GetPosition(Vision_GetDefault(), tick);
    return p && p->target == target && p->dx == dx && p->dy == dy && p->cx == cx && p->cy == cy;
}
int peer_stable(void) { return Vision_RingMarkStable(Vision_GetDefault(), tick); }
int peer_moved(void) {
    Vision_RingMarkMoving(Vision_GetDefault(), tick);
    return Vision_RingMarkStepDone(Vision_GetDefault(), tick);
}
int peer_ring_success(void) { return Vision_RingSucceeded(Vision_GetDefault()); }
int peer_cancel(void) { return Vision_Cancel(Vision_GetDefault()); }
int peer_reset(void) { return Vision_Reset(Vision_GetDefault()); }
int peer_error(void) { return Vision_GetResult(Vision_GetDefault()) == VISION_RESULT_ERROR; }
const char *peer_why(void) { return Vision_GetDefault()->last_error; }
