#ifndef CAR_QR_GATE_REVIEW_H
#define CAR_QR_GATE_REVIEW_H
#include <stdint.h>
/* Pure review gate, not integrated into existing VisionContext/AppRace. */
enum { Q_WAIT_ACK, Q_WAIT_DONE, Q_WAIT_IDLE, Q_READY, Q_FAULT };
enum { Q_ACK_SENT=1, Q_DONE_RX, Q_IDLE_RX, Q_TICK, Q_CANCEL };
typedef struct { uint32_t session, started; uint8_t state; } QGate;
void q_begin(QGate *q,uint32_t session,uint32_t now);
void q_event(QGate *q,uint8_t event,uint32_t session,uint8_t stationary,uint32_t now);
#endif
