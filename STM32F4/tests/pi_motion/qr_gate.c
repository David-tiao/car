#include "qr_gate.h"
void q_begin(QGate *q,uint32_t session,uint32_t now){q->session=session;q->started=now;q->state=session?Q_WAIT_ACK:Q_FAULT;}
void q_event(QGate *q,uint8_t event,uint32_t session,uint8_t stationary,uint32_t now){
    if(q->state==Q_FAULT||q->state==Q_READY)return;
    if(event==Q_CANCEL||stationary!=1||now-q->started>=5000u){q->state=Q_FAULT;return;}
    if(session!=q->session||event==Q_TICK)return;
    if(q->state==Q_WAIT_ACK&&event==Q_ACK_SENT){q->state=Q_WAIT_DONE;return;}
    if(q->state==Q_WAIT_DONE&&event==Q_DONE_RX){q->state=Q_WAIT_IDLE;return;}
    if(q->state==Q_WAIT_IDLE&&event==Q_IDLE_RX){q->state=Q_READY;return;}
    /* Duplicates are harmless; an early IDLE cannot complete the gate. */
}
