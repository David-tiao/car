#include "motion_link.h"
#include <string.h>
#include <limits.h>

uint16_t ml_crc(const uint8_t *p, size_t n) {
    uint16_t crc=0xffffu;
    while(n--) {
        crc ^= (uint16_t)(*p++) << 8;
        for(unsigned i=0;i<8;i++) crc=(uint16_t)((crc&0x8000u)?((uint32_t)crc<<1)^0x1021u:(uint32_t)crc<<1);
    }
    return crc;
}
static void put32(uint8_t *p,uint32_t v) { for(unsigned i=0;i<4;i++) p[i]=(uint8_t)(v>>(8*i)); }
static uint32_t get32(const uint8_t *p) { return (uint32_t)p[0]|((uint32_t)p[1]<<8)|((uint32_t)p[2]<<16)|((uint32_t)p[3]<<24); }
static int32_t signed32(uint32_t v) { return v<=INT32_MAX ? (int32_t)v : -1-(int32_t)(UINT32_MAX-v); }
static int kind_ok(uint8_t k) { return (k>=ML_ARM&&k<=ML_STOP)||k==ML_STATUS; }
int ml_encode(const MLFrame *f,uint8_t out[ML_SIZE]) {
    if(!f||!out||!kind_ok(f->kind)||!f->session||!f->seq||f->seq>INT32_MAX) return 0;
    out[0]=0x4d;out[1]=0x32;out[2]=1;out[3]=f->kind;
    put32(out+4,f->session);put32(out+8,f->seq);
    for(unsigned i=0;i<6;i++) put32(out+12+4*i,(uint32_t)f->a[i]);
    uint16_t crc=ml_crc(out,36);out[36]=(uint8_t)crc;out[37]=(uint8_t)(crc>>8);return 1;
}
int ml_decode(const uint8_t in[ML_SIZE],MLFrame *f) {
    if(!in||!f||in[0]!=0x4d||in[1]!=0x32||in[2]!=1||!kind_ok(in[3])||ml_crc(in,36)!=((uint16_t)in[36]|((uint16_t)in[37]<<8))) return 0;
    MLFrame t={0};t.kind=in[3];t.session=get32(in+4);t.seq=get32(in+8);
    if(!t.session||!t.seq||t.seq>INT32_MAX) return 0;
    for(unsigned i=0;i<6;i++)t.a[i]=signed32(get32(in+12+4*i));
    *f=t;return 1;
}
void ml_stream_init(MLStream *s){if(s)memset(s,0,sizeof(*s));}
int ml_stream_push(MLStream *s,uint8_t b,MLFrame *out){
    static const uint8_t magic[3]={0x4d,0x32,1};
    if(!s||!out||s->n>=ML_SIZE)return 0;
    s->bytes[s->n++]=b;
    while(s->n){
        unsigned n=s->n<3?s->n:3;
        if(memcmp(s->bytes,magic,n)==0){
            if(s->n<ML_SIZE)return 0;
            if(ml_decode(s->bytes,out)){s->n=0;return 1;}
        }
        --s->n;memmove(s->bytes,s->bytes+1,s->n);
    }
    return 0;
}
static int32_t wrap(int32_t a) { while(a>=180000)a-=360000;while(a< -180000)a+=360000;return a; }
static int32_t ab(int32_t a) { return a<0?-a:a; }
static int within(int32_t x,int32_t lim) { return x>=-lim&&x<=lim; }
static int usable(const MLFeedback *fb,uint32_t now) {
    return fb&&fb->valid==1&&within(fb->x,10000)&&within(fb->y,10000)&&fb->yaw>=-180000&&fb->yaw<180000&&now-fb->sample_ms<=ML_FEEDBACK_MS;
}
static void fault(MLCore *c,uint8_t reason) { c->state=ML_FAULT;c->reason=reason;c->count=0; }
void ml_init(MLCore *c) { if(c)memset(c,0,sizeof(*c)); }
int ml_grant(MLCore *c,uint32_t session,uint8_t local_enable,uint32_t now) {
    if(!c||local_enable!=1||!session||session<=c->last_session||c->state==ML_MOVING||c->state==ML_IDLE||c->state==ML_DONE) return 0;
    ml_init(c);c->last_session=session;c->session=session;c->last_rx=now;return 1;
}
void ml_tick(MLCore *c,const MLFeedback *fb,uint8_t local_enable,uint32_t now) {
    if(!c||!c->session||c->state==ML_FAULT)return;
    if(local_enable!=1){fault(c,ML_INTERLOCK);return;}
    if(now-c->last_rx>=ML_WATCHDOG_MS){fault(c,ML_TIMEOUT);return;}
    if(c->state==ML_DISARMED)return;
    if(!usable(fb,now)){fault(c,ML_FEEDBACK);return;}
    if(c->state!=ML_MOVING){if(fb->stationary!=1)fault(c,ML_INTERLOCK);return;}
    if(now-c->started>=ML_MOVE_MS){fault(c,ML_MOVE_TIMEOUT);return;}
    if(now-fb->sample_ms>now-c->started||fb->stationary!=1||ab(fb->x-c->gx)>ML_POS_TOL_MM||ab(fb->y-c->gy)>ML_POS_TOL_MM||ab(wrap(fb->yaw-c->gyaw))>ML_YAW_TOL_MDEG){c->count=0;c->have_sample=0;return;}
    if(c->have_sample){
        uint32_t advance=fb->sample_ms-c->last_sample;
        if(!advance)return;
        if(advance>ML_FEEDBACK_MS){c->count=0;c->have_sample=0;}
    }
    if(!c->count)c->still_since=fb->sample_ms;
    c->last_sample=fb->sample_ms;c->have_sample=1;if(c->count<ML_STILL_COUNT)c->count++;
    if(c->count>=ML_STILL_COUNT&&fb->sample_ms-c->still_since>=ML_STILL_MS){c->state=ML_DONE;c->reason=ML_OK;}
}
static void report(const MLCore *c,const MLFrame *f,const MLFeedback *fb,uint32_t now,uint8_t reason,MLFrame *r) {
    memset(r,0,sizeof(*r));r->kind=ML_STATUS;r->session=c->session;r->seq=f->seq;
    r->a[0]=c->state;r->a[1]=(int32_t)c->action_seq;r->a[5]=reason;
    /* INT32_MIN triplet means unavailable, never a fabricated pose. */
    r->a[2]=r->a[3]=r->a[4]=INT32_MIN;
    if(usable(fb,now)){r->a[2]=fb->x;r->a[3]=fb->y;r->a[4]=fb->yaw;}
}
static int same(const MLFrame *a,const MLFrame *b) { return a->kind==b->kind&&a->session==b->session&&a->seq==b->seq&&memcmp(a->a,b->a,sizeof(a->a))==0; }
static int args_zero(const MLFrame *f,unsigned start) {for(unsigned i=start;i<6;i++)if(f->a[i])return 0;return 1;}
static int cardinal(int32_t yaw,int32_t *out) {
    const int32_t choices[]={0,90000,-180000,-90000};
    for(unsigned i=0;i<4;i++)if(ab(wrap(yaw-choices[i]))<=ML_YAW_TOL_MDEG){*out=choices[i];return 1;}
    return 0;
}
int ml_receive(MLCore *c,const MLFrame *f,const MLFeedback *fb,uint8_t local_enable,uint32_t now,MLFrame *r) {
    if(!c||!f||!r||!c->session||f->session!=c->session||!f->seq||f->seq>INT32_MAX||f->kind<ML_ARM||f->kind>ML_STOP)return 0;
    ml_tick(c,fb,local_enable,now);
    uint8_t reason=ML_OK;
    /* STOP is idempotent, dominates replay handling, and latches fault. */
    if(f->kind==ML_STOP&&args_zero(f,0)){fault(c,ML_STOPPED);report(c,f,fb,now,c->reason,r);return 1;}
    if(c->state==ML_FAULT){report(c,f,fb,now,c->reason,r);return 1;}
    if(f->seq<=c->last_seq){
        reason=(f->seq==c->last_seq&&same(f,&c->last_command))?c->last_reason:ML_BADSEQ;
        /* A replay never refreshes watchdog or creates a new intent. */
        report(c,f,fb,now,reason,r);return 1;
    }
    if(f->seq!=c->last_seq+1u){report(c,f,fb,now,ML_BADSEQ,r);return 1;}
    c->last_seq=f->seq;c->last_command=*f;
    if(f->kind==ML_ARM){
        int32_t h;
        if(!args_zero(f,0)||c->state!=ML_DISARMED||!usable(fb,now)||fb->stationary!=1||!cardinal(fb->yaw,&h))reason=ML_REJECT;
        else c->state=ML_IDLE;
    }else if(f->kind==ML_HEARTBEAT){
        if(!args_zero(f,0)||c->state==ML_DISARMED)reason=ML_REJECT;
    }else if(f->kind==ML_TRANSLATE||f->kind==ML_TURN){
        int32_t yaw;
        if(c->state==ML_MOVING)reason=ML_BUSY;
        else if((c->state!=ML_IDLE&&c->state!=ML_DONE)||!usable(fb,now)||fb->stationary!=1||!cardinal(fb->yaw,&yaw))reason=ML_REJECT;
        else {
            int32_t x=fb->x,y=fb->y,h=yaw;
            if(f->kind==ML_TRANSLATE){ /* signed body X/Y in mm; exactly one nonzero axis */
                int32_t dx=f->a[0],dy=f->a[1];
                if(!args_zero(f,2)||!within(dx,ML_MAX_STEP_MM)||!within(dy,ML_MAX_STEP_MM)||((dx==0)==(dy==0)))reason=ML_BADARG;
                else if(yaw==0){x+=dx;y+=dy;}else if(yaw==90000){x-=dy;y+=dx;}else if(yaw==-180000){x-=dx;y-=dy;}else{x+=dy;y-=dx;}
            }else{if(!args_zero(f,1)||(f->a[0]!=90000&&f->a[0]!=-90000))reason=ML_BADARG;else h=wrap(yaw+f->a[0]);}
            if(!within(x,10000)||!within(y,10000))reason=ML_BADARG;
            if(reason==ML_OK){c->gx=x;c->gy=y;c->gyaw=h;c->action_seq=f->seq;c->started=now;c->count=0;c->have_sample=0;c->state=ML_MOVING;}
        }
    }else reason=ML_BADARG;
    c->last_reason=reason;
    if(reason==ML_OK)c->last_rx=now;
    report(c,f,fb,now,reason,r);return 1;
}
int ml_intent(const MLCore *c,int32_t goal[3]) {if(!c||!goal||c->state!=ML_MOVING)return 0;goal[0]=c->gx;goal[1]=c->gy;goal[2]=c->gyaw;return 1;}
