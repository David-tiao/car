#include "pi_uart2_port.h"
#include <assert.h>
#include <math.h>
#include <stdio.h>
#include <string.h>
static Motor_t motors[4];static MecanumChassis_t ch;
static UART_HandleTypeDef log_uart,*uart;static uint8_t *rx_dest;
static uint8_t tx_wire[ML_SIZE];static unsigned tx_count,position_calls,stop_calls,uart_init_calls;
static uint32_t now,irq;static int tx_error,rx_error;static uint8_t local_enable=1;
uint32_t __get_PRIMASK(void){return irq;}void __disable_irq(void){irq=1;}
void __set_PRIMASK(uint32_t x){irq=x;}uint32_t HAL_GetTick(void){return now;}
void Motor_Stop(Motor_t *m){m->target_speed=0;m->output=0;stop_calls++;}
void Mecanum_Stop(MecanumChassis_t *c){c->mode=0;c->body_frame=0;}
void Mecanum_SetSpeedLimit(MecanumChassis_t *c,float v,float w){c->max_speed=v;c->max_omega=w;}
void Mecanum_SetTolerance(MecanumChassis_t *c,float v,float w){c->position_tol=v;c->theta_tol=w;}
void Mecanum_SetPositionMode(MecanumChassis_t *c,float x,float y,float h){assert(irq==1u&&c->body_frame==0u);c->mode=1;c->x_target=x;c->y_target=y;c->theta_target=h;position_calls++;}
UART_HandleTypeDef *ULog_GetHandle(void){return &log_uart;}
uint8_t ULog_IsRecording(void){return 0;}
void HAL_GPIO_Init(void *p,GPIO_InitTypeDef *g){assert(p==GPIOD&&g->Pin==96u&&g->Alternate==7u);}
int HAL_UART_Init(UART_HandleTypeDef *u){assert(u->Instance==USART2&&u->Init.BaudRate==115200);uart=u;uart_init_calls++;return 0;}
int HAL_UART_Receive_IT(UART_HandleTypeDef *u,uint8_t *p,uint16_t n){assert(u==uart&&n==1);rx_dest=p;return rx_error;}
int HAL_UART_Transmit_IT(UART_HandleTypeDef *u,uint8_t *p,uint16_t n){assert(u==uart&&n==ML_SIZE);memcpy(tx_wire,p,n);tx_count++;return tx_error;}
void HAL_UART_IRQHandler(UART_HandleTypeDef *u){assert(u==uart);}
void HAL_NVIC_SetPriority(int n,unsigned a,unsigned b){assert(n==USART2_IRQn&&a==3&&b==0);}
void HAL_NVIC_EnableIRQ(int n){assert(n==USART2_IRQn);}
static void prepare(void){memset(&ch,0,sizeof(ch));ch.motor_lf=&motors[0];ch.motor_rf=&motors[1];ch.motor_lb=&motors[2];ch.motor_rb=&motors[3];ch.enable=1;ch.imu_snapshot.valid=1;ch.pulse_to_meter=1.0f/1750.0f;}
static void poll(uint32_t t,int capture){now=t;if(capture)PiMotion_Capture(now);PiMotion_Poll(now,local_enable,1,now);}
static void init(void){prepare();assert(PiUart2_Init(&ch,1,now));poll(0,0);poll(1,1);assert(irq==0);}
static void feed(const uint8_t *wire,unsigned n){for(unsigned i=0;i<n;i++){*rx_dest=wire[i];PiUart2_RxCplt(uart);}}
static MLFrame command(unsigned kind,unsigned session,unsigned seq,int a,int b,uint32_t t){
    MLFrame f={0},r={0};uint8_t wire[ML_SIZE];f.kind=(uint8_t)kind;f.session=session;f.seq=seq;f.a[0]=a;f.a[1]=b;assert(ml_encode(&f,wire));
    feed(wire,ML_SIZE);unsigned old=tx_count;poll(t,1);
    if(tx_count!=old){assert(ml_decode(tx_wire,&r));PiUart2_TxCplt(uart);}return r;
}
static void grant(void){assert(!PiMotion_LocalGrant(23,0,now));assert(PiMotion_LocalGrant(23,1,now));assert(command(ML_ARM,23,1,0,0,now+1).a[0]==ML_IDLE);}
static void moving(void){grant();MLFrame r=command(ML_TRANSLATE,23,2,100,0,now+1);assert(r.a[0]==ML_MOVING&&r.a[1]==2);assert(position_calls==1&&ch.mode==1&&fabsf(ch.x_target-.1f)<1e-6f);}
static void faulted(unsigned reason){assert(PiMotion_State()==ML_FAULT&&PiMotion_FaultReason()==reason);assert(ch.mode==0&&stop_calls>=8);assert(!PiMotion_ControlTick(now,1));assert(!PiMotion_LocalGrant(24,1,now));}
int main(int argc,char **argv){assert(argc==2);const char *s=argv[1];
    if(!strcmp(s,"default")){prepare();assert(!PiUart2_Init(&ch,1,0));assert(uart_init_calls==0&&stop_calls==0);return 0;}
    if(!strcmp(s,"owner")){prepare();assert(!PiUart2_Init(&ch,0,0));log_uart.Instance=USART2;assert(!PiUart2_Init(&ch,1,0));assert(uart_init_calls==0);return 0;}
    init();
    if(!strcmp(s,"grant")){assert(command(ML_ARM,23,1,0,0,2).kind==0);assert(position_calls==0);grant();assert(PiMotion_State()==ML_IDLE);return 0;}
    if(!strcmp(s,"fresh")){assert(!PiMotion_LocalGrant(23,1,200));return 0;}
    if(!strcmp(s,"turn")){grant();assert(command(ML_TURN,23,2,90000,0,3).a[0]==ML_MOVING);assert(position_calls==1&&fabsf(ch.theta_target-1.5707963f)<1e-5f&&ch.x_target==0&&ch.y_target==0);return 0;}
    moving();
    if(!strcmp(s,"done")){
        assert(command(ML_TRANSLATE,23,2,100,0,4).a[0]==ML_MOVING&&position_calls==1);
        ch.x_pos=.1f;motors[0].actual_speed=100;for(uint32_t t=10;t<=180;t+=10)poll(t,1);assert(PiMotion_State()==ML_MOVING);
        motors[0].actual_speed=0;for(uint32_t t=190;t<=330;t+=10)poll(t,1);assert(PiMotion_State()==ML_MOVING);poll(340,1);assert(PiMotion_State()==ML_DONE&&ch.mode==0);
        MLFrame r=command(ML_HEARTBEAT,23,3,0,0,350);assert(r.a[0]==ML_DONE&&r.a[1]==2&&r.a[2]==100);
        assert(command(ML_TRANSLATE,23,4,0,50,360).a[0]==ML_MOVING&&position_calls==2&&fabsf(ch.y_target-.05f)<1e-6f);return 0;
    }
    if(!strcmp(s,"stall")){now+=100;assert(!PiMotion_ControlTick(now,1));faulted(ML_TIMEOUT);return 0;}
    if(!strcmp(s,"watchdog")){for(uint32_t t=10;t<500;t+=10)poll(t,1);now=503;assert(!PiMotion_ControlTick(now,1));faulted(ML_TIMEOUT);return 0;}
    if(!strcmp(s,"local")){assert(!PiMotion_ControlTick(now,0));faulted(ML_INTERLOCK);return 0;}
    if(!strcmp(s,"imu")){now=10;PiMotion_Poll(now,1,0,now);faulted(ML_FEEDBACK);return 0;}
    if(!strcmp(s,"sample")){for(uint32_t t=10;t<=110;t+=10)poll(t,0);faulted(ML_FEEDBACK);return 0;}
    if(!strcmp(s,"overflow")){uint8_t data[260]={0};feed(data,sizeof(data));assert(!PiMotion_ControlTick(now,1));faulted(ML_INTERLOCK);return 0;}
    if(!strcmp(s,"rxerror")){rx_error=1;uint8_t b=0;feed(&b,1);assert(!PiMotion_ControlTick(now,1));faulted(ML_INTERLOCK);return 0;}
    if(!strcmp(s,"txerror")){tx_error=1;command(ML_HEARTBEAT,23,3,0,0,4);faulted(ML_INTERLOCK);return 0;}
    if(!strcmp(s,"stop")){assert(command(ML_STOP,23,1,0,0,4).a[0]==ML_FAULT);faulted(ML_STOPPED);return 0;}
    if(!strcmp(s,"badframe")){MLFrame f={0};uint8_t wire[ML_SIZE];f.kind=ML_TRANSLATE;f.session=23;f.seq=3;f.a[0]=100;assert(ml_encode(&f,wire));wire[16]^=1;feed(wire,ML_SIZE);unsigned old=tx_count;poll(4,1);assert(tx_count==old&&position_calls==1);return 0;}
    assert(!"unknown case");return 1;
}
