#include "../stm32/vision.h"
#include <assert.h>
#include <stdio.h>
#include <string.h>
static char mvout[2048];
static int stops;
static void mv_tx(const uint8_t *p, size_t n, void *u) {
    (void)u;
    assert(strlen(mvout)+n<sizeof(mvout));
    strncat(mvout,(const char*)p,n);
}
static void stop(void *u) { (void)u; ++stops; }
static void feed(VisionContext *v, const char *body, uint32_t now) {
    char h[180], frame[200]; unsigned chk=0; size_t i;
    (void)snprintf(h,sizeof(h),"8,%zu,%s",strlen(body),body);
    for(i=0;i<strlen(h);++i) chk ^= (unsigned char)h[i];
    (void)snprintf(frame,sizeof(frame),"@%s*%02X\n",h,chk);
    for(i=0;i<strlen(frame);++i) Vision_FeedByte(v,VISION_PORT_OPENMV,(uint8_t)frame[i],now);
}
int main(void) {
    VisionContext v; const VisionTarget *t;
    Vision_Init(&v,mv_tx,NULL,NULL,stop,NULL); /* no HOST/ROS port */
    assert(stops==1);
    assert(Vision_SendBody(&v,VISION_PORT_OPENMV,"MODE,MATERIAL"));
    assert(strstr(mvout,"MODE,MATERIAL")!=NULL);
    feed(&v,"DISK,1,160,120,110",50);
    feed(&v,"TARGET,7,1,165,118,21,0,0,0,0,S,92,0",70);
    t=Vision_GetTarget(&v,75);
    assert(t && t->pixel_x==165 && t->pixel_y==118 && t->metric_valid==0);
    assert(Vision_GetMetricTarget(&v,75)==NULL);
    feed(&v,"TARGET,7,1,165,118,21,12,-5,350,350,S,92,1",90);
    t=Vision_GetMetricTarget(&v,95);
    assert(t && t->x_mm==12 && t->y_mm==-5 && t->distance_mm==350);
    assert(Vision_GetTarget(&v,1000)==NULL);
    feed(&v,"RING,2,140,100,0,0,0,0,73,0",1005);
    assert(Vision_GetRing(&v,2,1006)!=NULL);
    assert(Vision_GetMetricRing(&v,2,1006)==NULL);
    feed(&v,"RING,2,140,100,-10,-20,300,301,88,1",1010);
    assert(Vision_GetMetricRing(&v,2,1011)->z_mm==300);
    feed(&v,"NONE,RING",1020);
    assert(Vision_GetRing(&v,2,1021)==NULL);
    feed(&v,"QR,156+123+516+231",1050);
    assert(v.task_present && strcmp(v.task_code,"156+123+516+231")==0);
    feed(&v,"STOP,EDGE",1060);
    assert(v.estop_latched && stops==2);
    printf("PASS: MCU-originated commands, OpenMV RX, stale data, metric validity, QR, STOP\n");
    return 0;
}
