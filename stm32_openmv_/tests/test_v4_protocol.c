#include "race_vision_protocol.h"
#include <assert.h>
#include <stdio.h>
#include <string.h>

static int feed(RaceVisionDecoder *decoder, const char *packet, RaceVisionEvent *event)
{
    int got = 0;
    for (; *packet; packet++) got += RaceVisionDecoder_PushEvent(decoder, (unsigned char)*packet, event);
    return got;
}
int main(void)
{
    RaceVisionEvent e;
    RaceVisionDecoder d;
    RaceVisionDecoder_Reset(&d);
    assert(feed(&d, "#SEQ,4,5,2,3,2,1", &e) == 0);
    assert(feed(&d, ",2,5,4,3,1,2\n", &e) == 1);
    assert(e.kind == VISION_SEQ && strcmp(e.task_code,"452+321+254+312") == 0);
    assert(feed(&d, "#DISK,1\n", &e) == 1 && e.kind == VISION_DISK && e.disk_locked);
    assert(feed(&d, "#DISK,0\r\n", &e) == 1 && !e.disk_locked);
    assert(feed(&d, "#SLOT,160,190,110\n", &e) == 1 && e.kind == VISION_SLOT);
    assert(e.x == 160 && e.y == 190 && e.radius == 110);
    assert(feed(&d, "#ARM_OK,1\n", &e) == 1 && e.kind == VISION_ARM_ACK && e.arm_enabled);
    assert(feed(&d, "#OBJ,4,160,190,30,30,706,0\n", &e) == 1);
    assert(e.kind == VISION_OBJ && e.color == 4 && e.x == 160 && e.y == 190);
    assert(feed(&d, "#OBJ,3,160,190,30,30,706,0\n", &e) == 1 && e.color == 3);
    assert(feed(&d, "#SLOT,320,190,110\n", &e) == 0);
    assert(feed(&d, "#SLOT,-1,190,110\n", &e) == 0);
    assert(feed(&d, "#SLOT,100,240,110\n", &e) == 0);
    assert(feed(&d, "#SLOT,160,190,110,3\n", &e) == 0);
    assert(feed(&d, "#OBJ,4,160,190,30,30,706,0XYZ\n", &e) == 0);
    assert(feed(&d, "#OBJ,9,160,190,30,30,706,0\n", &e) == 0);
    assert(feed(&d, "#ARM_OK,9\n", &e) == 0);
    assert(feed(&d, "#TRK,1,2,3,4,5,6,7,8,S,88\n", &e) == 0);
    puts("PASS V5-compatible visual packet protocol / fragmentation / validation");
    return 0;
}
