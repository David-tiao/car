#ifndef RACE_VISION_PROTOCOL_H
#define RACE_VISION_PROTOCOL_H
#include "race_fsm.h"
#define RACE_VISION_LINE_MAX 96U

typedef enum {
    VISION_NONE = 0, VISION_SEQ, VISION_DISK, VISION_SLOT,
    VISION_OBJ, VISION_ARM_ACK, VISION_TAKE_ACK
} RaceVisionEventKind;

typedef struct {
    RaceVisionEventKind kind;
    char task_code[RACE_TASK_CODE_SIZE];
    uint8_t disk_locked;
    uint8_t color;
    int16_t x, y, radius;
    uint8_t arm_enabled;
} RaceVisionEvent;

typedef struct {
    char line[RACE_VISION_LINE_MAX];
    uint8_t length, collecting, dropping;
} RaceVisionDecoder;

void RaceVisionDecoder_Reset(RaceVisionDecoder *decoder);
uint8_t RaceVisionDecoder_Push(RaceVisionDecoder *decoder, uint8_t byte,
                              char code[RACE_TASK_CODE_SIZE]);
uint8_t RaceVision_ParseSeq(const char *line, char code[RACE_TASK_CODE_SIZE]);
uint8_t RaceVision_ParseEvent(const char *line, RaceVisionEvent *event);
uint8_t RaceVisionDecoder_PushEvent(RaceVisionDecoder *decoder, uint8_t byte,
                                    RaceVisionEvent *event);
#endif
