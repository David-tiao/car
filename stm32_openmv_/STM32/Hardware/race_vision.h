#ifndef RACE_VISION_H
#define RACE_VISION_H
#include "race_vision_protocol.h"
#include <stdint.h>
uint8_t RaceVision_Init(void);
uint8_t RaceVision_BeginScan(void);
uint8_t RaceVision_Poll(char code[RACE_TASK_CODE_SIZE]); /* Legacy QR tests */
uint8_t RaceVision_PollEvent(RaceVisionEvent *event);
uint8_t RaceVision_Send(const char *packet);
uint8_t RaceVision_Healthy(void);
void RaceVision_EndScan(void);
#endif
