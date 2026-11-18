#ifndef DEMO_APP_VISION_H
#define DEMO_APP_VISION_H
#include "vision.h"

/* Standalone STM32F4/OpenMV integration facade. Add to your existing MCU app. */
void App_VisionInit(void);
void App_VisionPoll(void);
int App_VisionSetMode(const char *mode);
int App_VisionSetColor(uint8_t color);
int App_VisionSetRing(uint8_t ring);
int App_VisionGroupDone(uint8_t group);
int App_VisionPing(void);
int App_VisionReset(void);
const VisionTarget *App_VisionGetTarget(void);
const VisionTarget *App_VisionGetMetricTarget(void);
const VisionRing *App_VisionGetRing(uint8_t digit);
const VisionRing *App_VisionGetMetricRing(uint8_t digit);
int App_VisionGetTask(char out[20]);
int App_VisionDiskLocked(void);

#endif
