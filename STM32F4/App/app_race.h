#ifndef APP_RACE_H
#define APP_RACE_H
#include "mecanum_chassis.h"
#include "race_fsm.h"
#include "race_task_plan.h"
void AppRace_Init(MecanumChassis_t *chassis);
void AppRace_Poll(void);
uint8_t AppRace_ControlTick(void);
uint8_t AppRace_StartRingAlign(uint8_t target);
const RaceFsm *AppRace_Status(void);
const RaceTaskPlan *AppRace_TaskPlan(void);
/* Current executable flow has ended (also true for fault/abort).
 * In match mode completion requires all plan actions including confirmed home. */
uint8_t AppRace_LogFinished(void);
#endif
