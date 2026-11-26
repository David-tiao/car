#ifndef PI_MOTION_ADAPTER_H
#define PI_MOTION_ADAPTER_H
#include "motion_link.h"
#include "mecanum_chassis.h"
#ifndef PI_MOTION_ENABLE
#define PI_MOTION_ENABLE 0
#endif
/* Candidate for an EXCLUSIVE scheduler, not AppRace or direct-pick mode.
 * Default build cannot initialize or grant. No GPIO or UART opens here. */
typedef int (*PiMotionTx)(const uint8_t *data, uint16_t length);
int PiMotion_Init(MecanumChassis_t *chassis, PiMotionTx tx, uint32_t now);
/* Local-only, after physical authorization, warmup, interlocks and verified TF.
 * Session must be independently provisioned and fresh across power cycles.
 * Do not expose this function to the serial decoder. */
int PiMotion_LocalGrant(uint32_t session, uint8_t local_enable, uint32_t now);
/* Main loop, input imu_sample_ms is the time of a NEW successful IMU frame. */
void PiMotion_Poll(uint32_t now, uint8_t local_enable, uint8_t imu_valid, uint32_t imu_sample_ms);
/* TIM6 1ms BEFORE motor/chassis loops. Return 0 to inhibit loops on fault. */
uint8_t PiMotion_ControlTick(uint32_t now, uint8_t local_enable);
/* TIM6 AFTER actual 10ms chassis loop; never call from main poll. */
void PiMotion_Capture(uint32_t now);
/* ISR UART entry: bounded ring, no parsing, no transmit or HAL blocking. */
void PiMotion_RxByte(uint8_t b);
void PiMotion_IoFault(void);
uint8_t PiMotion_State(void);
uint8_t PiMotion_FaultReason(void);
#endif
