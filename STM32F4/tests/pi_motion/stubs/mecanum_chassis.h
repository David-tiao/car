#ifndef TEST_MECANUM_CHASSIS_H
#define TEST_MECANUM_CHASSIS_H
/* Host ABI substitute only; signatures follow production headers. */
#include <stdint.h>
#include <stddef.h>
typedef struct {float actual_speed,target_speed,output;} Motor_t;
typedef struct {uint8_t valid;} NavImuSnapshot_t;
typedef struct {
    Motor_t *motor_lf,*motor_rf,*motor_lb,*motor_rb;
    float pulse_to_meter,x_pos,y_pos,theta_pos,vx_actual,vy_actual,omega_actual;
    float x_target,y_target,theta_target,max_speed,max_omega,position_tol,theta_tol;
    NavImuSnapshot_t imu_snapshot;
    uint8_t enable,mode,body_frame;
} MecanumChassis_t;
uint32_t __get_PRIMASK(void);void __disable_irq(void);void __set_PRIMASK(uint32_t);
uint32_t HAL_GetTick(void);
void Mecanum_Stop(MecanumChassis_t *);void Motor_Stop(Motor_t *);
void Mecanum_SetSpeedLimit(MecanumChassis_t *,float,float);
void Mecanum_SetTolerance(MecanumChassis_t *,float,float);
void Mecanum_SetPositionMode(MecanumChassis_t *,float,float,float);
typedef struct {uint32_t BaudRate,WordLength,StopBits,Parity,Mode,HwFlowCtl,OverSampling;} UART_InitTypeDef;
typedef struct {void *Instance;UART_InitTypeDef Init;} UART_HandleTypeDef;
typedef struct {uint32_t Pin,Mode,Pull,Speed,Alternate;} GPIO_InitTypeDef;
#define USART2 ((void *)2)
#define GPIOD ((void *)4)
#define GPIO_PIN_5 32u
#define GPIO_PIN_6 64u
#define GPIO_MODE_AF_PP 2u
#define GPIO_PULLUP 1u
#define GPIO_SPEED_FREQ_VERY_HIGH 3u
#define GPIO_AF7_USART2 7u
#define USART2_IRQn 38
#define UART_WORDLENGTH_8B 0u
#define UART_STOPBITS_1 0u
#define UART_PARITY_NONE 0u
#define UART_MODE_TX_RX 3u
#define UART_HWCONTROL_NONE 0u
#define UART_OVERSAMPLING_16 0u
#define HAL_OK 0
#define __HAL_RCC_GPIOD_CLK_ENABLE() ((void)0)
#define __HAL_RCC_USART2_CLK_ENABLE() ((void)0)
void HAL_GPIO_Init(void *,GPIO_InitTypeDef *);
int HAL_UART_Init(UART_HandleTypeDef *);
int HAL_UART_Receive_IT(UART_HandleTypeDef *,uint8_t *,uint16_t);
int HAL_UART_Transmit_IT(UART_HandleTypeDef *,uint8_t *,uint16_t);
void HAL_UART_IRQHandler(UART_HandleTypeDef *);
void HAL_NVIC_SetPriority(int,unsigned,unsigned);
void HAL_NVIC_EnableIRQ(int);
#endif
