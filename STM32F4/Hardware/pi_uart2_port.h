#ifndef PI_UART2_PORT_H
#define PI_UART2_PORT_H
#include "pi_motion_adapter.h"
/* Own USART2 only in a dedicated mode. Never call ULog_Init in that boot. */
int PiUart2_Init(MecanumChassis_t *chassis, uint8_t exclusive_owner, uint32_t now);
void PiUart2_IRQHandler(void);
void PiUart2_RxCplt(UART_HandleTypeDef *uart);
void PiUart2_TxCplt(UART_HandleTypeDef *uart);
void PiUart2_Error(UART_HandleTypeDef *uart);
#endif
