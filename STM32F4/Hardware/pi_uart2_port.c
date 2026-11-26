#include "pi_uart2_port.h"
#include <bsp_ulog.h>
#include <string.h>
static UART_HandleTypeDef s_uart;
static uint8_t s_byte, s_wire[ML_SIZE];
static volatile uint8_t s_started, s_tx_busy;

static int send_frame(const uint8_t *data,uint16_t length)
{
    if(!s_started||s_tx_busy||!data||length!=ML_SIZE)return 0;
    memcpy(s_wire,data,length);s_tx_busy=1u;
    if(HAL_UART_Transmit_IT(&s_uart,s_wire,length)!=HAL_OK){s_tx_busy=0u;return 0;}
    return 1;
}
int PiUart2_Init(MecanumChassis_t *chassis,uint8_t owner,uint32_t now)
{
    if(!PI_MOTION_ENABLE||s_started||owner!=1u)return 0;
    UART_HandleTypeDef *log=ULog_GetHandle();
    /* Even a stopped recorder may still own the UART handle. No sharing. */
    if(ULog_IsRecording()||(log&&log->Instance==USART2))return 0;
    GPIO_InitTypeDef g={0};
    __HAL_RCC_GPIOD_CLK_ENABLE();__HAL_RCC_USART2_CLK_ENABLE();
    g.Pin=GPIO_PIN_5|GPIO_PIN_6;g.Mode=GPIO_MODE_AF_PP;g.Pull=GPIO_PULLUP;
    g.Speed=GPIO_SPEED_FREQ_VERY_HIGH;g.Alternate=GPIO_AF7_USART2;HAL_GPIO_Init(GPIOD,&g);
    s_uart.Instance=USART2;s_uart.Init.BaudRate=115200;
    s_uart.Init.WordLength=UART_WORDLENGTH_8B;s_uart.Init.StopBits=UART_STOPBITS_1;
    s_uart.Init.Parity=UART_PARITY_NONE;s_uart.Init.Mode=UART_MODE_TX_RX;
    s_uart.Init.HwFlowCtl=UART_HWCONTROL_NONE;s_uart.Init.OverSampling=UART_OVERSAMPLING_16;
    if(HAL_UART_Init(&s_uart)!=HAL_OK)return 0;
    if(!PiMotion_Init(chassis,send_frame,now))return 0;
    s_tx_busy=0u;
    if(HAL_UART_Receive_IT(&s_uart,&s_byte,1u)!=HAL_OK){PiMotion_IoFault();return 0;}
    s_started=1u;
    HAL_NVIC_SetPriority(USART2_IRQn,3u,0u);HAL_NVIC_EnableIRQ(USART2_IRQn);
    return 1;
}
void PiUart2_IRQHandler(void){if(s_started)HAL_UART_IRQHandler(&s_uart);}
void PiUart2_RxCplt(UART_HandleTypeDef *uart)
{
    if(!s_started||uart!=&s_uart)return;
    PiMotion_RxByte(s_byte);
    if(HAL_UART_Receive_IT(&s_uart,&s_byte,1u)!=HAL_OK)PiMotion_IoFault();
}
void PiUart2_TxCplt(UART_HandleTypeDef *uart){if(s_started&&uart==&s_uart)s_tx_busy=0u;}
void PiUart2_Error(UART_HandleTypeDef *uart){if(s_started&&uart==&s_uart)PiMotion_IoFault();}
