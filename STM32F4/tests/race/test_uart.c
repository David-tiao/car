#include "usart.h"
#include "vision.h"
#include <assert.h>
#include <stdio.h>
#include <string.h>

UART_HandleTypeDef huart3 = {USART3};
static uint8_t *s_destination;
static int s_receive_error, s_transmit_error;
static uint32_t s_irq;
static uint8_t s_command;
static uint8_t s_tx_data[192];
static uint16_t s_tx_length;
static uint32_t s_now;
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *uart);
void HAL_UART_TxCpltCallback(UART_HandleTypeDef *uart);
void HAL_UART_ErrorCallback(UART_HandleTypeDef *uart);
uint32_t __get_PRIMASK(void) { return s_irq; }
void __disable_irq(void) { s_irq = 1U; }
void __set_PRIMASK(uint32_t mask) { s_irq = mask; }

int HAL_UART_Receive_IT(UART_HandleTypeDef *uart, uint8_t *byte, uint16_t length)
{
    assert(uart == &huart3 && length == 1U);
    s_destination = byte;
    return s_receive_error;
}

int HAL_UART_Transmit_IT(UART_HandleTypeDef *uart, uint8_t *byte, uint16_t length)
{
    assert(uart == &huart3 && length > 0U && length < sizeof(s_tx_data));
    memcpy(s_tx_data, byte, length);
    s_tx_length = length;
    s_command = *byte;
    return s_transmit_error;
}
uint32_t HAL_GetTick(void) { return s_now; }
void Mecanum_Stop(MecanumChassis_t *chassis) { (void)chassis; }
void Mecanum_SetPositionMode(MecanumChassis_t *chassis, float x, float y, float yaw)
{ (void)chassis; (void)x; (void)y; (void)yaw; }
void Mecanum_GetPosition(MecanumChassis_t *chassis, float *x, float *y, float *theta)
{ (void)chassis; if (x) *x = 0.0f; if (y) *y = 0.0f; if (theta) *theta = 0.0f; }
void Mecanum_GetVelocity(MecanumChassis_t *chassis, float *vx, float *vy, float *omega)
{ (void)chassis; if (vx) *vx = 0.0f; if (vy) *vy = 0.0f; if (omega) *omega = 0.0f; }
uint8_t Mecanum_IsTargetReached(MecanumChassis_t *chassis) { (void)chassis; return 1U; }

static void receive(const char *text)
{
    while (*text) {
        *s_destination = (uint8_t)*text++;
        HAL_UART_RxCpltCallback(&huart3);
    }
}

static void receive_body(unsigned seq, const char *body)
{
    char header[160], wire[192];
    unsigned char check = 0U;
    unsigned i;
    int n = snprintf(header, sizeof(header), "%u,%u,%s", seq,
                     (unsigned)strlen(body), body);
    assert(n > 0 && (size_t)n < sizeof(header));
    for (i = 0U; i < (unsigned)n; i++) check ^= (unsigned char)header[i];
    n = snprintf(wire, sizeof(wire), "@%s*%02X\n", header, (unsigned)check);
    assert(n > 0 && (size_t)n < sizeof(wire));
    receive(wire);
}

static void test_tagged_terminal(void)
{
    char code[RACE_TASK_CODE_SIZE];
    assert(RaceVision_Init());
    assert(RaceVision_BeginScanSession(23U));
    HAL_UART_TxCpltCallback(&huart3); /* R -> JOB */
    HAL_UART_TxCpltCallback(&huart3);
    receive_body(100U, "ACK,JOB,1");
    receive_body(101U, "QR,156+123+516+231,23");
    assert(RaceVision_Poll(code));
    assert(RaceVision_AckQr(23U));
    HAL_UART_TxCpltCallback(&huart3);
    receive_body(102U, "DONE,QR");
    assert(!RaceVision_Poll(code));
    assert(!RaceVision_HandoffComplete(23U));
    receive_body(103U, "IDLE");
    assert(!RaceVision_Poll(code));
    assert(RaceVision_HandoffComplete(23U));
    assert(!RaceVision_HandoffComplete(24U));
    RaceVision_EndScan();
    assert(!RaceVision_HandoffComplete(23U));
}

int main(void)
{
    test_tagged_terminal();
    const char *valid = "#SEQ,4,5,2,3,2,1,2,5,4,3,1,2\n";
    char code[RACE_TASK_CODE_SIZE];
    unsigned index;
    assert(RaceVision_Init());
    receive(valid);
    assert(!RaceVision_Poll(code));
    receive(valid);
    s_irq = 1U;
    assert(RaceVision_BeginScan());
    assert(s_irq == 1U && s_tx_length == 1U && s_tx_data[0] == 'R');
    assert(!RaceVision_Poll(code));
    receive("2,5,4,3,1,2\n");
    assert(!RaceVision_Poll(code));
    receive(valid);
    assert(RaceVision_Poll(code));
    assert(strcmp(code, "452+321+254+312") == 0);
    RaceVision_EndScan();
    HAL_UART_TxCpltCallback(&huart3);
    receive(valid);
    assert(!RaceVision_Poll(code));
    assert(RaceVision_BeginScan());
    HAL_UART_TxCpltCallback(&huart3);
    for (index = 0U; index < 260U; index++) receive("x");
    assert(!RaceVision_Healthy());
    assert(!RaceVision_Poll(code));
    assert(!RaceVision_BeginScan());
    assert(RaceVision_Init());
    HAL_UART_ErrorCallback(&huart3);
    assert(!RaceVision_Healthy());
    s_receive_error = HAL_ERROR;
    assert(!RaceVision_Init());
    s_receive_error = HAL_OK;
    assert(RaceVision_Init());
    s_transmit_error = HAL_ERROR;
    assert(!RaceVision_BeginScan());
    receive(valid);
    assert(!RaceVision_Poll(code));
    puts("PASS: UART scan gate, queued/partial stale frames, reset command, IRQ preservation, overflow/error/arming failures");
    return 0;
}
