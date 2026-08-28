#include "uart_bus.h"
#include <string.h>

#define DE_PORT   GPIOA
#define DE_PIN    GPIO_PIN_11

static UART_HandleTypeDef *s_huart;
static uint8_t  s_rxByte;                       
static uint8_t  s_buf[UART_BUS_MAX_FRAME];
static volatile uint16_t s_len = 0;
static volatile uint32_t s_lastByteTick = 0;

#define FRAME_SILENCE_MS   5

#define RESPONSE_DELAY_MS  15

void UartBus_Init(UART_HandleTypeDef *huart) {
    s_huart = huart;
    s_len = 0;
    HAL_GPIO_WritePin(DE_PORT, DE_PIN, GPIO_PIN_RESET); 
    HAL_UART_Receive_IT(s_huart, &s_rxByte, 1);
}

void UartBus_OnByteReceived(UART_HandleTypeDef *huart) {
    if (huart != s_huart) return;

    if (s_len < UART_BUS_MAX_FRAME) {
        s_buf[s_len++] = s_rxByte;
    }
    
    s_lastByteTick = HAL_GetTick();

    HAL_UART_Receive_IT(s_huart, &s_rxByte, 1); 
}

bool UartBus_PollFrame(uint8_t *outBuf, uint16_t *outLen) {
    if (s_len == 0) return false; 

    uint32_t elapsed = HAL_GetTick() - s_lastByteTick;
    if (elapsed < FRAME_SILENCE_MS) return false; 

    __disable_irq();
    uint16_t len = s_len;
    memcpy(outBuf, s_buf, len);
    s_len = 0;
    __enable_irq();

    *outLen = len;
    return true;
}

void UartBus_SendFrame(const uint8_t *buf, uint16_t len) {
    uint32_t sinceLastByte = HAL_GetTick() - s_lastByteTick;
    if (sinceLastByte < RESPONSE_DELAY_MS) {
        HAL_Delay(RESPONSE_DELAY_MS - sinceLastByte);
    }

    HAL_GPIO_WritePin(DE_PORT, DE_PIN, GPIO_PIN_SET); 

    HAL_UART_Transmit(s_huart, buf, len, 100);

    HAL_GPIO_WritePin(DE_PORT, DE_PIN, GPIO_PIN_RESET); 

    HAL_UART_Receive_IT(s_huart, &s_rxByte, 1);
}