#ifndef UART_BUS_H
#define UART_BUS_H

#include <stdint.h>
#include <stdbool.h>
#include "stm32f0xx_hal.h"

#define UART_BUS_MAX_FRAME   247  

void UartBus_Init(UART_HandleTypeDef *huart);

bool UartBus_PollFrame(uint8_t *outBuf, uint16_t *outLen);

void UartBus_SendFrame(const uint8_t *buf, uint16_t len);

void UartBus_OnByteReceived(UART_HandleTypeDef *huart);

#endif 