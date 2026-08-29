#ifndef MAX31865_H
#define MAX31865_H

#include <stdint.h>
#include <stdbool.h>
#include "stm32f0xx_hal.h"  

/* ---- Cấu hình chân ---- */
#define MAX31865_DRDY_PORT  GPIOB
#define MAX31865_DRDY_PIN   GPIO_PIN_0   
#define MAX31865_SDO_PORT   GPIOB
#define MAX31865_SDO_PIN    GPIO_PIN_1  
#define MAX31865_SDI_PORT   GPIOB
#define MAX31865_SDI_PIN    GPIO_PIN_2  
#define MAX31865_SCLK_PORT  GPIOB
#define MAX31865_SCLK_PIN   GPIO_PIN_10 
#define MAX31865_CS_PORT    GPIOB
#define MAX31865_CS_PIN     GPIO_PIN_11  

/* ---- Thông số cảm biến ---- */
#define MAX31865_RREF       430.0f  
#define MAX31865_RNOMINAL   100.0f   
#define MAX31865_WIRES_3    1        

/* ---- Bitmask lỗi trả về  ---- */
#define MAX31865_FAULT_PROBE_OPEN   (1 << 0)  /* RTD hở mạch (bit D2 fault reg) */
#define MAX31865_FAULT_PROBE_SHORT  (1 << 1)  /* RTD chập / low threshold */
#define MAX31865_FAULT_OUT_RANGE    (1 << 2)  /* ngoài -55..+125 sau khi tính */
#define MAX31865_FAULT_CONV_TIMEOUT (1 << 3)  /* DRDY không lên trong thời gian chờ */

typedef struct {
    float   temperatureC;   
    uint8_t fault;          
    uint8_t rawFaultReg;   
} Max31865_Result;

void Max31865_Init(void);

Max31865_Result Max31865_ReadOnce(uint32_t timeoutMs);

#endif 