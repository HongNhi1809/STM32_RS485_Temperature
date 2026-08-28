
#include "max31865.h"
#include <math.h>

/* ---- Địa chỉ thanh ghi MAX31865 ---- */
#define REG_CONFIG        0x00
#define REG_RTD_MSB       0x01
#define REG_RTD_LSB       0x02
#define REG_FAULT_STATUS  0x07
#define REG_WRITE_BIT     0x80  /* OR vào địa chỉ khi ghi */

/* ---- Bit cấu hình thanh ghi 0x00 ---- */
#define CFG_VBIAS         (1 << 7)
#define CFG_AUTO_CONV     (1 << 6)  
#define CFG_1SHOT         (1 << 5)
#define CFG_3WIRE         (1 << 4)
#define CFG_FAULT_CLEAR   (1 << 1)
#define CFG_FILTER_50HZ   (1 << 0)  /* 0 = lọc 60Hz, dùng cho VN nếu lưới nhiễu 50Hz thì bật */

/* ---------- Bit-bang GPIO helpers ---------- */
static inline void cs_low(void)   { HAL_GPIO_WritePin(MAX31865_CS_PORT,  MAX31865_CS_PIN,  GPIO_PIN_RESET); }
static inline void cs_high(void)  { HAL_GPIO_WritePin(MAX31865_CS_PORT,  MAX31865_CS_PIN,  GPIO_PIN_SET);   }
static inline void sclk_low(void) { HAL_GPIO_WritePin(MAX31865_SCLK_PORT,MAX31865_SCLK_PIN,GPIO_PIN_RESET); }
static inline void sclk_high(void){ HAL_GPIO_WritePin(MAX31865_SCLK_PORT,MAX31865_SCLK_PIN,GPIO_PIN_SET);   }
static inline void sdi_write(uint8_t bit) {
    HAL_GPIO_WritePin(MAX31865_SDI_PORT, MAX31865_SDI_PIN, bit ? GPIO_PIN_SET : GPIO_PIN_RESET);
}
static inline uint8_t sdo_read(void) {
    return (HAL_GPIO_ReadPin(MAX31865_SDO_PORT, MAX31865_SDO_PIN) == GPIO_PIN_SET) ? 1 : 0;
}

static inline void spi_dly(void) {
    for (volatile int i = 0; i < 8; i++) { __NOP(); }
}

static uint8_t spi_transfer(uint8_t out) {
    uint8_t in = 0;
    for (int i = 7; i >= 0; i--) {
        sdi_write((out >> i) & 0x01);
        spi_dly();
        sclk_high();              
        spi_dly();
        in = (uint8_t)((in << 1) | sdo_read());
        sclk_low();                
        spi_dly();
    }
    return in;
}

static void reg_write(uint8_t reg, uint8_t value) {
    cs_low();
    spi_transfer(reg | REG_WRITE_BIT);
    spi_transfer(value);
    cs_high();
}

static uint8_t reg_read(uint8_t reg) {
    uint8_t v;
    cs_low();
    spi_transfer(reg & 0x7F);
    v = spi_transfer(0x00);
    cs_high();
    return v;
}

static void regs_read_multi(uint8_t startReg, uint8_t *buf, uint8_t len) {
    cs_low();
    spi_transfer(startReg & 0x7F);
    for (uint8_t i = 0; i < len; i++) {
        buf[i] = spi_transfer(0x00);
    }
    cs_high();
}

static float resistance_to_celsius(float rtd_ohm) {
    const float A = 3.9083e-3f;
    const float B = -5.775e-7f;
    float Z1 = -A;
    float Z2 = A * A - 4.0f * B;
    float Z3 = (4.0f * B) / MAX31865_RNOMINAL;
    float Z4 = 2.0f * B;
    float temp = Z2 + Z3 * rtd_ohm;
    temp = (sqrtf(temp) + Z1) / Z4;
    return temp;
}

void Max31865_Init(void) {
    cs_high();
    sclk_low();

    uint8_t cfg = CFG_VBIAS | CFG_FAULT_CLEAR;
#if MAX31865_WIRES_3
    cfg |= CFG_3WIRE;
#endif
    reg_write(REG_CONFIG, cfg);
    HAL_Delay(10); /* datasheet: chờ ổn định bias trước khi đo lần đầu */
}

Max31865_Result Max31865_ReadOnce(uint32_t timeoutMs) {
    Max31865_Result res = {0};

    uint8_t cfg = CFG_VBIAS | CFG_1SHOT;
#if MAX31865_WIRES_3
    cfg |= CFG_3WIRE;
#endif
    reg_write(REG_CONFIG, cfg);

    uint32_t start = HAL_GetTick();
    while (HAL_GPIO_ReadPin(MAX31865_DRDY_PORT, MAX31865_DRDY_PIN) == GPIO_PIN_SET) {
        if ((HAL_GetTick() - start) > timeoutMs) {
            res.fault = MAX31865_FAULT_CONV_TIMEOUT;
            return res;
        }
    }

    uint8_t raw[2];
    regs_read_multi(REG_RTD_MSB, raw, 2);
    uint16_t rtdReg = ((uint16_t)raw[0] << 8) | raw[1];
    uint8_t faultBit = rtdReg & 0x0001;
    uint16_t adcCode = rtdReg >> 1;

    uint8_t faultReg = reg_read(REG_FAULT_STATUS);
    res.rawFaultReg = faultReg;

    if (faultBit || faultReg != 0x00) {
        reg_write(REG_CONFIG, cfg | CFG_FAULT_CLEAR);
    }

    float resistance = ((float)adcCode / 32768.0f) * MAX31865_RREF;

    if (faultBit || (faultReg & 0x04) /* over/undervoltage */ || resistance > 200.0f) {
        res.fault |= MAX31865_FAULT_PROBE_OPEN;
    } else if (resistance < 20.0f) {
        res.fault |= MAX31865_FAULT_PROBE_SHORT;
    }

    if (res.fault == 0) {
        float t = resistance_to_celsius(resistance);
        if (t < -55.0f || t > 125.0f) {
            res.fault |= MAX31865_FAULT_OUT_RANGE;
        }
        res.temperatureC = t;
    }

    return res;
}