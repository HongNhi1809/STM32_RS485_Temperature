#ifndef COMMANDS_H
#define COMMANDS_H

#include "protocol.h"

/* ---- Data TAGs ---- */
#define TAG_TEMP_C          0x01
#define TAG_TEMP_RAW_DECI   0x02
#define TAG_PROBE_TYPE      0x03
#define TAG_WARN_STATE      0x04
#define TAG_STATUS_BITS     0x05
#define TAG_CFG_STATE       0x08

/* ---- Diagnostic TAGs ---- */
#define TAG_STATUS_DETAIL   0x06
#define TAG_SAMPLE_INFO     0x07

/* ---- Config TAGs ---- */
#define TAG_MAX_TEMP        0xD0
#define TAG_MIN_TEMP        0xD1
#define TAG_WARN_LEVEL      0xD2
#define TAG_SAMPLE_MS       0xD3
#define TAG_OFFSET_DECI     0xD4
#define TAG_PROBE_TYPE_CFG  0xD5

/* ---- Identity TAGs ---- */
#define TAG_DEVICE_TYPE     0xE0
#define TAG_FW_VERSION      0xE1
#define TAG_PROTOCOL_VER    0xE2
#define TAG_SERIAL          0xE3

/* ---- statusBits ---- */
#define STATUS_PROBE_OPEN   (1 << 0)
#define STATUS_PROBE_SHORT  (1 << 1)
#define STATUS_OUT_RANGE    (1 << 2)
#define STATUS_CONV_TMO     (1 << 3)

/* Toàn bộ "bộ nhớ sống" của board — sau này vòng đo (MAX31865) sẽ ghi vào đây,
 * còn vòng bus (command handler) chỉ đọc/ghi theo đúng lệnh nhận được. */
typedef struct {
    /* Data */
    float    tempC;
    int16_t  tempRawDeci;
    uint8_t  probeType;
    uint8_t  warnState;
    uint8_t  statusBits;
    uint8_t  cfgState;

    /* Diagnostic */
    uint8_t  faultCnt;
    uint8_t  lastFault;
    int16_t  minSeen;
    int16_t  maxSeen;
    uint32_t sampleCount;
    uint16_t ageMs;

    /* Config */
    int16_t  maxTempDeci;
    int16_t  minTempDeci;
    uint8_t  warnLevel;
    uint16_t sampleMs;
    int16_t  offsetDeci;
    uint8_t  probeTypeCfg;

    /* Identity*/
    uint16_t fwVersion;
    uint8_t  protocolVersion;
    uint32_t serial;

    float filteredTempC;
    uint8_t filterInitDone;
    uint8_t minMaxInitDone;
} SensorState;

void state_init_defaults(SensorState *s);

int handle_frame(const Frame *req, Frame *resp, SensorState *state);

#endif 