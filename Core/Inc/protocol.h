#ifndef PROTOCOL_H
#define PROTOCOL_H

#include <stdint.h>
#include <stddef.h>

#define FRAME_SOF        0xAA
#define MY_ADDR          0x0C   
#define MAX_PAYLOAD      240
#define MAX_FRAME        247   

/* ---- CMD ---- */
#define CMD_READ_ALL     0x01
#define CMD_READ         0x02
#define CMD_RESET        0x03
#define CMD_CONFIG       0x04
#define CMD_IDENTIFY     0x05
#define CMD_ACK          0x80
#define CMD_NACK         0x81

/* ---- Bit FLAGS ---- */
#define FLAG_ACK         (1 << 0)
#define FLAG_PAR         (1 << 1)
#define FLAG_ERR         (1 << 2)
#define FLAG_BSY         (1 << 3)
#define FLAG_OVF         (1 << 4)

typedef struct {
    uint8_t addr;
    uint8_t cmd;
    uint8_t seq;
    uint8_t flags;
    uint8_t payload[MAX_PAYLOAD];
    uint8_t payloadLen;
} Frame;

typedef struct {
    uint8_t tag;
    uint8_t len;
    const uint8_t *value; 
} TlvEntry;

typedef enum {
    FRAME_OK = 0,
    FRAME_ERR_TOO_SHORT,     /* bước 1: độ dài < 7 byte */
    FRAME_ERR_BAD_SOF,       /* bước 2: SOF != 0xAA */
    FRAME_ERR_LEN_MISMATCH,  /* bước 3: LEN+7 != số byte nhận */
    FRAME_ERR_BAD_CRC,       /* bước 4: CRC không khớp — khung này phải bị BỎ ÂM THẦM, không NACK */
} FrameParseResult;

uint8_t crc_xor(const uint8_t *data, size_t len);

size_t frame_build(uint8_t *out, size_t outCap, const Frame *f);

FrameParseResult frame_parse(const uint8_t *buf, size_t bufLen, Frame *out);

int tlv_encode(uint8_t *out, size_t outCap, size_t *offset, 
                    uint8_t tag, uint8_t len, const uint8_t *value);

int tlv_decode_all(const uint8_t *payload, uint8_t payloadLen,
                    TlvEntry *outEntries, int maxEntries);

#endif 