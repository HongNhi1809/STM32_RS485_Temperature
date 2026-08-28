#include "protocol.h"
#include <string.h>

uint8_t crc_xor(const uint8_t *data, size_t len) {
    uint8_t c = 0;
    for (size_t i = 0; i < len; i++) c ^= data[i];
    return c;
}

size_t frame_build(uint8_t *out, size_t outCap, const Frame *f) {
    if (f->payloadLen > MAX_PAYLOAD) return 0;
    size_t total = 7 + f->payloadLen; 
    if (total > outCap) return 0;

    out[0] = FRAME_SOF;
    out[1] = f->addr;
    out[2] = f->cmd;
    out[3] = f->seq;
    out[4] = f->flags;
    out[5] = f->payloadLen;
    if (f->payloadLen > 0) {
        memcpy(&out[6], f->payload, f->payloadLen);
    }
    uint8_t crc = crc_xor(&out[1], 5 + f->payloadLen);
    out[6 + f->payloadLen] = crc;

    return total;
}

FrameParseResult frame_parse(const uint8_t *buf, size_t bufLen, Frame *out) {
    if (bufLen < 7) return FRAME_ERR_TOO_SHORT;
    if (buf[0] != FRAME_SOF) return FRAME_ERR_BAD_SOF;

    uint8_t len = buf[5];
    size_t expectedTotal = 7 + (size_t)len;
    if (expectedTotal != bufLen) return FRAME_ERR_LEN_MISMATCH;

    uint8_t crcCalc = crc_xor(&buf[1], 5 + (size_t)len);
    uint8_t crcRecv = buf[6 + len];
    if (crcCalc != crcRecv) return FRAME_ERR_BAD_CRC;

    out->addr = buf[1];
    out->cmd  = buf[2];
    out->seq  = buf[3];
    out->flags = buf[4];
    out->payloadLen = len;
    if (len > 0) memcpy(out->payload, &buf[6], len);

    return FRAME_OK;
}

int tlv_encode(uint8_t *out, size_t outCap, size_t *offset,
               uint8_t tag, uint8_t len, const uint8_t *value) {
    size_t need = *offset + 2 + len;
    if (need > outCap) return -1;
    out[*offset]     = tag;
    out[*offset + 1] = len;
    if (len > 0) memcpy(&out[*offset + 2], value, len);
    *offset += (2 + len);
    return 0;
}

int tlv_decode_all(const uint8_t *payload, uint8_t payloadLen,
                    TlvEntry *outEntries, int maxEntries) {
    size_t pos = 0;
    int count = 0;

    while (pos < (size_t)payloadLen) {
        if (pos + 2 > (size_t)payloadLen) return -1; 

        uint8_t tag = payload[pos];
        uint8_t len = payload[pos + 1];

        if (pos + 2 + (size_t)len > (size_t)payloadLen) return -1; 

        if (count < maxEntries) {
            outEntries[count].tag = tag;
            outEntries[count].len = len;
            outEntries[count].value = &payload[pos + 2];
            count++;
        }

        pos += (2 + (size_t)len);
    }

    return count;
}