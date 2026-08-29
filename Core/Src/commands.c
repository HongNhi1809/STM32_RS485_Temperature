#include "commands.h"
#include <string.h>

void state_init_defaults(SensorState *s) {
    memset(s, 0, sizeof(*s));
    s->probeType   = 2; 
    s->cfgState    = 0; 
    s->maxTempDeci = 1000; 
    s->minTempDeci = -500; 
    s->warnLevel   = 2;
    s->sampleMs    = 1000;
    s->offsetDeci  = 0;
    s->probeTypeCfg = 2;
    s->fwVersion   = 0x0100;
    s->protocolVersion = 0x23;
    s->serial      = 0x00012345;
}

static int get_data_tag(const SensorState *s, uint8_t tag, uint8_t *buf, uint8_t *outLen) {
    switch (tag) {
        case TAG_TEMP_C:
            memcpy(buf, &s->tempC, 4);
            *outLen = 4;
            return 1;
        case TAG_TEMP_RAW_DECI:
            buf[0] = (uint8_t)(s->tempRawDeci & 0xFF);
            buf[1] = (uint8_t)((s->tempRawDeci >> 8) & 0xFF);
            *outLen = 2;
            return 1;
        case TAG_PROBE_TYPE:
            buf[0] = s->probeType;
            *outLen = 1;
            return 1;
        case TAG_WARN_STATE:
            buf[0] = s->warnState;
            *outLen = 1;
            return 1;
        case TAG_STATUS_BITS:
            buf[0] = s->statusBits;
            *outLen = 1;
            return 1;
        case TAG_CFG_STATE:
            buf[0] = s->cfgState;
            *outLen = 1;
            return 1;
        case TAG_STATUS_DETAIL: 
            buf[0] = s->faultCnt;
            buf[1] = s->lastFault;
            buf[2] = (uint8_t)(s->minSeen & 0xFF);
            buf[3] = (uint8_t)((s->minSeen >> 8) & 0xFF);
            buf[4] = (uint8_t)(s->maxSeen & 0xFF);
            buf[5] = (uint8_t)((s->maxSeen >> 8) & 0xFF);
            *outLen = 6;
            return 1;
        case TAG_SAMPLE_INFO: 
            buf[0] = (uint8_t)(s->sampleCount & 0xFF);
            buf[1] = (uint8_t)((s->sampleCount >> 8) & 0xFF);
            buf[2] = (uint8_t)((s->sampleCount >> 16) & 0xFF);
            buf[3] = (uint8_t)((s->sampleCount >> 24) & 0xFF);
            buf[4] = (uint8_t)(s->ageMs & 0xFF);
            buf[5] = (uint8_t)((s->ageMs >> 8) & 0xFF);
            *outLen = 6;
            return 1;
        default:
            return 0;
    }
}

static void set_ack_flags(Frame *resp, const SensorState *s) {
    resp->flags = FLAG_ACK;
    if (s->statusBits != 0) resp->flags |= FLAG_ERR;
}

static void base_resp(Frame *resp, const Frame *req, uint8_t cmd) {
    resp->addr = MY_ADDR;
    resp->cmd  = cmd;
    resp->seq  = req->seq; 
    resp->payloadLen = 0;
}

/* ---- CMD_READ_ALL (0x01) ---- */
static void do_read_all(const Frame *req, Frame *resp, SensorState *s) {
    base_resp(resp, req, CMD_ACK);
    size_t off = 0;
    uint8_t buf[6]; uint8_t len;
    static const uint8_t order[] = {TAG_TEMP_C, TAG_TEMP_RAW_DECI, TAG_PROBE_TYPE,
                                     TAG_WARN_STATE, TAG_STATUS_BITS, TAG_CFG_STATE};
    for (size_t i = 0; i < sizeof(order); i++) {
        get_data_tag(s, order[i], buf, &len);
        tlv_encode(resp->payload, MAX_PAYLOAD, &off, order[i], len, buf);
    }
    resp->payloadLen = (uint8_t)off;
    set_ack_flags(resp, s);
}

/* ---- CMD_READ (0x02) ---- */
static void do_read(const Frame *req, Frame *resp, SensorState *s) {
    base_resp(resp, req, CMD_ACK);
    size_t off = 0;
    int served = 0, unknown = 0;
    uint8_t buf[6]; uint8_t len;

    for (uint8_t i = 0; i < req->payloadLen; i++) {
        uint8_t tag = req->payload[i];
        if (get_data_tag(s, tag, buf, &len)) {
            tlv_encode(resp->payload, MAX_PAYLOAD, &off, tag, len, buf);
            served++;
        } else {
            unknown++;
        }
    }

    resp->payloadLen = (uint8_t)off;

    if (served == 0) {
        base_resp(resp, req, CMD_NACK);
        resp->flags = 0x00;
        return;
    }

    set_ack_flags(resp, s);
    if (unknown > 0) resp->flags |= FLAG_PAR;
}

/* ---- CMD_RESET (0x03) ---- */
static void do_reset(const Frame *req, Frame *resp, SensorState *s) {
    s->statusBits = 0;
    s->faultCnt = 0;
    s->lastFault = 0;
    s->minSeen = s->tempRawDeci;
    s->maxSeen = s->tempRawDeci;
    s->sampleCount = 0;

    base_resp(resp, req, CMD_ACK);
    set_ack_flags(resp, s); 
}

/* ---- CMD_CONFIG (0x04) ---- */
static void do_config(const Frame *req, Frame *resp, SensorState *s) {
    TlvEntry entries[8];
    int n = tlv_decode_all(req->payload, req->payloadLen, entries, 8);

    int16_t newMax = s->maxTempDeci, newMin = s->minTempDeci;
    uint8_t newWarnLevel = s->warnLevel, newProbeTypeCfg = s->probeTypeCfg;
    uint16_t newSampleMs = s->sampleMs;
    int16_t newOffset = s->offsetDeci;
    int allKnown = (n >= 0);

    for (int i = 0; i < n && allKnown; i++) {
        const TlvEntry *e = &entries[i];
        switch (e->tag) {
            case TAG_MAX_TEMP:
                if (e->len != 2) { allKnown = 0; break; }
                newMax = (int16_t)(e->value[0] | (e->value[1] << 8));
                break;
            case TAG_MIN_TEMP:
                if (e->len != 2) { allKnown = 0; break; }
                newMin = (int16_t)(e->value[0] | (e->value[1] << 8));
                break;
            case TAG_WARN_LEVEL:
                if (e->len != 1) { allKnown = 0; break; }
                newWarnLevel = e->value[0];
                break;
            case TAG_SAMPLE_MS:
                if (e->len != 2) { allKnown = 0; break; }
                newSampleMs = (uint16_t)(e->value[0] | (e->value[1] << 8));
                break;
            case TAG_OFFSET_DECI:
                if (e->len != 2) { allKnown = 0; break; }
                newOffset = (int16_t)(e->value[0] | (e->value[1] << 8));
                break;
            case TAG_PROBE_TYPE_CFG:
                if (e->len != 1) { allKnown = 0; break; }
                if (e->value[0] != 2) { 
                    allKnown = 0;
                    break;
                }
                newProbeTypeCfg = e->value[0];
                break;
            default:
                allKnown = 0; 
                break;
        }
    }

    int valid = allKnown && (newMax > newMin) && (newSampleMs >= 100 && newSampleMs <= 10000);

    if (!valid) {
        base_resp(resp, req, CMD_NACK);
        resp->flags = 0x00;
        return;
    }

    s->maxTempDeci = newMax;
    s->minTempDeci = newMin;
    s->warnLevel = newWarnLevel;
    s->sampleMs = newSampleMs;
    s->offsetDeci = newOffset;
    s->probeTypeCfg = newProbeTypeCfg;
    s->cfgState = 1; 

    base_resp(resp, req, CMD_ACK);
    resp->payloadLen = 0;
    set_ack_flags(resp, s);
}

/* ---- CMD_IDENTIFY (0x05) ---- */
static void do_identify(const Frame *req, Frame *resp, SensorState *s) {
    base_resp(resp, req, CMD_ACK);
    size_t off = 0;
    uint8_t v1[1], v2[2], v4[4];

    v1[0] = 0x01; 
    tlv_encode(resp->payload, MAX_PAYLOAD, &off, TAG_DEVICE_TYPE, 1, v1);

    v2[0] = (uint8_t)(s->fwVersion & 0xFF);
    v2[1] = (uint8_t)((s->fwVersion >> 8) & 0xFF);
    tlv_encode(resp->payload, MAX_PAYLOAD, &off, TAG_FW_VERSION, 2, v2);

    v1[0] = s->protocolVersion;
    tlv_encode(resp->payload, MAX_PAYLOAD, &off, TAG_PROTOCOL_VER, 1, v1);

    v4[0] = (uint8_t)(s->serial & 0xFF);
    v4[1] = (uint8_t)((s->serial >> 8) & 0xFF);
    v4[2] = (uint8_t)((s->serial >> 16) & 0xFF);
    v4[3] = (uint8_t)((s->serial >> 24) & 0xFF);
    tlv_encode(resp->payload, MAX_PAYLOAD, &off, TAG_SERIAL, 4, v4);

    resp->payloadLen = (uint8_t)off;
    set_ack_flags(resp, s); 
}

int handle_frame(const Frame *req, Frame *resp, SensorState *state) {
    switch (req->cmd) {
        case CMD_READ_ALL: do_read_all(req, resp, state); return 1;
        case CMD_READ:     do_read(req, resp, state);     return 1;
        case CMD_RESET:    do_reset(req, resp, state);    return 1;
        case CMD_CONFIG:   do_config(req, resp, state);   return 1;
        case CMD_IDENTIFY: do_identify(req, resp, state); return 1;
        default:
            base_resp(resp, req, CMD_NACK);
            resp->flags = 0x00;
            return 1;
    }
}