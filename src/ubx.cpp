#include "ubx.h"
#include <string.h>

// ── ubxChecksum ───────────────────────────────────────────────────────────────
void ubxChecksum(const uint8_t* data, size_t startIdx, size_t count,
                 uint8_t& ckA, uint8_t& ckB) {
    ckA = 0;
    ckB = 0;
    for (size_t i = startIdx; i < startIdx + count; i++) {
        ckA += data[i];
        ckB += ckA;
    }
}

// ── ubxDecode ─────────────────────────────────────────────────────────────────
bool ubxDecode(const uint8_t* buf, size_t len, UbxPacket& out) {
    // Minimum frame: overhead bytes with zero-length payload
    if (len < UBX_FRAME_OVERHEAD) return false;

    // Sync bytes
    if (buf[0] != UBX_SYNC_1 || buf[1] != UBX_SYNC_2) return false;

    // Payload length field (little-endian)
    uint16_t payloadLen = static_cast<uint16_t>(buf[4]) |
                          (static_cast<uint16_t>(buf[5]) << 8);

    // Total frame must match: overhead + payload
    if (len < static_cast<size_t>(UBX_FRAME_OVERHEAD + payloadLen)) return false;
    if (payloadLen > UBX_MAX_PAYLOAD) return false;

    // Fletcher-16 checksum covers bytes [2 .. 5+payloadLen] (class through payload)
    uint8_t ckA, ckB;
    ubxChecksum(buf, 2, 4 + payloadLen, ckA, ckB);

    size_t ckOffset = 6 + payloadLen;
    if (buf[ckOffset] != ckA || buf[ckOffset + 1] != ckB) return false;

    out.cls = buf[2];
    out.id  = buf[3];
    out.len = payloadLen;
    memcpy(out.payload, buf + 6, payloadLen);
    return true;
}

// ── ubxEncode ─────────────────────────────────────────────────────────────────
size_t ubxEncode(const UbxPacket& pkt, uint8_t* out, size_t outLen) {
    size_t frameLen = UBX_FRAME_OVERHEAD + pkt.len;
    if (outLen < frameLen) return 0;
    if (pkt.len > UBX_MAX_PAYLOAD) return 0;

    out[0] = UBX_SYNC_1;
    out[1] = UBX_SYNC_2;
    out[2] = pkt.cls;
    out[3] = pkt.id;
    out[4] = static_cast<uint8_t>(pkt.len & 0xFF);
    out[5] = static_cast<uint8_t>(pkt.len >> 8);
    memcpy(out + 6, pkt.payload, pkt.len);

    uint8_t ckA, ckB;
    ubxChecksum(out, 2, 4 + pkt.len, ckA, ckB);
    out[6 + pkt.len]     = ckA;
    out[6 + pkt.len + 1] = ckB;

    return frameLen;
}
