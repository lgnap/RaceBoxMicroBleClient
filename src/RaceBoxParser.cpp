// RaceBoxParser.cpp — no Arduino/BLE dependencies; compiles cleanly for host tests.
#include "RaceBoxParser.h"
#include <string.h>

static constexpr size_t RACEBOX_PAYLOAD_LEN = 74;  // minimum bytes needed

// Helper: read little-endian primitives from payload
static inline int32_t  readI32(const uint8_t* p, size_t off) {
    int32_t v;
    memcpy(&v, p + off, 4);
    return v;
}
static inline uint32_t readU32(const uint8_t* p, size_t off) {
    uint32_t v;
    memcpy(&v, p + off, 4);
    return v;
}
static inline uint16_t readU16(const uint8_t* p, size_t off) {
    uint16_t v;
    memcpy(&v, p + off, 2);
    return v;
}
static inline int16_t  readI16(const uint8_t* p, size_t off) {
    int16_t v;
    memcpy(&v, p + off, 2);
    return v;
}

bool raceBoxParse(const UbxPacket& pkt, RaceBoxData& out) {
    // Accept live data (0xFF/0x01) and history data (0xFF/0x21) — identical payload
    if (pkt.cls != UBX_CLASS_RACEBOX) return false;
    if (pkt.id != UBX_ID_LIVE && pkt.id != UBX_ID_HISTORY) return false;
    if (pkt.len < RACEBOX_PAYLOAD_LEN) return false;

    const uint8_t* p = pkt.payload;

    // ── Timestamp ────────────────────────────────────────────────────────────
    out.iTOW          = readU32(p, 0);
    out.year          = readU16(p, 4);
    out.month         = p[6];
    out.day           = p[7];
    out.hour          = p[8];
    out.minute        = p[9];
    out.second        = p[10];
    out.validityFlags = p[11];

    // ── Fix ──────────────────────────────────────────────────────────────────
    out.fixStatus = p[20];
    out.numSVs    = p[23];

    // ── Position ─────────────────────────────────────────────────────────────
    out.longitude    = readI32(p, 24) / 1e7f;
    out.latitude     = readI32(p, 28) / 1e7f;
    out.wgsAltitude  = readI32(p, 32) / 1000.0f;
    out.altitude     = readI32(p, 36) / 1000.0f;

    // ── Motion (offsets confirmed via BikeTrace field testing) ────────────────
    out.speed         = readU32(p, 48) * 3.6f / 1000.0f;  // mm/s → km/h
    out.heading       = readU32(p, 52) / 100000.0f;
    out.speedAccuracy = readU32(p, 56) * 3.6f / 1000.0f;  // mm/s → km/h

    // ── IMU (offsets from RaceBox BLE Protocol rev 8) ────────────────────────
    out.gForceX  = readI16(p, 60);  // mG
    out.gForceY  = readI16(p, 62);
    out.gForceZ  = readI16(p, 64);
    out.rotRateX = readI16(p, 66);  // 0.01 deg/s
    out.rotRateY = readI16(p, 68);
    out.rotRateZ = readI16(p, 70);

    // ── Battery ──────────────────────────────────────────────────────────────
    out.batteryLevel = p[72];
    out.charging     = (p[73] & 0x01) != 0;

    return true;
}
