#pragma once
#include <stdint.h>

/**
 * Parsed RaceBox data point.
 * Populated from live data (0xFF/0x01) and history data (0xFF/0x21) messages.
 * Both message types carry an identical 80-byte payload, so a single struct
 * covers both use-cases.
 */
struct RaceBoxData {
    // ── Timestamp ───────────────────────────────────────────────────────────
    uint32_t iTOW;      // GPS time of week (ms)
    uint16_t year;
    uint8_t  month;
    uint8_t  day;
    uint8_t  hour;
    uint8_t  minute;
    uint8_t  second;
    uint8_t  validityFlags;

    // ── Fix quality ──────────────────────────────────────────────────────────
    uint8_t  fixStatus;    // 0=no fix, 2=2D, 3=3D
    uint8_t  numSVs;       // satellites used

    // ── Position ─────────────────────────────────────────────────────────────
    float    longitude;    // degrees
    float    latitude;     // degrees
    float    wgsAltitude;  // m above WGS84 ellipsoid
    float    altitude;     // m above mean sea level

    // ── Motion ───────────────────────────────────────────────────────────────
    float    speed;         // km/h
    float    heading;       // degrees (0–360)
    float    speedAccuracy; // km/h
    float    headingAccuracy; // degrees

    // ── Precision ────────────────────────────────────────────────────────────
    uint16_t pdop;          // position dilution of precision × 100

    // ── IMU ──────────────────────────────────────────────────────────────────
    int16_t  gForceX;   // mG
    int16_t  gForceY;
    int16_t  gForceZ;
    int16_t  rotRateX;  // deg/s × 100
    int16_t  rotRateY;
    int16_t  rotRateZ;

    // ── Battery ──────────────────────────────────────────────────────────────
    uint8_t  batteryLevel;  // 0–100 %
    bool     charging;
};
