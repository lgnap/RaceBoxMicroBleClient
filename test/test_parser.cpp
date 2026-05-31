// test_parser.cpp — native unit tests for RaceBoxParser
#include "unity.h"
#include "ubx.h"
#include "RaceBoxParser.h"
#include <string.h>
#include <math.h>

void setUp()    {}
void tearDown() {}

// ── Helpers ───────────────────────────────────────────────────────────────────

static void writeI32(uint8_t* p, size_t off, int32_t v) {
    memcpy(p + off, &v, 4);
}
static void writeU32(uint8_t* p, size_t off, uint32_t v) {
    memcpy(p + off, &v, 4);
}
static void writeU16(uint8_t* p, size_t off, uint16_t v) {
    memcpy(p + off, &v, 2);
}
static void writeI16(uint8_t* p, size_t off, int16_t v) {
    memcpy(p + off, &v, 2);
}

// Build a UbxPacket with a synthetic 80-byte payload.
static UbxPacket makeLivePacket() {
    UbxPacket pkt{};
    pkt.cls = UBX_CLASS_RACEBOX;
    pkt.id  = UBX_ID_LIVE;
    pkt.len = 80;
    memset(pkt.payload, 0, 80);

    uint8_t* p = pkt.payload;

    // Timestamp
    writeU32(p, 0,  123456789u);        // iTOW
    writeU16(p, 4,  2024u);             // year
    p[6] = 4; p[7] = 15;               // April 15
    p[8] = 10; p[9] = 30; p[10] = 45; // 10:30:45
    p[11] = 0x07;                       // validityFlags (date+time+UTC valid)

    // Fix
    p[20] = 3;   // 3D fix
    p[23] = 12;  // 12 satellites

    // Position: Paris (48.8566° N, 2.3522° E), 100m MSL
    writeI32(p, 24, static_cast<int32_t>(2.3522f * 1e7f));   // longitude
    writeI32(p, 28, static_cast<int32_t>(48.8566f * 1e7f));  // latitude
    writeI32(p, 32, 200000);   // wgsAltitude = 200m (in mm)
    writeI32(p, 36, 100000);   // altitude MSL = 100m (in mm)

    // Motion: 36 km/h = 10000 mm/s, heading 90°
    writeU32(p, 48, 10000u);               // speed: 10000 mm/s = 36 km/h
    writeU32(p, 52, 9000000u);             // heading: 9000000 * 1e-5 = 90.0 deg
    writeU32(p, 56, 500u);                 // speedAccuracy: 500 mm/s = 1.8 km/h

    // Battery (offset 67): bit7=charging, bits[6..0]=level%
    p[67] = 85;  // 85%, not charging (bit7=0)

    // IMU — correct offsets per RaceBox BLE Protocol rev 8
    // GForce: X=front/back, Y=right/left, Z=up/down
    writeI16(p, 68, 100);   // gForceX = 100 mG
    writeI16(p, 70, -200);  // gForceY = -200 mG
    writeI16(p, 72, 980);   // gForceZ ≈ 1G
    // RotRate: X=roll, Y=pitch, Z=yaw
    writeI16(p, 74, 50);    // rotRateX = 0.50 deg/s
    writeI16(p, 76, -30);   // rotRateY = -0.30 deg/s
    writeI16(p, 78, 10);    // rotRateZ = 0.10 deg/s

    return pkt;
}

// ── Tests ─────────────────────────────────────────────────────────────────────

void test_parse_live_packet_succeeds() {
    UbxPacket pkt = makeLivePacket();
    RaceBoxData d{};
    TEST_ASSERT_TRUE(raceBoxParse(pkt, d));
}

void test_timestamp_fields() {
    UbxPacket pkt = makeLivePacket();
    RaceBoxData d{};
    raceBoxParse(pkt, d);
    TEST_ASSERT_EQUAL(2024, d.year);
    TEST_ASSERT_EQUAL(4,    d.month);
    TEST_ASSERT_EQUAL(15,   d.day);
    TEST_ASSERT_EQUAL(10,   d.hour);
    TEST_ASSERT_EQUAL(30,   d.minute);
    TEST_ASSERT_EQUAL(45,   d.second);
    TEST_ASSERT_EQUAL(0x07, d.validityFlags);
    TEST_ASSERT_EQUAL(123456789u, d.iTOW);
}

void test_fix_fields() {
    UbxPacket pkt = makeLivePacket();
    RaceBoxData d{};
    raceBoxParse(pkt, d);
    TEST_ASSERT_EQUAL(3,  d.fixStatus);
    TEST_ASSERT_EQUAL(12, d.numSVs);
}

void test_position_longitude() {
    UbxPacket pkt = makeLivePacket();
    RaceBoxData d{};
    raceBoxParse(pkt, d);
    TEST_ASSERT_FLOAT_WITHIN(0.0001f, 2.3522f, d.longitude);
}

void test_position_latitude() {
    UbxPacket pkt = makeLivePacket();
    RaceBoxData d{};
    raceBoxParse(pkt, d);
    TEST_ASSERT_FLOAT_WITHIN(0.0001f, 48.8566f, d.latitude);
}

void test_altitude_msl() {
    UbxPacket pkt = makeLivePacket();
    RaceBoxData d{};
    raceBoxParse(pkt, d);
    TEST_ASSERT_FLOAT_WITHIN(0.01f, 100.0f, d.altitude);
}

void test_wgs_altitude() {
    UbxPacket pkt = makeLivePacket();
    RaceBoxData d{};
    raceBoxParse(pkt, d);
    TEST_ASSERT_FLOAT_WITHIN(0.01f, 200.0f, d.wgsAltitude);
}

void test_speed_conversion_mm_s_to_km_h() {
    // 10000 mm/s = 36.0 km/h
    UbxPacket pkt = makeLivePacket();
    RaceBoxData d{};
    raceBoxParse(pkt, d);
    TEST_ASSERT_FLOAT_WITHIN(0.01f, 36.0f, d.speed);
}

void test_speed_1_mm_s_equals_0_0036_km_h() {
    UbxPacket pkt = makeLivePacket();
    writeU32(pkt.payload, 48, 1000u);  // 1000 mm/s = 3.6 km/h
    RaceBoxData d{};
    raceBoxParse(pkt, d);
    TEST_ASSERT_FLOAT_WITHIN(0.001f, 3.6f, d.speed);
}

void test_heading_degrees() {
    // 9000000 * 1e-5 = 90.0 degrees
    UbxPacket pkt = makeLivePacket();
    RaceBoxData d{};
    raceBoxParse(pkt, d);
    TEST_ASSERT_FLOAT_WITHIN(0.001f, 90.0f, d.heading);
}

void test_imu_fields() {
    UbxPacket pkt = makeLivePacket();
    RaceBoxData d{};
    raceBoxParse(pkt, d);
    TEST_ASSERT_EQUAL(100,  d.gForceX);
    TEST_ASSERT_EQUAL(-200, d.gForceY);
    TEST_ASSERT_EQUAL(980,  d.gForceZ);
    TEST_ASSERT_EQUAL(50,   d.rotRateX);
    TEST_ASSERT_EQUAL(-30,  d.rotRateY);
    TEST_ASSERT_EQUAL(10,   d.rotRateZ);
}

void test_battery_level_and_charging() {
    UbxPacket pkt = makeLivePacket();
    RaceBoxData d{};
    raceBoxParse(pkt, d);
    TEST_ASSERT_EQUAL(85,    d.batteryLevel);
    TEST_ASSERT_FALSE(d.charging);
}

void test_battery_charging_flag() {
    UbxPacket pkt = makeLivePacket();
    pkt.payload[67] |= 0x80;  // set bit7 = charging
    RaceBoxData d{};
    raceBoxParse(pkt, d);
    TEST_ASSERT_TRUE(d.charging);
}

void test_history_packet_parsed_same_as_live() {
    UbxPacket pkt = makeLivePacket();
    pkt.id = UBX_ID_HISTORY;  // 0xFF/0x21 — same payload format
    RaceBoxData d{};
    TEST_ASSERT_TRUE(raceBoxParse(pkt, d));
    TEST_ASSERT_FLOAT_WITHIN(0.001f, 36.0f, d.speed);
    TEST_ASSERT_EQUAL(2024, d.year);
}

void test_wrong_class_returns_false() {
    UbxPacket pkt = makeLivePacket();
    pkt.cls = 0x01;  // standard UBX class, not RaceBox
    RaceBoxData d{};
    TEST_ASSERT_FALSE(raceBoxParse(pkt, d));
}

void test_wrong_id_returns_false() {
    UbxPacket pkt = makeLivePacket();
    pkt.id = 0x22;  // recording status, not data
    RaceBoxData d{};
    TEST_ASSERT_FALSE(raceBoxParse(pkt, d));
}

void test_payload_too_short_returns_false() {
    UbxPacket pkt = makeLivePacket();
    pkt.len = 40;  // too short
    RaceBoxData d{};
    TEST_ASSERT_FALSE(raceBoxParse(pkt, d));
}

// ── Real packet from official RaceBox BLE Protocol rev 8 (page 7-8) ──────────
//
// Example Mini/Mini S frame (90 bytes = 6 header + 80 payload + 2 checksum).
// Expected values from the PDF table:
//   Year=2022, Month=1, Day=10, Hour=8, Minute=51, Second=8
//   fixStatus=3, numSVs=11, lon=23.2887238°, lat=42.6719035°
//   wgsAlt=625.761m, alt=590.095m, speed≈0.126 km/h, heading=0°
//   battery=89%, not charging
//   gForceX=-3 mG, gForceY=113 mG, gForceZ=974 mG (≈1G ✓)
//   rotRateX=-209, rotRateY=86, rotRateZ=-4
static const uint8_t kPdfFrame[] = {
    0xB5, 0x62, 0xFF, 0x01, 0x50, 0x00,
    0xA0, 0xE7, 0x0C, 0x07,  // [0..3]  iTOW
    0xE6, 0x07,              // [4..5]  year=2022
    0x01,                    // [6]     month=1
    0x0A,                    // [7]     day=10
    0x08,                    // [8]     hour=8
    0x33,                    // [9]     minute=51
    0x08,                    // [10]    second=8
    0x37,                    // [11]    validityFlags
    0x19, 0x00, 0x00, 0x00,  // [12..15] tAcc
    0x2A, 0xAD, 0x4D, 0x0E,  // [16..19] nano
    0x03,                    // [20]    fixStatus=3
    0x01,                    // [21]    flags
    0xEA,                    // [22]    flags2
    0x0B,                    // [23]    numSVs=11
    0xC6, 0x93, 0xE1, 0x0D,  // [24..27] longitude
    0x3B, 0x37, 0x6F, 0x19,  // [28..31] latitude
    0x61, 0x8C, 0x09, 0x00,  // [32..35] wgsAltitude=625761mm
    0x0F, 0x01, 0x09, 0x00,  // [36..39] altitudeMSL=590095mm... wait
    0x9C, 0x03, 0x00, 0x00,  // [40..43] hAcc
    0x2C, 0x07, 0x00, 0x00,  // [44..47] vAcc
    0x23, 0x00, 0x00, 0x00,  // [48..51] speed=35 mm/s
    0x00, 0x00, 0x00, 0x00,  // [52..55] heading=0
    0xD0, 0x00, 0x00, 0x00,  // [56..59] speedAccuracy
    0x88, 0xA9, 0xDD, 0x00,  // [60..63] headingAccuracy (not parsed)
    0x2C, 0x01,              // [64..65] PDOP (not parsed)
    0x00,                    // [66]     Lat/Lon Flags (not parsed)
    0x59,                    // [67]     battery=89%, bit7=0 → not charging
    0xFD, 0xFF,              // [68..69] gForceX=-3 mG
    0x71, 0x00,              // [70..71] gForceY=113 mG
    0xCE, 0x03,              // [72..73] gForceZ=974 mG (≈1G ✓)
    0x2F, 0xFF,              // [74..75] rotRateX=-209
    0x56, 0x00,              // [76..77] rotRateY=86
    0xFC, 0xFF,              // [78..79] rotRateZ=-4
    0x06, 0xDB              // checksum
};

void test_parse_real_packet_from_pdf() {
    UbxPacket pkt{};
    TEST_ASSERT_TRUE(ubxDecode(kPdfFrame, sizeof(kPdfFrame), pkt));

    RaceBoxData d{};
    TEST_ASSERT_TRUE(raceBoxParse(pkt, d));

    TEST_ASSERT_EQUAL_UINT16(2022, d.year);
    TEST_ASSERT_EQUAL_UINT8(1,  d.month);
    TEST_ASSERT_EQUAL_UINT8(10, d.day);
    TEST_ASSERT_EQUAL_UINT8(8,  d.hour);
    TEST_ASSERT_EQUAL_UINT8(51, d.minute);
    TEST_ASSERT_EQUAL_UINT8(8,  d.second);
    TEST_ASSERT_EQUAL_UINT8(3,  d.fixStatus);
    TEST_ASSERT_EQUAL_UINT8(11, d.numSVs);
    TEST_ASSERT_FLOAT_WITHIN(0.0001f, 23.2887f, d.longitude);
    TEST_ASSERT_FLOAT_WITHIN(0.0001f, 42.6719f, d.latitude);
    TEST_ASSERT_FLOAT_WITHIN(0.1f,  625.761f,   d.wgsAltitude);
    TEST_ASSERT_FLOAT_WITHIN(0.01f, 0.126f,     d.speed);
    TEST_ASSERT_FLOAT_WITHIN(0.1f,  0.0f,       d.heading);
    TEST_ASSERT_EQUAL_UINT8(89, d.batteryLevel);
    TEST_ASSERT_FALSE(d.charging);
    TEST_ASSERT_EQUAL_INT16(-3,   d.gForceX);
    TEST_ASSERT_EQUAL_INT16(113,  d.gForceY);
    TEST_ASSERT_EQUAL_INT16(974,  d.gForceZ);   // ≈1G at rest ✓
    TEST_ASSERT_EQUAL_INT16(-209, d.rotRateX);  // roll
    TEST_ASSERT_EQUAL_INT16(86,   d.rotRateY);  // pitch
    TEST_ASSERT_EQUAL_INT16(-4,   d.rotRateZ);  // yaw
}

// ── Edge cases ────────────────────────────────────────────────────────────────

void test_negative_longitude_western_hemisphere() {
    // New York: lon ≈ -74.006° (west of prime meridian → negative)
    UbxPacket pkt = makeLivePacket();
    writeI32(pkt.payload, 24, static_cast<int32_t>(-74.006f * 1e7f));
    RaceBoxData d{};
    TEST_ASSERT_TRUE(raceBoxParse(pkt, d));
    TEST_ASSERT_FLOAT_WITHIN(0.001f, -74.006f, d.longitude);
}

void test_negative_latitude_southern_hemisphere() {
    // Sydney: lat ≈ -33.869° (south → negative)
    UbxPacket pkt = makeLivePacket();
    writeI32(pkt.payload, 28, static_cast<int32_t>(-33.869f * 1e7f));
    RaceBoxData d{};
    TEST_ASSERT_TRUE(raceBoxParse(pkt, d));
    TEST_ASSERT_FLOAT_WITHIN(0.001f, -33.869f, d.latitude);
}

void test_zero_speed() {
    UbxPacket pkt = makeLivePacket();
    writeU32(pkt.payload, 48, 0u);  // 0 mm/s
    RaceBoxData d{};
    raceBoxParse(pkt, d);
    TEST_ASSERT_FLOAT_WITHIN(0.001f, 0.0f, d.speed);
}

void test_fix_status_2d() {
    UbxPacket pkt = makeLivePacket();
    pkt.payload[20] = 2;  // 2D fix
    RaceBoxData d{};
    raceBoxParse(pkt, d);
    TEST_ASSERT_EQUAL(2, d.fixStatus);
}

void test_fix_status_no_fix() {
    UbxPacket pkt = makeLivePacket();
    pkt.payload[20] = 0;  // no fix
    RaceBoxData d{};
    raceBoxParse(pkt, d);
    TEST_ASSERT_EQUAL(0, d.fixStatus);
}

void test_battery_zero_percent() {
    UbxPacket pkt = makeLivePacket();
    pkt.payload[67] = 0x00;  // 0%, not charging
    RaceBoxData d{};
    raceBoxParse(pkt, d);
    TEST_ASSERT_EQUAL(0, d.batteryLevel);
    TEST_ASSERT_FALSE(d.charging);
}

void test_battery_full_and_charging() {
    UbxPacket pkt = makeLivePacket();
    pkt.payload[67] = 0x80 | 100;  // 100% + charging bit
    RaceBoxData d{};
    raceBoxParse(pkt, d);
    TEST_ASSERT_EQUAL(100, d.batteryLevel);
    TEST_ASSERT_TRUE(d.charging);
}

void test_max_positive_rotation_rate() {
    UbxPacket pkt = makeLivePacket();
    writeI16(pkt.payload, 74, 32767);  // INT16_MAX
    RaceBoxData d{};
    raceBoxParse(pkt, d);
    TEST_ASSERT_EQUAL_INT16(32767, d.rotRateX);
}

void test_max_negative_rotation_rate() {
    UbxPacket pkt = makeLivePacket();
    writeI16(pkt.payload, 74, -32768);  // INT16_MIN
    RaceBoxData d{};
    raceBoxParse(pkt, d);
    TEST_ASSERT_EQUAL_INT16(-32768, d.rotRateX);
}

void test_payload_exactly_80_bytes_accepted() {
    UbxPacket pkt = makeLivePacket();
    pkt.len = 80;  // minimum valid size
    RaceBoxData d{};
    TEST_ASSERT_TRUE(raceBoxParse(pkt, d));
}

void test_payload_79_bytes_rejected() {
    UbxPacket pkt = makeLivePacket();
    pkt.len = 79;  // one byte below minimum
    RaceBoxData d{};
    TEST_ASSERT_FALSE(raceBoxParse(pkt, d));
}

// ── Runner ────────────────────────────────────────────────────────────────────
int main() {
    UNITY_BEGIN();
    RUN_TEST(test_parse_live_packet_succeeds);
    RUN_TEST(test_timestamp_fields);
    RUN_TEST(test_fix_fields);
    RUN_TEST(test_position_longitude);
    RUN_TEST(test_position_latitude);
    RUN_TEST(test_altitude_msl);
    RUN_TEST(test_wgs_altitude);
    RUN_TEST(test_speed_conversion_mm_s_to_km_h);
    RUN_TEST(test_speed_1_mm_s_equals_0_0036_km_h);
    RUN_TEST(test_heading_degrees);
    RUN_TEST(test_imu_fields);
    RUN_TEST(test_battery_level_and_charging);
    RUN_TEST(test_battery_charging_flag);
    RUN_TEST(test_history_packet_parsed_same_as_live);
    RUN_TEST(test_wrong_class_returns_false);
    RUN_TEST(test_wrong_id_returns_false);
    RUN_TEST(test_payload_too_short_returns_false);
    RUN_TEST(test_parse_real_packet_from_pdf);
    RUN_TEST(test_negative_longitude_western_hemisphere);
    RUN_TEST(test_negative_latitude_southern_hemisphere);
    RUN_TEST(test_zero_speed);
    RUN_TEST(test_fix_status_2d);
    RUN_TEST(test_fix_status_no_fix);
    RUN_TEST(test_battery_zero_percent);
    RUN_TEST(test_battery_full_and_charging);
    RUN_TEST(test_max_positive_rotation_rate);
    RUN_TEST(test_max_negative_rotation_rate);
    RUN_TEST(test_payload_exactly_80_bytes_accepted);
    RUN_TEST(test_payload_79_bytes_rejected);
    return UNITY_END();
}
