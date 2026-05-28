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

    // IMU
    writeI16(p, 60, 100);   // gForceX = 100 mG
    writeI16(p, 62, -200);  // gForceY = -200 mG
    writeI16(p, 64, 980);   // gForceZ ≈ 1G
    writeI16(p, 66, 50);    // rotRateX = 50 * 0.01 = 0.5 deg/s
    writeI16(p, 68, -30);   // rotRateY
    writeI16(p, 70, 10);    // rotRateZ

    // Battery: 85%, not charging
    p[72] = 85;
    p[73] = 0x00;

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
    pkt.payload[73] = 0x01;  // charging bit set
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
    return UNITY_END();
}
