// test_ubx.cpp — native unit tests for ubx.h / ubx.cpp
#include "unity.h"
#include "ubx.h"
#include <string.h>

void setUp()    {}
void tearDown() {}

// ── Helpers ───────────────────────────────────────────────────────────────────

// Build a valid UBX frame for class=0xFF, id=0x23, empty payload.
// Frame: B5 62 FF 23 00 00 22 85
// Checksum over [FF 23 00 00]: ckA=0xFF+0x23+0+0=0x22, ckB=0xFF+0x122+0x122=...
// Let's just use ubxEncode and verify.
static void buildDownloadTrigger(uint8_t* buf, size_t& outLen) {
    UbxPacket pkt{};
    pkt.cls = UBX_CLASS_RACEBOX;
    pkt.id  = UBX_ID_DOWNLOAD;
    pkt.len = 0;
    outLen = ubxEncode(pkt, buf, 64);
}

// ── Tests ─────────────────────────────────────────────────────────────────────

void test_encode_empty_payload() {
    uint8_t buf[16];
    size_t len;
    buildDownloadTrigger(buf, len);

    TEST_ASSERT_EQUAL(8, (int)len);          // UBX_FRAME_OVERHEAD
    TEST_ASSERT_EQUAL_HEX8(0xB5, buf[0]);   // sync 1
    TEST_ASSERT_EQUAL_HEX8(0x62, buf[1]);   // sync 2
    TEST_ASSERT_EQUAL_HEX8(0xFF, buf[2]);   // class
    TEST_ASSERT_EQUAL_HEX8(0x23, buf[3]);   // id
    TEST_ASSERT_EQUAL_HEX8(0x00, buf[4]);   // length LSB
    TEST_ASSERT_EQUAL_HEX8(0x00, buf[5]);   // length MSB
    // Checksum over [FF 23 00 00]:
    // step 1: ckA=FF, ckB=FF
    // step 2: ckA=FF+23=22 (wrap), ckB=FF+22=21 (wrap)
    // step 3: ckA=22+00=22, ckB=21+22=43
    // step 4: ckA=22+00=22, ckB=43+22=65
    TEST_ASSERT_EQUAL_HEX8(0x22, buf[6]);
    TEST_ASSERT_EQUAL_HEX8(0x65, buf[7]);
}

void test_encode_then_decode_roundtrip() {
    UbxPacket orig{};
    orig.cls       = UBX_CLASS_RACEBOX;
    orig.id        = UBX_ID_REC_CONFIG;
    orig.len       = 4;
    orig.payload[0] = 0x01;  // start
    orig.payload[1] = 0x19;  // 25 Hz
    orig.payload[2] = 0x00;
    orig.payload[3] = 0x00;

    uint8_t buf[64];
    size_t frameLen = ubxEncode(orig, buf, sizeof(buf));
    TEST_ASSERT_EQUAL(12, (int)frameLen);  // 8 overhead + 4 payload

    UbxPacket decoded{};
    bool ok = ubxDecode(buf, frameLen, decoded);
    TEST_ASSERT_TRUE(ok);
    TEST_ASSERT_EQUAL_HEX8(orig.cls, decoded.cls);
    TEST_ASSERT_EQUAL_HEX8(orig.id,  decoded.id);
    TEST_ASSERT_EQUAL(orig.len, decoded.len);
    TEST_ASSERT_EQUAL_MEMORY(orig.payload, decoded.payload, orig.len);
}

void test_decode_bad_sync_returns_false() {
    uint8_t buf[8] = {0xAA, 0x62, 0xFF, 0x23, 0x00, 0x00, 0x22, 0x21};
    UbxPacket pkt{};
    TEST_ASSERT_FALSE(ubxDecode(buf, sizeof(buf), pkt));
}

void test_decode_bad_checksum_returns_false() {
    uint8_t buf[16];
    size_t len;
    buildDownloadTrigger(buf, len);
    buf[len - 1] ^= 0xFF;  // corrupt last checksum byte
    UbxPacket pkt{};
    TEST_ASSERT_FALSE(ubxDecode(buf, len, pkt));
}

void test_decode_too_short_returns_false() {
    uint8_t buf[4] = {0xB5, 0x62, 0xFF, 0x23};
    UbxPacket pkt{};
    TEST_ASSERT_FALSE(ubxDecode(buf, sizeof(buf), pkt));
}

void test_decode_correct_class_and_id() {
    uint8_t buf[16];
    size_t len;
    buildDownloadTrigger(buf, len);
    UbxPacket pkt{};
    TEST_ASSERT_TRUE(ubxDecode(buf, len, pkt));
    TEST_ASSERT_EQUAL_HEX8(UBX_CLASS_RACEBOX, pkt.cls);
    TEST_ASSERT_EQUAL_HEX8(UBX_ID_DOWNLOAD,   pkt.id);
    TEST_ASSERT_EQUAL(0, pkt.len);
}

void test_encode_returns_zero_if_buffer_too_small() {
    UbxPacket pkt{};
    pkt.cls = 0xFF;
    pkt.id  = 0x01;
    pkt.len = 0;
    uint8_t buf[4];  // too small for UBX_FRAME_OVERHEAD (8)
    size_t result = ubxEncode(pkt, buf, sizeof(buf));
    TEST_ASSERT_EQUAL(0, (int)result);
}

void test_roundtrip_80_byte_payload() {
    UbxPacket orig{};
    orig.cls = UBX_CLASS_RACEBOX;
    orig.id  = UBX_ID_LIVE;
    orig.len = 80;
    for (uint16_t i = 0; i < 80; i++) orig.payload[i] = (uint8_t)(i & 0xFF);

    uint8_t buf[128];
    size_t frameLen = ubxEncode(orig, buf, sizeof(buf));
    TEST_ASSERT_EQUAL(88, (int)frameLen);  // 8 + 80

    UbxPacket decoded{};
    TEST_ASSERT_TRUE(ubxDecode(buf, frameLen, decoded));
    TEST_ASSERT_EQUAL(80, decoded.len);
    TEST_ASSERT_EQUAL_MEMORY(orig.payload, decoded.payload, 80);
}

void test_checksum_constants() {
    // Verify the known checksum for 0xFF/0x23 empty frame
    // Bytes covered: FF 23 00 00
    uint8_t data[] = {0xB5, 0x62, 0xFF, 0x23, 0x00, 0x00};
    uint8_t ckA, ckB;
    ubxChecksum(data, 2, 4, ckA, ckB);
    TEST_ASSERT_EQUAL_HEX8(0x22, ckA);
    TEST_ASSERT_EQUAL_HEX8(0x65, ckB);
}

// ── Runner ────────────────────────────────────────────────────────────────────
int main() {
    UNITY_BEGIN();
    RUN_TEST(test_encode_empty_payload);
    RUN_TEST(test_encode_then_decode_roundtrip);
    RUN_TEST(test_decode_bad_sync_returns_false);
    RUN_TEST(test_decode_bad_checksum_returns_false);
    RUN_TEST(test_decode_too_short_returns_false);
    RUN_TEST(test_decode_correct_class_and_id);
    RUN_TEST(test_encode_returns_zero_if_buffer_too_small);
    RUN_TEST(test_roundtrip_80_byte_payload);
    RUN_TEST(test_checksum_constants);
    return UNITY_END();
}
