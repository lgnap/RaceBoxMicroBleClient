// test_fifo.cpp — native unit tests for RaceBoxFifo
//
// Tests the ring buffer and UBX frame extraction in isolation,
// without any BLE or NimBLE dependency.
#include "unity.h"
#include "RaceBoxFifo.h"
#include "ubx.h"
#include <string.h>

void setUp()    {}
void tearDown() {}

// ── Helpers ───────────────────────────────────────────────────────────────────

// Build and encode a minimal valid UBX frame (class=0xFF, id=0x01, empty payload).
static size_t makeFrame(uint8_t* out, uint8_t cls, uint8_t id,
                        const uint8_t* payload, uint16_t payloadLen) {
    UbxPacket pkt{};
    pkt.cls = cls;
    pkt.id  = id;
    pkt.len = payloadLen;
    if (payloadLen > 0 && payload != nullptr)
        memcpy(pkt.payload, payload, payloadLen);
    return ubxEncode(pkt, out, UBX_FRAME_OVERHEAD + UBX_MAX_PAYLOAD);
}

// Build an 80-byte live-data frame (0xFF/0x01) with zeroed payload.
static size_t makeLiveFrame(uint8_t* out) {
    uint8_t payload[80] = {};
    return makeFrame(out, 0xFF, 0x01, payload, 80);
}

// ── Test 1: single frame extracted correctly ──────────────────────────────────
void test_single_frame_extracted() {
    RaceBoxFifo fifo;

    uint8_t rawFrame[UBX_FRAME_OVERHEAD + UBX_MAX_PAYLOAD];
    size_t  rawLen = makeLiveFrame(rawFrame);

    fifo.push(rawFrame, rawLen);
    TEST_ASSERT_EQUAL(rawLen, fifo.available());

    uint8_t out[UBX_FRAME_OVERHEAD + UBX_MAX_PAYLOAD];
    size_t  outLen = 0;
    TEST_ASSERT_TRUE(fifo.extractFrame(out, outLen));
    TEST_ASSERT_EQUAL(rawLen, outLen);
    TEST_ASSERT_EQUAL_MEMORY(rawFrame, out, rawLen);

    // FIFO should be empty now
    TEST_ASSERT_EQUAL(0u, fifo.available());
    TEST_ASSERT_FALSE(fifo.extractFrame(out, outLen));
}

// ── Test 2: three consecutive frames extracted in order ───────────────────────
void test_multi_frame_three_frames_extracted_in_order() {
    RaceBoxFifo fifo;

    // Build 3 distinct frames (different IDs)
    uint8_t f[3][UBX_FRAME_OVERHEAD + UBX_MAX_PAYLOAD];
    size_t  flen[3];
    flen[0] = makeFrame(f[0], 0xFF, 0x01, nullptr, 0);
    flen[1] = makeFrame(f[1], 0xFF, 0x21, nullptr, 0);
    flen[2] = makeFrame(f[2], 0xFF, 0x22, nullptr, 0);

    // Push all three at once
    for (int i = 0; i < 3; i++) fifo.push(f[i], flen[i]);

    uint8_t out[UBX_FRAME_OVERHEAD + UBX_MAX_PAYLOAD];
    size_t  outLen;

    for (int i = 0; i < 3; i++) {
        TEST_ASSERT_TRUE(fifo.extractFrame(out, outLen));
        TEST_ASSERT_EQUAL(flen[i], outLen);
        TEST_ASSERT_EQUAL_MEMORY(f[i], out, flen[i]);
    }

    // All consumed
    TEST_ASSERT_FALSE(fifo.extractFrame(out, outLen));
}

// ── Test 3: partial frame is not extracted ────────────────────────────────────
void test_partial_frame_not_extracted() {
    RaceBoxFifo fifo;

    uint8_t rawFrame[UBX_FRAME_OVERHEAD + UBX_MAX_PAYLOAD];
    size_t  rawLen = makeLiveFrame(rawFrame);

    // Push all but the last 2 bytes (missing checksum)
    fifo.push(rawFrame, rawLen - 2);

    uint8_t out[UBX_FRAME_OVERHEAD + UBX_MAX_PAYLOAD];
    size_t  outLen = 0;
    TEST_ASSERT_FALSE(fifo.extractFrame(out, outLen));
    TEST_ASSERT_EQUAL(0u, outLen);

    // After pushing the remaining 2 bytes the frame becomes complete
    fifo.push(rawFrame + rawLen - 2, 2);
    TEST_ASSERT_TRUE(fifo.extractFrame(out, outLen));
    TEST_ASSERT_EQUAL(rawLen, outLen);
}

// ── Test 4: FIFO overflow drops oldest bytes and increments counter ───────────
void test_overflow_increments_counter() {
    RaceBoxFifo fifo;

    // Fill the FIFO to capacity with 0xAA bytes
    uint8_t fill[RACEBOX_FIFO_SIZE];
    memset(fill, 0xAA, sizeof(fill));
    fifo.push(fill, RACEBOX_FIFO_SIZE);
    TEST_ASSERT_EQUAL(0u, fifo.overflowCount());

    // Push one more byte — should overflow
    uint8_t extra = 0xBB;
    fifo.push(&extra, 1);
    TEST_ASSERT_EQUAL(1u, fifo.overflowCount());

    // Push another block — all bytes overflow (FIFO still full)
    uint8_t more[16];
    memset(more, 0xCC, sizeof(more));
    fifo.push(more, sizeof(more));
    TEST_ASSERT_EQUAL(17u, fifo.overflowCount());
}

// ── Test 5: garbage bytes before sync are discarded; valid frame is recovered ──
void test_sync_recovery_skips_garbage() {
    RaceBoxFifo fifo;

    // Push 32 garbage bytes that don't start with 0xB5
    uint8_t garbage[32];
    memset(garbage, 0x55, sizeof(garbage));
    // Sprinkle one 0xB5 in the middle without a following 0x62 (not a real sync)
    garbage[10] = UBX_SYNC_1;
    garbage[11] = 0x00;  // wrong second sync byte
    fifo.push(garbage, sizeof(garbage));

    // Push a valid UBX frame immediately after
    uint8_t rawFrame[UBX_FRAME_OVERHEAD + UBX_MAX_PAYLOAD];
    size_t  rawLen = makeLiveFrame(rawFrame);
    fifo.push(rawFrame, rawLen);

    uint8_t out[UBX_FRAME_OVERHEAD + UBX_MAX_PAYLOAD];
    size_t  outLen = 0;
    TEST_ASSERT_TRUE(fifo.extractFrame(out, outLen));
    TEST_ASSERT_EQUAL(rawLen, outLen);
    TEST_ASSERT_EQUAL_MEMORY(rawFrame, out, rawLen);

    // Nothing left
    TEST_ASSERT_FALSE(fifo.extractFrame(out, outLen));
}

// ── Runner ────────────────────────────────────────────────────────────────────
int main() {
    UNITY_BEGIN();
    RUN_TEST(test_single_frame_extracted);
    RUN_TEST(test_multi_frame_three_frames_extracted_in_order);
    RUN_TEST(test_partial_frame_not_extracted);
    RUN_TEST(test_overflow_increments_counter);
    RUN_TEST(test_sync_recovery_skips_garbage);
    return UNITY_END();
}
