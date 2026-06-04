// test_recorder.cpp — native unit tests for RaceBoxRecorder._onPacket() logic.
// Tests the state machine that handles RaceBox recording status, ACK/NACK,
// state-change events, and ACK timeout — all without BLE hardware.
#include "unity.h"
#include "ubx.h"
#include "RaceBoxBle.h"
#include "RaceBoxRecorder.h"

// Spy state defined in RaceBoxBle_stub.cpp
extern UbxPacket g_stubLastSent;
extern int       g_stubSendCount;
extern bool      g_stubSendResult;
extern void      stubBleReset();
extern uint32_t  _fakeMillis;

void setUp()    { stubBleReset(); _fakeMillis = 0; }
void tearDown() {}

// ── Helpers ───────────────────────────────────────────────────────────────────

// Build a status response packet (0xFF/0x22): state, dataRate, recordCount (LE)
static UbxPacket makeStatusResponse(uint8_t state, uint8_t rate, uint32_t count) {
    UbxPacket pkt{};
    pkt.cls = UBX_CLASS_RACEBOX;
    pkt.id  = UBX_ID_REC_STATUS;
    pkt.len = 6;
    pkt.payload[0] = state;
    pkt.payload[1] = rate;
    pkt.payload[2] = (uint8_t)(count & 0xFF);
    pkt.payload[3] = (uint8_t)((count >> 8)  & 0xFF);
    pkt.payload[4] = (uint8_t)((count >> 16) & 0xFF);
    pkt.payload[5] = (uint8_t)((count >> 24) & 0xFF);
    return pkt;
}

static UbxPacket makeAck(uint8_t ackedId = 0)  {
    UbxPacket p{}; p.cls=UBX_CLASS_RACEBOX; p.id=UBX_ID_ACK;
    p.len=2; p.payload[0]=UBX_CLASS_RACEBOX; p.payload[1]=ackedId;
    return p;
}
static UbxPacket makeUnlockAck() { return makeAck(UBX_ID_MEM_UNLOCK); }
static UbxPacket makeNack() { UbxPacket p{}; p.cls=UBX_CLASS_RACEBOX; p.id=UBX_ID_NACK; p.len=0; return p; }

static UbxPacket makeStateChange(uint8_t event) {
    UbxPacket p{};
    p.cls = UBX_CLASS_RACEBOX;
    p.id  = UBX_ID_STATE_CHANGE;
    p.len = 2;
    p.payload[0] = event;
    return p;
}

// ── Initial state ─────────────────────────────────────────────────────────────

void test_initial_state_is_unknown() {
    RaceBoxBle ble;
    RaceBoxRecorder rec(ble);
    TEST_ASSERT_EQUAL((int)RecordingState::UNKNOWN, (int)rec.state());
}

void test_initial_record_count_is_zero() {
    RaceBoxBle ble;
    RaceBoxRecorder rec(ble);
    TEST_ASSERT_EQUAL(0u, rec.recordCount());
}

// ── Status response parsing ───────────────────────────────────────────────────

void test_status_response_updates_state_idle() {
    RaceBoxBle ble;
    RaceBoxRecorder rec(ble);
    rec._onPacket(makeStatusResponse(0, 25, 0));  // IDLE, 25 Hz, 0 records
    TEST_ASSERT_EQUAL((int)RecordingState::IDLE, (int)rec.state());
}

void test_status_response_updates_state_recording() {
    RaceBoxBle ble;
    RaceBoxRecorder rec(ble);
    rec._onPacket(makeStatusResponse(1, 25, 42));
    TEST_ASSERT_EQUAL((int)RecordingState::RECORDING, (int)rec.state());
}

void test_status_response_updates_state_paused() {
    RaceBoxBle ble;
    RaceBoxRecorder rec(ble);
    rec._onPacket(makeStatusResponse(2, 10, 0));
    TEST_ASSERT_EQUAL((int)RecordingState::PAUSED, (int)rec.state());
}

void test_status_response_updates_record_count() {
    RaceBoxBle ble;
    RaceBoxRecorder rec(ble);
    rec._onPacket(makeStatusResponse(1, 25, 123456u));
    TEST_ASSERT_EQUAL(123456u, rec.recordCount());
}

void test_status_response_updates_data_rate() {
    RaceBoxBle ble;
    RaceBoxRecorder rec(ble);
    rec._onPacket(makeStatusResponse(1, 10, 0));
    TEST_ASSERT_EQUAL((int)DataRate::HZ_10, (int)rec.dataRate());
}

void test_status_response_too_short_ignored() {
    RaceBoxBle ble;
    RaceBoxRecorder rec(ble);
    // Payload < 6 bytes — must be ignored
    UbxPacket p{};
    p.cls = UBX_CLASS_RACEBOX;
    p.id  = UBX_ID_REC_STATUS;
    p.len = 3;
    rec._onPacket(p);
    // State must remain UNKNOWN
    TEST_ASSERT_EQUAL((int)RecordingState::UNKNOWN, (int)rec.state());
}

void test_status_large_record_count_le_decode() {
    RaceBoxBle ble;
    RaceBoxRecorder rec(ble);
    // 0x01020304 = 16909060 in LE: payload = [04 03 02 01]
    rec._onPacket(makeStatusResponse(1, 25, 0x01020304u));
    TEST_ASSERT_EQUAL(0x01020304u, rec.recordCount());
}

// ── ACK / NACK ────────────────────────────────────────────────────────────────

void test_ack_sets_last_ack_true() {
    RaceBoxBle ble;
    RaceBoxRecorder rec(ble);
    rec._onPacket(makeAck());
    TEST_ASSERT_TRUE(rec.lastAck());
}

void test_nack_sets_last_ack_false() {
    RaceBoxBle ble;
    RaceBoxRecorder rec(ble);
    rec._onPacket(makeAck());   // first set to true
    rec._onPacket(makeNack());  // then override to false
    TEST_ASSERT_FALSE(rec.lastAck());
}

// ── State change callback ─────────────────────────────────────────────────────

void test_state_change_fires_callback() {
    RaceBoxBle ble;
    RaceBoxRecorder rec(ble);
    StateChangeEvent captured = (StateChangeEvent)0xFF;
    rec.setStateChangeCallback([&](StateChangeEvent e) { captured = e; });
    rec._onPacket(makeStateChange((uint8_t)StateChangeEvent::RECORDING_START));
    TEST_ASSERT_EQUAL((int)StateChangeEvent::RECORDING_START, (int)captured);
}

void test_state_change_stop_fires_callback() {
    RaceBoxBle ble;
    RaceBoxRecorder rec(ble);
    StateChangeEvent captured = (StateChangeEvent)0xFF;
    rec.setStateChangeCallback([&](StateChangeEvent e) { captured = e; });
    rec._onPacket(makeStateChange((uint8_t)StateChangeEvent::RECORDING_STOP));
    TEST_ASSERT_EQUAL((int)StateChangeEvent::RECORDING_STOP, (int)captured);
}

void test_state_change_without_callback_does_not_crash() {
    RaceBoxBle ble;
    RaceBoxRecorder rec(ble);
    // No callback set — must not crash
    rec._onPacket(makeStateChange((uint8_t)StateChangeEvent::RECORDING_START));
    TEST_PASS();
}

void test_state_change_too_short_ignored() {
    RaceBoxBle ble;
    RaceBoxRecorder rec(ble);
    bool fired = false;
    rec.setStateChangeCallback([&](StateChangeEvent) { fired = true; });
    UbxPacket p{};
    p.cls = UBX_CLASS_RACEBOX;
    p.id  = UBX_ID_STATE_CHANGE;
    p.len = 0;  // too short — no event byte
    rec._onPacket(p);
    TEST_ASSERT_FALSE(fired);
}

// ── Wrong class ignored ───────────────────────────────────────────────────────

void test_wrong_class_packet_ignored() {
    RaceBoxBle ble;
    RaceBoxRecorder rec(ble);
    UbxPacket p{};
    p.cls = 0x01;  // standard UBX, not RaceBox
    p.id  = UBX_ID_REC_STATUS;
    p.len = 6;
    rec._onPacket(p);
    TEST_ASSERT_EQUAL((int)RecordingState::UNKNOWN, (int)rec.state());
}

// ── ACK timeout ───────────────────────────────────────────────────────────────

void test_ack_timeout_clears_pending_after_5s() {
    RaceBoxBle ble;
    RaceBoxRecorder rec(ble);
    rec.begin();
    rec.queryStatus();  // sets _cmdSentMs = millis() = 0

    // Advance time just below timeout — no effect
    _fakeMillis = 4999;
    rec.update();
    TEST_ASSERT_TRUE(g_stubSendCount > 0);  // command was sent

    // Advance past 5 s timeout
    _fakeMillis = 5001;
    rec.update();
    // After timeout, lastAck should be false
    TEST_ASSERT_FALSE(rec.lastAck());
}

// ── sendCommand spy ───────────────────────────────────────────────────────────

void test_query_status_sends_correct_packet() {
    RaceBoxBle ble;
    RaceBoxRecorder rec(ble);
    rec.begin();
    rec.queryStatus();
    TEST_ASSERT_EQUAL_HEX8(UBX_CLASS_RACEBOX, g_stubLastSent.cls);
    TEST_ASSERT_EQUAL_HEX8(UBX_ID_REC_STATUS, g_stubLastSent.id);
    TEST_ASSERT_EQUAL(0, g_stubLastSent.len);
    TEST_ASSERT_EQUAL(1, g_stubSendCount);
}

void test_start_recording_sends_unlock_then_config() {
    // startRecording() now sends unlock (0x30) first, config (0x25) after ACK
    RaceBoxBle ble;
    RaceBoxRecorder rec(ble);
    rec.begin();
    rec.startRecording(DataRate::HZ_25);
    // Step 1: unlock sent
    TEST_ASSERT_EQUAL_HEX8(UBX_CLASS_RACEBOX, g_stubLastSent.cls);
    TEST_ASSERT_EQUAL_HEX8(UBX_ID_MEM_UNLOCK, g_stubLastSent.id);
    TEST_ASSERT_EQUAL(4, g_stubLastSent.len);
    // Step 2: simulate unlock ACK → config sent
    rec._onPacket(makeUnlockAck());
    TEST_ASSERT_EQUAL_HEX8(UBX_ID_REC_CONFIG, g_stubLastSent.id);
    TEST_ASSERT_EQUAL(8, g_stubLastSent.len);
    TEST_ASSERT_EQUAL_HEX8(0x01, g_stubLastSent.payload[0]);  // command = start
    TEST_ASSERT_EQUAL_HEX8(25,   g_stubLastSent.payload[1]);  // rate = 25 Hz
}

void test_stop_recording_sends_unlock_then_stop() {
    RaceBoxBle ble;
    RaceBoxRecorder rec(ble);
    rec.begin();
    rec.stopRecording();
    // Unlock first
    TEST_ASSERT_EQUAL_HEX8(UBX_ID_MEM_UNLOCK, g_stubLastSent.id);
    // Then config on ACK
    rec._onPacket(makeUnlockAck());
    TEST_ASSERT_EQUAL_HEX8(UBX_ID_REC_CONFIG, g_stubLastSent.id);
    TEST_ASSERT_EQUAL_HEX8(0x00, g_stubLastSent.payload[0]);  // command = stop
}

void test_start_with_filters_encodes_correctly() {
    RaceBoxBle ble;
    RaceBoxRecorder rec(ble);
    rec.begin();
    rec.startRecording(DataRate::HZ_10, /*stationary=*/true, /*noFix=*/true, /*shutdown=*/5);
    rec._onPacket(makeUnlockAck());  // trigger config send after unlock
    TEST_ASSERT_EQUAL_HEX8(UBX_ID_REC_CONFIG, g_stubLastSent.id);
    TEST_ASSERT_EQUAL_HEX8(10, g_stubLastSent.payload[1]);  // rate
    TEST_ASSERT_EQUAL_HEX8(1,  g_stubLastSent.payload[2]);  // stationaryFilter
    TEST_ASSERT_EQUAL_HEX8(1,  g_stubLastSent.payload[3]);  // noFixFilter
    TEST_ASSERT_EQUAL_HEX8(5,  g_stubLastSent.payload[4]);  // autoShutdownMin
}

// ── Runner ────────────────────────────────────────────────────────────────────
int main() {
    UNITY_BEGIN();
    RUN_TEST(test_initial_state_is_unknown);
    RUN_TEST(test_initial_record_count_is_zero);
    RUN_TEST(test_status_response_updates_state_idle);
    RUN_TEST(test_status_response_updates_state_recording);
    RUN_TEST(test_status_response_updates_state_paused);
    RUN_TEST(test_status_response_updates_record_count);
    RUN_TEST(test_status_response_updates_data_rate);
    RUN_TEST(test_status_response_too_short_ignored);
    RUN_TEST(test_status_large_record_count_le_decode);
    RUN_TEST(test_ack_sets_last_ack_true);
    RUN_TEST(test_nack_sets_last_ack_false);
    RUN_TEST(test_state_change_fires_callback);
    RUN_TEST(test_state_change_stop_fires_callback);
    RUN_TEST(test_state_change_without_callback_does_not_crash);
    RUN_TEST(test_state_change_too_short_ignored);
    RUN_TEST(test_wrong_class_packet_ignored);
    RUN_TEST(test_ack_timeout_clears_pending_after_5s);
    RUN_TEST(test_query_status_sends_correct_packet);
    RUN_TEST(test_start_recording_sends_unlock_then_config);
    RUN_TEST(test_stop_recording_sends_unlock_then_stop);
    RUN_TEST(test_start_with_filters_encodes_correctly);
    return UNITY_END();
}
