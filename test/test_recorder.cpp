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

// Build a status response packet (0xFF/0x22) per protocol rev 8:
//   [0] state, [1] memLevel%, [2] security, [3] reserved,
//   [4..7] recordCount LE, [8..11] memorySize LE
static UbxPacket makeStatusResponse(uint8_t state, uint8_t memLevel, uint32_t count,
                                    uint32_t memSize = 0) {
    UbxPacket pkt{};
    pkt.cls = UBX_CLASS_RACEBOX;
    pkt.id  = UBX_ID_REC_STATUS;
    pkt.len = 12;
    pkt.payload[0] = state;
    pkt.payload[1] = memLevel;
    pkt.payload[2] = 0;  // security
    pkt.payload[3] = 0;  // reserved
    pkt.payload[4] = (uint8_t)(count & 0xFF);
    pkt.payload[5] = (uint8_t)((count >> 8)  & 0xFF);
    pkt.payload[6] = (uint8_t)((count >> 16) & 0xFF);
    pkt.payload[7] = (uint8_t)((count >> 24) & 0xFF);
    pkt.payload[8]  = (uint8_t)(memSize & 0xFF);
    pkt.payload[9]  = (uint8_t)((memSize >> 8)  & 0xFF);
    pkt.payload[10] = (uint8_t)((memSize >> 16) & 0xFF);
    pkt.payload[11] = (uint8_t)((memSize >> 24) & 0xFF);
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

// Full 12-byte STATE_CHANGE (mirrors REC CONFIG payload after [0..1])
static UbxPacket makeStateChangeFull(uint8_t state, uint8_t rate, uint8_t flags,
                                     uint16_t statSpd, uint16_t statInt,
                                     uint16_t nofixInt, uint16_t shutdownSecs) {
    UbxPacket p{};
    p.cls        = UBX_CLASS_RACEBOX;
    p.id         = UBX_ID_STATE_CHANGE;
    p.len        = 12;
    p.payload[0]  = state;
    p.payload[1]  = 0;  // reserved
    p.payload[2]  = rate;
    p.payload[3]  = flags;
    p.payload[4]  = (uint8_t)(statSpd & 0xFF);
    p.payload[5]  = (uint8_t)((statSpd >> 8) & 0xFF);
    p.payload[6]  = (uint8_t)(statInt & 0xFF);
    p.payload[7]  = (uint8_t)((statInt >> 8) & 0xFF);
    p.payload[8]  = (uint8_t)(nofixInt & 0xFF);
    p.payload[9]  = (uint8_t)((nofixInt >> 8) & 0xFF);
    p.payload[10] = (uint8_t)(shutdownSecs & 0xFF);
    p.payload[11] = (uint8_t)((shutdownSecs >> 8) & 0xFF);
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
    rec._onPacket(makeStatusResponse(0, 34, 0));  // IDLE, 34% mem, 0 records
    TEST_ASSERT_EQUAL((int)RecordingState::IDLE, (int)rec.state());
}

void test_status_response_updates_state_recording() {
    RaceBoxBle ble;
    RaceBoxRecorder rec(ble);
    rec._onPacket(makeStatusResponse(1, 50, 42));
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
    rec._onPacket(makeStatusResponse(1, 50, 123456u));
    TEST_ASSERT_EQUAL(123456u, rec.recordCount());
}

void test_status_response_updates_memory_level() {
    RaceBoxBle ble;
    RaceBoxRecorder rec(ble);
    rec._onPacket(makeStatusResponse(1, 42, 0));
    TEST_ASSERT_EQUAL(42u, rec.memoryLevel());
}

void test_status_response_too_short_ignored() {
    RaceBoxBle ble;
    RaceBoxRecorder rec(ble);
    // Payload < 8 bytes — must be ignored (need at least state + memLevel + 4 bytes count)
    UbxPacket p{};
    p.cls = UBX_CLASS_RACEBOX;
    p.id  = UBX_ID_REC_STATUS;
    p.len = 6;
    rec._onPacket(p);
    // State must remain UNKNOWN
    TEST_ASSERT_EQUAL((int)RecordingState::UNKNOWN, (int)rec.state());
}

void test_status_large_record_count_le_decode() {
    RaceBoxBle ble;
    RaceBoxRecorder rec(ble);
    // 0x01020304 = 16909060; stored at payload[4..7] in LE: [04 03 02 01]
    rec._onPacket(makeStatusResponse(1, 50, 0x01020304u));
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
    // Protocol state 1 = running → RECORDING_START
    rec._onPacket(makeStateChange(1));
    TEST_ASSERT_EQUAL((int)StateChangeEvent::RECORDING_START, (int)captured);
}

void test_state_change_stop_fires_callback() {
    RaceBoxBle ble;
    RaceBoxRecorder rec(ble);
    StateChangeEvent captured = (StateChangeEvent)0xFF;
    rec.setStateChangeCallback([&](StateChangeEvent e) { captured = e; });
    // Protocol state 0 = disabled → RECORDING_STOP
    rec._onPacket(makeStateChange(0));
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
    TEST_ASSERT_EQUAL(12, g_stubLastSent.len);
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
    TEST_ASSERT_EQUAL(12, g_stubLastSent.len);
    TEST_ASSERT_EQUAL_HEX8(0x00, g_stubLastSent.payload[0]);  // command = stop
}

void test_start_with_filters_encodes_correctly() {
    RaceBoxBle ble;
    RaceBoxRecorder rec(ble);
    rec.begin();
    rec.startRecording(DataRate::HZ_10, /*stationary=*/true, /*noFix=*/true, /*shutdown=*/5);
    rec._onPacket(makeUnlockAck());  // trigger config send after unlock
    TEST_ASSERT_EQUAL_HEX8(UBX_ID_REC_CONFIG, g_stubLastSent.id);
    TEST_ASSERT_EQUAL(12, g_stubLastSent.len);
    TEST_ASSERT_EQUAL_HEX8(10,   g_stubLastSent.payload[1]);  // rate = 10 Hz
    // flags: bit1=stationary, bit2=noFix, bit3=autoShutdown → 0b00001110 = 0x0E
    TEST_ASSERT_EQUAL_HEX8(0x0E, g_stubLastSent.payload[2]);
    TEST_ASSERT_EQUAL_HEX8(0,    g_stubLastSent.payload[3]);  // reserved
    // stationary speed threshold: 1389 mm/s LE = 0x6D 0x05
    TEST_ASSERT_EQUAL_HEX8(0x6D, g_stubLastSent.payload[4]);
    TEST_ASSERT_EQUAL_HEX8(0x05, g_stubLastSent.payload[5]);
    // auto-shutdown: 5 min = 300 s = 0x012C LE = 0x2C 0x01
    TEST_ASSERT_EQUAL_HEX8(0x2C, g_stubLastSent.payload[10]);
    TEST_ASSERT_EQUAL_HEX8(0x01, g_stubLastSent.payload[11]);
}

// ── setConfig() + no-args startRecording() ────────────────────────────────────

void test_setConfig_stores_rate() {
    RaceBoxBle ble;
    RaceBoxRecorder rec(ble);
    rec.setConfig(DataRate::HZ_10, false, false, 0);
    TEST_ASSERT_EQUAL((int)DataRate::HZ_10, (int)rec.config().rate);
}

void test_setConfig_stores_filters() {
    RaceBoxBle ble;
    RaceBoxRecorder rec(ble);
    rec.setConfig(DataRate::HZ_5, true, true, 3);
    TEST_ASSERT_TRUE(rec.config().stationaryFilter);
    TEST_ASSERT_TRUE(rec.config().noFixFilter);
    TEST_ASSERT_EQUAL(3, rec.config().autoShutdownMin);
}

void test_startRecording_noargs_uses_stored_config_rate() {
    RaceBoxBle ble;
    RaceBoxRecorder rec(ble);
    rec.begin();
    rec.setConfig(DataRate::HZ_5, false, false, 0);
    rec.startRecording();  // no-args: should use HZ_5
    rec._onPacket(makeUnlockAck());
    TEST_ASSERT_EQUAL_HEX8(UBX_ID_REC_CONFIG, g_stubLastSent.id);
    TEST_ASSERT_EQUAL_HEX8(5, g_stubLastSent.payload[1]);  // rate = 5 Hz
}

void test_explicit_startRecording_updates_stored_config() {
    RaceBoxBle ble;
    RaceBoxRecorder rec(ble);
    rec.begin();
    rec.setConfig(DataRate::HZ_5, false, false, 0);
    rec.startRecording(DataRate::HZ_25, /*stationary=*/true, false, 0);
    TEST_ASSERT_EQUAL((int)DataRate::HZ_25, (int)rec.config().rate);
    TEST_ASSERT_TRUE(rec.config().stationaryFilter);
}

// ── Full STATE_CHANGE payload parsing (issue #21) ─────────────────────────────

void test_state_change_full_payload_populates_confirmed_config() {
    RaceBoxBle ble;
    RaceBoxRecorder rec(ble);
    // rate=10, flags: bit1=stationary + bit2=nofix = 0x06, statSpd=1389, intervals=30, shutdown=0
    UbxPacket pkt = makeStateChangeFull(1, 10, 0x06, 1389, 30, 30, 0);
    rec._onPacket(pkt);
    const RecordingConfig& c = rec.confirmedConfig();
    TEST_ASSERT_EQUAL((int)DataRate::HZ_10, (int)c.rate);
    TEST_ASSERT_TRUE(c.stationaryFilter);
    TEST_ASSERT_TRUE(c.noFixFilter);
    TEST_ASSERT_EQUAL(1389u, c.stationarySpeedMmS);
    TEST_ASSERT_EQUAL(30u,   c.stationaryIntervalS);
    TEST_ASSERT_EQUAL(30u,   c.noFixIntervalS);
    TEST_ASSERT_EQUAL(0u,    c.autoShutdownSecs);
}

void test_state_change_full_payload_fires_config_confirmed_callback() {
    RaceBoxBle ble;
    RaceBoxRecorder rec(ble);
    bool cbFired = false;
    DataRate capturedRate = DataRate::HZ_1;
    rec.setConfigConfirmedCallback([&](const RecordingConfig& c) {
        cbFired = true;
        capturedRate = c.rate;
    });
    UbxPacket pkt = makeStateChangeFull(1, 25, 0x00, 0, 0, 0, 0);
    rec._onPacket(pkt);
    TEST_ASSERT_TRUE(cbFired);
    TEST_ASSERT_EQUAL((int)DataRate::HZ_25, (int)capturedRate);
}

void test_state_change_stop_does_not_update_confirmed_config() {
    RaceBoxBle ble;
    RaceBoxRecorder rec(ble);
    // First: a full start → confirmed rate = HZ_10
    rec._onPacket(makeStateChangeFull(1, 10, 0x00, 0, 0, 0, 0));
    TEST_ASSERT_EQUAL((int)DataRate::HZ_10, (int)rec.confirmedConfig().rate);
    // Then: stop (state=0) — confirmed should NOT change
    rec._onPacket(makeStateChangeFull(0, 25, 0x00, 0, 0, 0, 0));
    TEST_ASSERT_EQUAL((int)DataRate::HZ_10, (int)rec.confirmedConfig().rate);
}

void test_state_change_no_confirmed_callback_does_not_crash() {
    RaceBoxBle ble;
    RaceBoxRecorder rec(ble);
    // No callback set — must not crash
    UbxPacket pkt = makeStateChangeFull(1, 25, 0x06, 1389, 30, 30, 300);
    rec._onPacket(pkt);
    TEST_PASS();
}

void test_state_change_short_payload_no_confirmed_update() {
    RaceBoxBle ble;
    RaceBoxRecorder rec(ble);
    // len < 12 — confirmed config must remain at zero defaults
    UbxPacket p{};
    p.cls = UBX_CLASS_RACEBOX;
    p.id  = UBX_ID_STATE_CHANGE;
    p.len = 3;   // state + reserved + rate only
    p.payload[0] = 1;  // RECORDING_START
    p.payload[2] = 10;
    rec._onPacket(p);
    TEST_ASSERT_EQUAL(0u, rec.confirmedConfig().stationarySpeedMmS);
}

// ── Erase ─────────────────────────────────────────────────────────────────────

static UbxPacket makeEraseAck() {
    UbxPacket p{};
    p.cls = UBX_CLASS_RACEBOX;
    p.id  = UBX_ID_ACK;
    p.len = 2;
    p.payload[0] = UBX_CLASS_RACEBOX;
    p.payload[1] = UBX_ID_ERASE;
    return p;
}

static UbxPacket makeEraseProgress(uint8_t pct) {
    UbxPacket p{};
    p.cls        = UBX_CLASS_RACEBOX;
    p.id         = UBX_ID_ERASE;
    p.len        = 1;
    p.payload[0] = pct;
    return p;
}

void test_erase_sends_unlock_then_erase_command() {
    RaceBoxBle ble;
    RaceBoxRecorder rec(ble);
    rec.begin();
    rec.eraseMemory();
    // Step 1: unlock sent
    TEST_ASSERT_EQUAL_HEX8(UBX_ID_MEM_UNLOCK, g_stubLastSent.id);
    TEST_ASSERT_FALSE(rec.isErasing());  // not yet — waiting for unlock ACK
    // Step 2: simulate unlock ACK → erase command sent
    rec._onPacket(makeUnlockAck());
    TEST_ASSERT_EQUAL_HEX8(UBX_ID_ERASE, g_stubLastSent.id);
    TEST_ASSERT_EQUAL(0, g_stubLastSent.len);  // empty payload = start
    TEST_ASSERT_TRUE(rec.isErasing());
}

void test_erase_progress_callback_fires() {
    RaceBoxBle ble;
    RaceBoxRecorder rec(ble);
    rec.begin();
    uint8_t captured = 0xFF;
    rec.setEraseProgressCallback([&](uint8_t pct) { captured = pct; });
    // Inject a progress notification (59%)
    rec._onPacket(makeEraseProgress(59));
    TEST_ASSERT_EQUAL(59, captured);
    TEST_ASSERT_EQUAL(59, rec.eraseProgress());
}

void test_erase_complete_on_erase_ack() {
    RaceBoxBle ble;
    RaceBoxRecorder rec(ble);
    rec.begin();
    rec.eraseMemory();
    rec._onPacket(makeUnlockAck());  // start erase
    TEST_ASSERT_TRUE(rec.isErasing());

    bool doneFired = false;
    rec.setEraseProgressCallback([&](uint8_t pct) {
        if (pct == 100) doneFired = true;
    });
    rec._onPacket(makeEraseAck());
    TEST_ASSERT_FALSE(rec.isErasing());
    TEST_ASSERT_EQUAL(100, rec.eraseProgress());
    TEST_ASSERT_TRUE(doneFired);
}

void test_cancel_erase_sends_erase_with_payload() {
    RaceBoxBle ble;
    RaceBoxRecorder rec(ble);
    rec.begin();
    rec.eraseMemory();
    rec._onPacket(makeUnlockAck());  // start erase
    TEST_ASSERT_TRUE(rec.isErasing());

    stubBleReset();
    rec.cancelErase();
    TEST_ASSERT_FALSE(rec.isErasing());
    TEST_ASSERT_EQUAL_HEX8(UBX_ID_ERASE, g_stubLastSent.id);
    TEST_ASSERT_EQUAL(1, g_stubLastSent.len);  // 1-byte payload = cancel
}

void test_erase_not_started_twice() {
    RaceBoxBle ble;
    RaceBoxRecorder rec(ble);
    rec.begin();
    rec.eraseMemory();
    rec._onPacket(makeUnlockAck());
    TEST_ASSERT_TRUE(rec.isErasing());
    int countBefore = g_stubSendCount;
    rec.eraseMemory();  // second call — must be ignored
    TEST_ASSERT_EQUAL(countBefore, g_stubSendCount);
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
    RUN_TEST(test_status_response_updates_memory_level);
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
    RUN_TEST(test_setConfig_stores_rate);
    RUN_TEST(test_setConfig_stores_filters);
    RUN_TEST(test_startRecording_noargs_uses_stored_config_rate);
    RUN_TEST(test_explicit_startRecording_updates_stored_config);
    RUN_TEST(test_state_change_full_payload_populates_confirmed_config);
    RUN_TEST(test_state_change_full_payload_fires_config_confirmed_callback);
    RUN_TEST(test_state_change_stop_does_not_update_confirmed_config);
    RUN_TEST(test_state_change_no_confirmed_callback_does_not_crash);
    RUN_TEST(test_state_change_short_payload_no_confirmed_update);
    RUN_TEST(test_erase_sends_unlock_then_erase_command);
    RUN_TEST(test_erase_progress_callback_fires);
    RUN_TEST(test_erase_complete_on_erase_ack);
    RUN_TEST(test_cancel_erase_sends_erase_with_payload);
    RUN_TEST(test_erase_not_started_twice);
    return UNITY_END();
}
