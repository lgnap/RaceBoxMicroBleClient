// test_downloader.cpp — native unit tests for RaceBoxDownloader state machine.
// Tests IDLE→REQUESTED→RECEIVING→DONE flow, NACK/error, state-change callbacks,
// progress percent, and the 30 s inactivity timeout — all without BLE hardware.
#include "unity.h"
#include "ubx.h"
#include "RaceBoxBle.h"
#include "RaceBoxDownloader.h"
#include "RaceBoxData.h"
#include <string.h>

// Spy state defined in RaceBoxBle_stub.cpp
extern UbxPacket g_stubLastSent;
extern int       g_stubSendCount;
extern bool      g_stubSendResult;
extern void      stubBleReset();
extern uint32_t  _fakeMillis;

void setUp()    { stubBleReset(); _fakeMillis = 0; }
void tearDown() {}

// ── Helpers ───────────────────────────────────────────────────────────────────

static UbxPacket makeDownloadResponse(uint32_t count) {
    UbxPacket p{};
    p.cls = UBX_CLASS_RACEBOX;
    p.id  = UBX_ID_DOWNLOAD;
    p.len = 4;
    p.payload[0] = (uint8_t)(count & 0xFF);
    p.payload[1] = (uint8_t)((count >> 8)  & 0xFF);
    p.payload[2] = (uint8_t)((count >> 16) & 0xFF);
    p.payload[3] = (uint8_t)((count >> 24) & 0xFF);
    return p;
}

// Build a minimal valid 80-byte history record (all-zero fields → raceBoxParse accepts it)
static UbxPacket makeHistoryRecord() {
    UbxPacket p{};
    p.cls = UBX_CLASS_RACEBOX;
    p.id  = UBX_ID_HISTORY;
    p.len = 80;
    memset(p.payload, 0, 80);
    return p;
}

static UbxPacket makeDownloadAck() {
    UbxPacket p{};
    p.cls = UBX_CLASS_RACEBOX;
    p.id  = UBX_ID_ACK;
    p.len = 2;
    p.payload[0] = UBX_CLASS_RACEBOX;
    p.payload[1] = UBX_ID_DOWNLOAD;
    return p;
}

static UbxPacket makeNack() {
    UbxPacket p{};
    p.cls = UBX_CLASS_RACEBOX;
    p.id  = UBX_ID_NACK;
    // Payload must identify the NACKed command: [class=0xFF, id=0x23 (download)]
    p.payload[0] = UBX_CLASS_RACEBOX;
    p.payload[1] = UBX_ID_DOWNLOAD;
    p.len = 2;
    return p;
}

static UbxPacket makeStateChange(uint8_t event) {
    UbxPacket p{};
    p.cls = UBX_CLASS_RACEBOX;
    p.id  = UBX_ID_STATE_CHANGE;
    p.len = 2;
    p.payload[0] = event;
    return p;
}

// Bring downloader to RECEIVING state: begin() then inject download-response
static void bringToReceiving(RaceBoxDownloader& dl, uint32_t expectedCount = 10) {
    dl.begin();
    dl._onPacket(makeDownloadResponse(expectedCount));
}

// ── Initial state ─────────────────────────────────────────────────────────────

void test_initial_state_not_done_not_error() {
    RaceBoxBle ble;
    RaceBoxDownloader dl(ble, nullptr);
    TEST_ASSERT_FALSE(dl.isDone());
    TEST_ASSERT_FALSE(dl.isError());
}

void test_initial_counts_are_zero() {
    RaceBoxBle ble;
    RaceBoxDownloader dl(ble, nullptr);
    TEST_ASSERT_EQUAL(0u, dl.recordCount());
    TEST_ASSERT_EQUAL(0u, dl.expectedCount());
}

// ── begin() ───────────────────────────────────────────────────────────────────

void test_begin_sends_download_trigger() {
    RaceBoxBle ble;
    RaceBoxDownloader dl(ble, nullptr);
    dl.begin();
    TEST_ASSERT_EQUAL_HEX8(UBX_CLASS_RACEBOX, g_stubLastSent.cls);
    TEST_ASSERT_EQUAL_HEX8(UBX_ID_DOWNLOAD,  g_stubLastSent.id);
    TEST_ASSERT_EQUAL(0, g_stubLastSent.len);
    TEST_ASSERT_EQUAL(1, g_stubSendCount);
}

void test_begin_failure_sets_error() {
    RaceBoxBle ble;
    RaceBoxDownloader dl(ble, nullptr);
    g_stubSendResult = false;
    dl.begin();
    TEST_ASSERT_TRUE(dl.isError());
}

// ── Download response (0xFF/0x23) ─────────────────────────────────────────────

void test_download_response_sets_expected_count() {
    RaceBoxBle ble;
    RaceBoxDownloader dl(ble, nullptr);
    dl.begin();
    dl._onPacket(makeDownloadResponse(42u));
    TEST_ASSERT_EQUAL(42u, dl.expectedCount());
}

void test_download_response_le_decode() {
    RaceBoxBle ble;
    RaceBoxDownloader dl(ble, nullptr);
    dl.begin();
    dl._onPacket(makeDownloadResponse(0x01020304u));
    TEST_ASSERT_EQUAL(0x01020304u, dl.expectedCount());
}

void test_download_response_too_short_ignored() {
    RaceBoxBle ble;
    RaceBoxDownloader dl(ble, nullptr);
    dl.begin();
    UbxPacket p{};
    p.cls = UBX_CLASS_RACEBOX;
    p.id  = UBX_ID_DOWNLOAD;
    p.len = 3;  // needs 4 bytes
    dl._onPacket(p);
    TEST_ASSERT_EQUAL(0u, dl.expectedCount());
    TEST_ASSERT_FALSE(dl.isDone());
}

// ── History records (0xFF/0x21) ───────────────────────────────────────────────

void test_history_record_fires_callback() {
    RaceBoxBle ble;
    bool called = false;
    RaceBoxDownloader dl(ble, [&](const RaceBoxData&, uint32_t) { called = true; });
    bringToReceiving(dl, 5);
    dl._onPacket(makeHistoryRecord());
    TEST_ASSERT_TRUE(called);
}

void test_history_record_index_is_zero_based() {
    RaceBoxBle ble;
    uint32_t lastIndex = 0xFFFFFFFF;
    RaceBoxDownloader dl(ble, [&](const RaceBoxData&, uint32_t idx) { lastIndex = idx; });
    bringToReceiving(dl, 5);
    dl._onPacket(makeHistoryRecord());  // index = 0
    TEST_ASSERT_EQUAL(0u, lastIndex);
    dl._onPacket(makeHistoryRecord());  // index = 1
    TEST_ASSERT_EQUAL(1u, lastIndex);
}

void test_history_record_increments_count() {
    RaceBoxBle ble;
    RaceBoxDownloader dl(ble, [](const RaceBoxData&, uint32_t) {});
    bringToReceiving(dl, 5);
    dl._onPacket(makeHistoryRecord());
    dl._onPacket(makeHistoryRecord());
    dl._onPacket(makeHistoryRecord());
    TEST_ASSERT_EQUAL(3u, dl.recordCount());
}

void test_history_record_ignored_before_response() {
    // State must be RECEIVING to process history records; REQUESTED must reject them
    RaceBoxBle ble;
    bool called = false;
    RaceBoxDownloader dl(ble, [&](const RaceBoxData&, uint32_t) { called = true; });
    dl.begin();  // state = REQUESTED (no download-response yet)
    dl._onPacket(makeHistoryRecord());
    TEST_ASSERT_FALSE(called);
    TEST_ASSERT_EQUAL(0u, dl.recordCount());
}

// ── State change (0xFF/0x26) ──────────────────────────────────────────────────

void test_state_change_fires_callback() {
    RaceBoxBle ble;
    uint8_t captured = 0xFF;
    RaceBoxDownloader dl(ble, nullptr);
    dl.setStateChangeCallback([&](uint8_t e) { captured = e; });
    bringToReceiving(dl);
    dl._onPacket(makeStateChange(0x01));
    TEST_ASSERT_EQUAL(0x01, captured);
}

void test_state_change_without_callback_does_not_crash() {
    RaceBoxBle ble;
    RaceBoxDownloader dl(ble, nullptr);  // no state-change callback set
    bringToReceiving(dl);
    dl._onPacket(makeStateChange(0x02));
    TEST_PASS();
}

void test_state_change_too_short_ignored() {
    RaceBoxBle ble;
    bool fired = false;
    RaceBoxDownloader dl(ble, nullptr);
    dl.setStateChangeCallback([&](uint8_t) { fired = true; });
    bringToReceiving(dl);
    UbxPacket p{};
    p.cls = UBX_CLASS_RACEBOX;
    p.id  = UBX_ID_STATE_CHANGE;
    p.len = 0;  // no event byte
    dl._onPacket(p);
    TEST_ASSERT_FALSE(fired);
}

// ── ACK — download complete (0xFF/0x02 with payload [0xFF, 0x23]) ─────────────

void test_ack_completes_download() {
    RaceBoxBle ble;
    RaceBoxDownloader dl(ble, nullptr);
    bringToReceiving(dl);
    dl._onPacket(makeDownloadAck());
    TEST_ASSERT_TRUE(dl.isDone());
}

void test_ack_wrong_payload_not_completed() {
    RaceBoxBle ble;
    RaceBoxDownloader dl(ble, nullptr);
    bringToReceiving(dl);
    UbxPacket p{};
    p.cls = UBX_CLASS_RACEBOX;
    p.id  = UBX_ID_ACK;
    p.len = 2;
    p.payload[0] = 0x01;         // wrong class (standard UBX, not 0xFF)
    p.payload[1] = UBX_ID_DOWNLOAD;
    dl._onPacket(p);
    TEST_ASSERT_FALSE(dl.isDone());
}

void test_ack_too_short_ignored() {
    RaceBoxBle ble;
    RaceBoxDownloader dl(ble, nullptr);
    bringToReceiving(dl);
    UbxPacket p{};
    p.cls = UBX_CLASS_RACEBOX;
    p.id  = UBX_ID_ACK;
    p.len = 1;  // needs 2 bytes
    dl._onPacket(p);
    TEST_ASSERT_FALSE(dl.isDone());
}

// ── NACK (0xFF/0x03) ──────────────────────────────────────────────────────────

void test_nack_sets_error() {
    RaceBoxBle ble;
    RaceBoxDownloader dl(ble, nullptr);
    dl.begin();
    dl._onPacket(makeNack());
    TEST_ASSERT_TRUE(dl.isError());
}

// ── Wrong class ───────────────────────────────────────────────────────────────

void test_wrong_class_packet_ignored() {
    RaceBoxBle ble;
    RaceBoxDownloader dl(ble, nullptr);
    bringToReceiving(dl, 5);
    UbxPacket p{};
    p.cls = 0x01;        // standard UBX, not RaceBox
    p.id  = UBX_ID_HISTORY;
    p.len = 80;
    dl._onPacket(p);
    TEST_ASSERT_EQUAL(0u, dl.recordCount());
}

// ── progressPercent() ─────────────────────────────────────────────────────────

void test_progress_zero_when_expected_is_zero() {
    RaceBoxBle ble;
    RaceBoxDownloader dl(ble, nullptr);
    TEST_ASSERT_EQUAL(0, dl.progressPercent());
}

void test_progress_calculates_correctly() {
    RaceBoxBle ble;
    RaceBoxDownloader dl(ble, [](const RaceBoxData&, uint32_t) {});
    bringToReceiving(dl, 100);
    for (int i = 0; i < 50; i++) dl._onPacket(makeHistoryRecord());
    TEST_ASSERT_EQUAL(50, dl.progressPercent());
}

void test_progress_caps_at_100() {
    // If more records arrive than expected, progress should not exceed 100
    RaceBoxBle ble;
    RaceBoxDownloader dl(ble, [](const RaceBoxData&, uint32_t) {});
    bringToReceiving(dl, 2);
    for (int i = 0; i < 5; i++) dl._onPacket(makeHistoryRecord());
    TEST_ASSERT_EQUAL(100, dl.progressPercent());
}

// ── 30 s inactivity timeout ───────────────────────────────────────────────────

void test_timeout_sets_error_after_30s() {
    RaceBoxBle ble;
    RaceBoxDownloader dl(ble, nullptr);
    dl.begin();  // _lastRxMs set to millis() = 0

    _fakeMillis = 30001;
    dl.update();
    TEST_ASSERT_TRUE(dl.isError());
}

void test_no_error_just_before_timeout() {
    RaceBoxBle ble;
    RaceBoxDownloader dl(ble, nullptr);
    dl.begin();

    _fakeMillis = 29999;
    dl.update();
    TEST_ASSERT_FALSE(dl.isError());
}

void test_timeout_with_all_records_received_sets_done() {
    // All expected records received but ACK lost — timeout must declare DONE not ERROR.
    RaceBoxBle ble;
    RaceBoxDownloader dl(ble, [](const RaceBoxData&, uint32_t) {});
    bringToReceiving(dl, 3);
    dl._onPacket(makeHistoryRecord());
    dl._onPacket(makeHistoryRecord());
    dl._onPacket(makeHistoryRecord());
    // recordCount == expectedCount == 3, but ACK never arrives
    _fakeMillis = 30001;
    dl.update();
    TEST_ASSERT_TRUE(dl.isDone());
    TEST_ASSERT_FALSE(dl.isError());
}

void test_timeout_with_partial_records_still_sets_error() {
    // Only some records received — genuine timeout, must stay ERROR.
    RaceBoxBle ble;
    RaceBoxDownloader dl(ble, [](const RaceBoxData&, uint32_t) {});
    bringToReceiving(dl, 10);
    dl._onPacket(makeHistoryRecord());
    dl._onPacket(makeHistoryRecord());
    // recordCount=2, expectedCount=10
    _fakeMillis = 30001;
    dl.update();
    TEST_ASSERT_TRUE(dl.isError());
    TEST_ASSERT_FALSE(dl.isDone());
}

void test_timeout_with_expected_zero_sets_error() {
    // expectedCount == 0 (no download response received) — must be ERROR not DONE.
    RaceBoxBle ble;
    RaceBoxDownloader dl(ble, nullptr);
    dl.begin();  // state=REQUESTED, _expectedCount=0
    _fakeMillis = 30001;
    dl.update();
    TEST_ASSERT_TRUE(dl.isError());
    TEST_ASSERT_FALSE(dl.isDone());
}

// ── Security code / memory unlock (0xFF/0x30) ─────────────────────────────────

static UbxPacket makeUnlockAck() {
    UbxPacket p{};
    p.cls = UBX_CLASS_RACEBOX;
    p.id  = UBX_ID_ACK;
    p.len = 2;
    p.payload[0] = UBX_CLASS_RACEBOX;
    p.payload[1] = UBX_ID_MEM_UNLOCK;
    return p;
}

static UbxPacket makeUnlockNack() {
    UbxPacket p{};
    p.cls = UBX_CLASS_RACEBOX;
    p.id  = UBX_ID_NACK;
    p.len = 2;
    p.payload[0] = UBX_CLASS_RACEBOX;
    p.payload[1] = UBX_ID_MEM_UNLOCK;
    return p;
}

void test_no_security_code_sends_download_trigger() {
    // Without a security code, begin() must send 0xFF/0x23 directly.
    RaceBoxBle ble;
    RaceBoxDownloader dl(ble, nullptr);
    dl.begin();
    TEST_ASSERT_EQUAL_HEX8(UBX_ID_DOWNLOAD, g_stubLastSent.id);
    TEST_ASSERT_EQUAL(1, g_stubSendCount);
}

void test_security_code_sends_unlock_first() {
    // With a security code, begin() must send 0xFF/0x30 (unlock) first.
    RaceBoxBle ble;
    RaceBoxDownloader dl(ble, nullptr);
    dl.setSecurityCode(123456);
    dl.begin();
    TEST_ASSERT_EQUAL_HEX8(UBX_ID_MEM_UNLOCK, g_stubLastSent.id);
    TEST_ASSERT_EQUAL(1, g_stubSendCount);
}

void test_security_code_payload_little_endian() {
    // Payload must be 4-byte LE encoding of the security code.
    RaceBoxBle ble;
    RaceBoxDownloader dl(ble, nullptr);
    dl.setSecurityCode(0x01020304u);
    dl.begin();
    TEST_ASSERT_EQUAL_HEX8(0x04, g_stubLastSent.payload[0]);
    TEST_ASSERT_EQUAL_HEX8(0x03, g_stubLastSent.payload[1]);
    TEST_ASSERT_EQUAL_HEX8(0x02, g_stubLastSent.payload[2]);
    TEST_ASSERT_EQUAL_HEX8(0x01, g_stubLastSent.payload[3]);
    TEST_ASSERT_EQUAL(4, g_stubLastSent.len);
}

void test_unlock_ack_sends_download_trigger() {
    // After ACK for 0xFF/0x30, the downloader must send 0xFF/0x23.
    RaceBoxBle ble;
    RaceBoxDownloader dl(ble, nullptr);
    dl.setSecurityCode(123456);
    dl.begin();
    // Now inject the unlock ACK
    dl._onPacket(makeUnlockAck());
    TEST_ASSERT_EQUAL(2, g_stubSendCount);  // unlock + download trigger
    TEST_ASSERT_EQUAL_HEX8(UBX_ID_DOWNLOAD, g_stubLastSent.id);
    TEST_ASSERT_FALSE(dl.isError());
}

void test_unlock_nack_sets_error() {
    // NACK for 0xFF/0x30 means wrong security code — must go to ERROR.
    RaceBoxBle ble;
    RaceBoxDownloader dl(ble, nullptr);
    dl.setSecurityCode(999999);
    dl.begin();
    dl._onPacket(makeUnlockNack());
    TEST_ASSERT_TRUE(dl.isError());
    TEST_ASSERT_EQUAL(1, g_stubSendCount);  // only the unlock was sent
}

void test_unlock_ack_then_download_response_receives_records() {
    // Full flow: unlock ACK → download trigger → download response → records
    RaceBoxBle ble;
    uint32_t count = 0;
    RaceBoxDownloader dl(ble, [&](const RaceBoxData&, uint32_t) { count++; });
    dl.setSecurityCode(123456);
    dl.begin();
    dl._onPacket(makeUnlockAck());
    dl._onPacket(makeDownloadResponse(3));
    dl._onPacket(makeHistoryRecord());
    dl._onPacket(makeHistoryRecord());
    dl._onPacket(makeHistoryRecord());
    dl._onPacket(makeDownloadAck());
    TEST_ASSERT_EQUAL(3u, count);
    TEST_ASSERT_TRUE(dl.isDone());
}

void test_reset_security_code_to_zero_disables_unlock() {
    // setSecurityCode(0) must disable unlock (back to direct trigger).
    RaceBoxBle ble;
    RaceBoxDownloader dl(ble, nullptr);
    dl.setSecurityCode(123456);
    dl.setSecurityCode(0);
    dl.begin();
    TEST_ASSERT_EQUAL_HEX8(UBX_ID_DOWNLOAD, g_stubLastSent.id);
    TEST_ASSERT_EQUAL(1, g_stubSendCount);
}

// ── Runner ────────────────────────────────────────────────────────────────────
int main() {
    UNITY_BEGIN();
    RUN_TEST(test_initial_state_not_done_not_error);
    RUN_TEST(test_initial_counts_are_zero);
    RUN_TEST(test_begin_sends_download_trigger);
    RUN_TEST(test_begin_failure_sets_error);
    RUN_TEST(test_download_response_sets_expected_count);
    RUN_TEST(test_download_response_le_decode);
    RUN_TEST(test_download_response_too_short_ignored);
    RUN_TEST(test_history_record_fires_callback);
    RUN_TEST(test_history_record_index_is_zero_based);
    RUN_TEST(test_history_record_increments_count);
    RUN_TEST(test_history_record_ignored_before_response);
    RUN_TEST(test_state_change_fires_callback);
    RUN_TEST(test_state_change_without_callback_does_not_crash);
    RUN_TEST(test_state_change_too_short_ignored);
    RUN_TEST(test_ack_completes_download);
    RUN_TEST(test_ack_wrong_payload_not_completed);
    RUN_TEST(test_ack_too_short_ignored);
    RUN_TEST(test_nack_sets_error);
    RUN_TEST(test_wrong_class_packet_ignored);
    RUN_TEST(test_progress_zero_when_expected_is_zero);
    RUN_TEST(test_progress_calculates_correctly);
    RUN_TEST(test_progress_caps_at_100);
    RUN_TEST(test_timeout_sets_error_after_30s);
    RUN_TEST(test_no_error_just_before_timeout);
    RUN_TEST(test_timeout_with_all_records_received_sets_done);
    RUN_TEST(test_timeout_with_partial_records_still_sets_error);
    RUN_TEST(test_timeout_with_expected_zero_sets_error);
    RUN_TEST(test_no_security_code_sends_download_trigger);
    RUN_TEST(test_security_code_sends_unlock_first);
    RUN_TEST(test_security_code_payload_little_endian);
    RUN_TEST(test_unlock_ack_sends_download_trigger);
    RUN_TEST(test_unlock_nack_sets_error);
    RUN_TEST(test_unlock_ack_then_download_response_receives_records);
    RUN_TEST(test_reset_security_code_to_zero_disables_unlock);
    return UNITY_END();
}
