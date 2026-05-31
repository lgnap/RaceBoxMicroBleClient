// Stub implementations of RaceBoxBle non-inline methods for native tests.
// Uses an explicit relative path so this file (in test/mocks/) compiles
// against the real header; FreeRTOS types are satisfied by test/mocks/freertos/.
//
// This file intentionally does NOT define a complete RaceBoxBle implementation —
// only the methods called by RaceBoxRecorder and RaceBoxDownloader are stubbed.
#include "../../src/RaceBoxBle.h"
#include <string.h>

// ── Spy state — accessible from test files via extern ────────────────────────
UbxPacket g_stubLastSent{};
int       g_stubSendCount  = 0;
bool      g_stubSendResult = true;  // set to false to simulate failure

void stubBleReset() {
    memset(&g_stubLastSent, 0, sizeof(g_stubLastSent));
    g_stubSendCount  = 0;
    g_stubSendResult = true;
}

// ── RaceBoxBle method stubs ───────────────────────────────────────────────────

RaceBoxBle::RaceBoxBle(RaceBoxLiveCallback cb) : _liveCb(std::move(cb)) {
    _fifoMutex = nullptr;
}

bool RaceBoxBle::sendCommand(const UbxPacket& pkt) {
    g_stubLastSent = pkt;
    g_stubSendCount++;
    return g_stubSendResult;
}

void RaceBoxBle::begin()   {}
void RaceBoxBle::update()  {}

// NimBLE bridge stubs — not used in unit tests
RaceBoxBle* RaceBoxBle::_instance = nullptr;
void RaceBoxBle::_handleNotification(const uint8_t*, size_t) {}
void RaceBoxBle::_handleConnect()    {}
void RaceBoxBle::_handleDisconnect() {}
void RaceBoxBle::_handleScanResult(const char*, const char*, uint8_t) {}

// FIFO stubs — not exercised in unit tests
void RaceBoxBle::_fifoPush(const uint8_t*, size_t) {}
bool RaceBoxBle::_fifoPeek(uint8_t*, size_t) const { return false; }
void RaceBoxBle::_fifoDrop(size_t) {}
void RaceBoxBle::_drainFifo() {}
