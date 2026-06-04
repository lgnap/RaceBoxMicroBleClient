#include "RaceBoxRecorder.h"
#include "RaceBoxBle.h"
#include <Arduino.h>
#include <string.h>

// ACK timeout (ms) — if device doesn't respond within this time, assume failure
static constexpr uint32_t ACK_TIMEOUT_MS = 5000;

// ── Constructor ───────────────────────────────────────────────────────────────
RaceBoxRecorder::RaceBoxRecorder(RaceBoxBle& ble) : _ble(ble) {}

// ── begin() ───────────────────────────────────────────────────────────────────
void RaceBoxRecorder::begin() {
    _ble.setPacketCallback([this](const UbxPacket& pkt) {
        _onPacket(pkt);
    });
}

// ── update() ─────────────────────────────────────────────────────────────────
void RaceBoxRecorder::update() {
    // ACK timeout check (only when a command is pending)
    if (_cmdSentMs != 0 && (millis() - _cmdSentMs) > ACK_TIMEOUT_MS) {
        Serial.println("[Recorder] ACK timeout");
        _lastAck     = false;
        _cmdSentMs   = 0;
        _pendingCmd  = _Pending::NONE;
    }
}

// ── _sendUnlock() — internal ─────────────────────────────────────────────────
bool RaceBoxRecorder::_sendUnlock() {
    UbxPacket pkt{};
    pkt.cls    = UBX_CLASS_RACEBOX;
    pkt.id     = UBX_ID_MEM_UNLOCK;
    pkt.len    = 4;
    pkt.payload[0] = (uint8_t)(_securityCode & 0xFF);
    pkt.payload[1] = (uint8_t)((_securityCode >> 8)  & 0xFF);
    pkt.payload[2] = (uint8_t)((_securityCode >> 16) & 0xFF);
    pkt.payload[3] = (uint8_t)((_securityCode >> 24) & 0xFF);
    if (_ble.sendCommand(pkt)) {
        _cmdSentMs = millis();
        Serial.printf("[Recorder] Unlock sent (code=%lu)\n", (unsigned long)_securityCode);
        return true;
    }
    return false;
}

// ── queryStatus() ─────────────────────────────────────────────────────────────
void RaceBoxRecorder::queryStatus() {
    UbxPacket pkt{};
    pkt.cls = UBX_CLASS_RACEBOX;
    pkt.id  = UBX_ID_REC_STATUS;
    pkt.len = 0;
    if (_ble.sendCommand(pkt)) {
        _cmdSentMs = millis();
        Serial.println("[Recorder] Status query sent");
    }
}

// ── startRecording() ──────────────────────────────────────────────────────────
void RaceBoxRecorder::startRecording(DataRate rate, bool stationaryFilter,
                                      bool noFixFilter, uint8_t autoShutdownMin) {
    // Save args, send unlock first; _sendConfig called on unlock ACK
    _pendingRate         = rate;
    _pendingStationary   = stationaryFilter;
    _pendingNoFix        = noFixFilter;
    _pendingAutoShutdown = autoShutdownMin;
    _pendingCmd          = _Pending::START;
    _sendUnlock();
}

// ── stopRecording() ───────────────────────────────────────────────────────────
void RaceBoxRecorder::stopRecording() {
    _pendingCmd = _Pending::STOP;
    _sendUnlock();
}

// ── _sendConfig() ─────────────────────────────────────────────────────────────
void RaceBoxRecorder::_sendConfig(uint8_t command, DataRate rate,
                                   bool stationaryFilter, bool noFixFilter,
                                   uint8_t autoShutdownMin) {
    UbxPacket pkt{};
    pkt.cls       = UBX_CLASS_RACEBOX;
    pkt.id        = UBX_ID_REC_CONFIG;
    pkt.len       = 8;
    memset(pkt.payload, 0, 8);
    pkt.payload[0] = command;
    pkt.payload[1] = static_cast<uint8_t>(rate);
    pkt.payload[2] = stationaryFilter ? 1 : 0;
    pkt.payload[3] = noFixFilter      ? 1 : 0;
    pkt.payload[4] = autoShutdownMin;
    // payload[5..7] reserved, already 0

    if (_ble.sendCommand(pkt)) {
        _cmdSentMs = millis();
        Serial.printf("[Recorder] Config sent: cmd=%d rate=%d\n", command, (int)rate);
    }
}

// ── _onPacket() — dispatched by RaceBoxBle ────────────────────────────────────
void RaceBoxRecorder::_onPacket(const UbxPacket& pkt) {
    if (pkt.cls != UBX_CLASS_RACEBOX) return;

    switch (pkt.id) {

    case UBX_ID_REC_STATUS:
        // 0xFF/0x22 response: state(1), dataRate(1), recordCount(4), memUsed(4)
        if (pkt.len >= 6) {
            _state       = static_cast<RecordingState>(pkt.payload[0]);
            _dataRate    = static_cast<DataRate>(pkt.payload[1]);
            _recordCount = static_cast<uint32_t>(pkt.payload[2])
                         | (static_cast<uint32_t>(pkt.payload[3]) << 8)
                         | (static_cast<uint32_t>(pkt.payload[4]) << 16)
                         | (static_cast<uint32_t>(pkt.payload[5]) << 24);
            _cmdSentMs = 0;
            Serial.printf("[Recorder] Status: state=%d rate=%d records=%lu\n",
                          (int)_state, (int)_dataRate, (unsigned long)_recordCount);
        }
        break;

    case UBX_ID_ACK:
        // 0xFF/0x02: payload = [class, id] of the command being ACKed
        _lastAck   = true;
        _cmdSentMs = 0;
        if (pkt.len >= 2 && pkt.payload[0] == UBX_CLASS_RACEBOX
                         && pkt.payload[1] == UBX_ID_MEM_UNLOCK) {
            // Unlock ACKed — fire the queued command
            Serial.println("[Recorder] Unlock ACK — sending queued command");
            _Pending pending = _pendingCmd;
            _pendingCmd = _Pending::NONE;
            if (pending == _Pending::START) {
                _sendConfig(0x01, _pendingRate, _pendingStationary,
                            _pendingNoFix, _pendingAutoShutdown);
            } else if (pending == _Pending::STOP) {
                _sendConfig(0x00, _dataRate, false, false, 0);
            }
        } else {
            Serial.println("[Recorder] ACK received");
        }
        break;

    case UBX_ID_NACK:
        // 0xFF/0x03
        _lastAck    = false;
        _cmdSentMs  = 0;
        _pendingCmd = _Pending::NONE;
        if (pkt.len >= 2 && pkt.payload[0] == UBX_CLASS_RACEBOX
                         && pkt.payload[1] == UBX_ID_MEM_UNLOCK) {
            Serial.println("[Recorder] Unlock NACK — wrong security code?");
        } else {
            Serial.println("[Recorder] NACK received");
        }
        break;

    case UBX_ID_STATE_CHANGE:
        // 0xFF/0x26: event(1), reserved(1)
        if (pkt.len >= 1 && _stateChangeCb) {
            _stateChangeCb(static_cast<StateChangeEvent>(pkt.payload[0]));
        }
        break;

    default:
        break;
    }
}
