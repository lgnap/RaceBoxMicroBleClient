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
// REC CONFIG (0xFF/0x25) — 12-byte payload per protocol rev 8:
//   [0]     Enable (1=start, 0=stop)
//   [1]     Data Rate (Hz: 1/5/10/25)
//   [2]     Flags bitmask: bit0=WaitGnssFix, bit1=Stationary, bit2=NoFix, bit3=AutoShutdown, bit4=WaitForData
//   [3]     Reserved
//   [4..5]  Stationary speed threshold (UInt16 LE, mm/s)
//   [6..7]  Stationary detection interval (UInt16 LE, seconds)
//   [8..9]  No-fix detection interval (UInt16 LE, seconds)
//   [10..11] Auto-shutdown detection interval (UInt16 LE, seconds)
void RaceBoxRecorder::_sendConfig(uint8_t command, DataRate rate,
                                   bool stationaryFilter, bool noFixFilter,
                                   uint8_t autoShutdownMin) {
    UbxPacket pkt{};
    pkt.cls = UBX_CLASS_RACEBOX;
    pkt.id  = UBX_ID_REC_CONFIG;
    pkt.len = 12;
    memset(pkt.payload, 0, 12);

    pkt.payload[0] = command;
    pkt.payload[1] = static_cast<uint8_t>(rate);

    uint8_t flags = 0;
    if (stationaryFilter) flags |= (1 << 1);
    if (noFixFilter)      flags |= (1 << 2);
    if (autoShutdownMin)  flags |= (1 << 3);
    pkt.payload[2] = flags;
    // payload[3] = reserved = 0

    // Stationary filter: speed < 1389 mm/s (~5 km/h) for 30 s
    if (stationaryFilter) {
        pkt.payload[4] = 0x6D; pkt.payload[5] = 0x05;  // 1389 LE
        pkt.payload[6] = 30;   pkt.payload[7] = 0;
    }

    // No-fix filter: 30 s
    if (noFixFilter) {
        pkt.payload[8] = 30; pkt.payload[9] = 0;
    }

    // Auto-shutdown: convert minutes → seconds
    if (autoShutdownMin) {
        uint16_t secs = (uint16_t)autoShutdownMin * 60;
        pkt.payload[10] = secs & 0xFF;
        pkt.payload[11] = (secs >> 8) & 0xFF;
    }

    if (_ble.sendCommand(pkt)) {
        _cmdSentMs = millis();
        Serial.printf("[Recorder] Config sent: cmd=%d rate=%d flags=0x%02X\n",
                      command, (int)rate, flags);
    }
}

// ── _onPacket() — dispatched by RaceBoxBle ────────────────────────────────────
void RaceBoxRecorder::_onPacket(const UbxPacket& pkt) {
    if (pkt.cls != UBX_CLASS_RACEBOX) return;

    switch (pkt.id) {

    case UBX_ID_REC_STATUS:
        // 0xFF/0x22 response (12 bytes, protocol rev 8):
        //   [0]     Recording state (0=idle, non-zero=recording)
        //   [1]     Memory level (0..100 %)
        //   [2]     Memory security bitmask
        //   [3]     Reserved
        //   [4..7]  Stored record count (UInt32 LE)
        //   [8..11] Total memory size in records (UInt32 LE)
        if (pkt.len >= 8) {
            _state       = static_cast<RecordingState>(pkt.payload[0]);
            _memoryLevel = pkt.payload[1];
            _cmdSentMs   = 0;
            _recordCount = static_cast<uint32_t>(pkt.payload[4])
                         | (static_cast<uint32_t>(pkt.payload[5]) << 8)
                         | (static_cast<uint32_t>(pkt.payload[6]) << 16)
                         | (static_cast<uint32_t>(pkt.payload[7]) << 24);
            if (pkt.len >= 12) {
                _memorySize = static_cast<uint32_t>(pkt.payload[8])
                            | (static_cast<uint32_t>(pkt.payload[9])  << 8)
                            | (static_cast<uint32_t>(pkt.payload[10]) << 16)
                            | (static_cast<uint32_t>(pkt.payload[11]) << 24);
            }
            Serial.printf("[Recorder] Status: state=%d mem=%d%% records=%lu/%lu\n",
                          (int)_state, _memoryLevel,
                          (unsigned long)_recordCount, (unsigned long)_memorySize);
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
        // 0xFF/0x26 (12 bytes, protocol rev 8):
        //   [0]  Standalone Recording State (0=disabled, 1=running, 2=paused)
        //   [1]  Reserved
        //   [2]  Data Rate
        //   [3]  Flags
        //   [4..] filter/shutdown parameters (mirrors REC CONFIG payload)
        if (pkt.len >= 1) {
            _state = static_cast<RecordingState>(pkt.payload[0]);
            if (pkt.len >= 3) {
                _dataRate = static_cast<DataRate>(pkt.payload[2]);
            }
            if (_stateChangeCb) {
                _stateChangeCb(static_cast<StateChangeEvent>(pkt.payload[0]));
            }
        }
        break;

    default:
        break;
    }
}
