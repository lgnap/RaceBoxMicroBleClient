#include "RaceBoxDownloader.h"
#include "RaceBoxBle.h"
#include "RaceBoxParser.h"
#include <Arduino.h>

// ── Constructor ───────────────────────────────────────────────────────────────
RaceBoxDownloader::RaceBoxDownloader(RaceBoxBle& ble, HistoryCallback onRecord)
    : _ble(ble), _onRecord(std::move(onRecord)) {}

// ── begin() ───────────────────────────────────────────────────────────────────
void RaceBoxDownloader::begin() {
    _ble.setPacketCallback([this](const UbxPacket& pkt) {
        _onPacket(pkt);
    });

    if (_hasSecurityCode) {
        // Send memory unlock (0xFF/0x30) before the download trigger.
        // The device resets the lock on every new connection; unlocking is
        // required even for unsecured devices — sending the command is harmless
        // if security is not enabled (device ACKs regardless of the code).
        UbxPacket unlock{};
        unlock.cls        = UBX_CLASS_RACEBOX;
        unlock.id         = UBX_ID_MEM_UNLOCK;
        unlock.len        = 4;
        unlock.payload[0] = static_cast<uint8_t>(_securityCode);
        unlock.payload[1] = static_cast<uint8_t>(_securityCode >> 8);
        unlock.payload[2] = static_cast<uint8_t>(_securityCode >> 16);
        unlock.payload[3] = static_cast<uint8_t>(_securityCode >> 24);

        if (_ble.sendCommand(unlock)) {
            _state    = State::UNLOCKING;
            _lastRxMs = millis();
            Serial.println("[Downloader] Unlock sent — waiting for ACK");
        } else {
            _state = State::ERROR;
            Serial.println("[Downloader] Failed to send unlock (not connected?)");
        }
    } else {
        _sendDownloadTrigger();
    }
}

// ── _sendDownloadTrigger() ────────────────────────────────────────────────────
bool RaceBoxDownloader::_sendDownloadTrigger() {
    UbxPacket trigger{};
    trigger.cls = UBX_CLASS_RACEBOX;
    trigger.id  = UBX_ID_DOWNLOAD;
    trigger.len = 0;

    if (_ble.sendCommand(trigger)) {
        _state    = State::REQUESTED;
        _lastRxMs = millis();
        Serial.println("[Downloader] Download trigger sent");
        return true;
    } else {
        _state = State::ERROR;
        Serial.println("[Downloader] Failed to send trigger (not connected?)");
        return false;
    }
}

// ── eraseMemory() ─────────────────────────────────────────────────────────────
void RaceBoxDownloader::eraseMemory() {
    UbxPacket cmd{};
    cmd.cls = UBX_CLASS_RACEBOX;
    cmd.id  = UBX_ID_ERASE;
    cmd.len = 0;
    if (_ble.sendCommand(cmd)) {
        Serial.println("[Downloader] Erase command sent (0xFF/0x24)");
    } else {
        Serial.println("[Downloader] Failed to send erase command");
    }
}

// ── update() — timeout watchdog ───────────────────────────────────────────────
void RaceBoxDownloader::update() {
    if (_state == State::REQUESTED || _state == State::RECEIVING) {
        if ((millis() - _lastRxMs) > TIMEOUT_MS) {
            Serial.println("[Downloader] Timeout — no data received");
            _state = State::ERROR;
        }
    }
}

// ── _onPacket() ───────────────────────────────────────────────────────────────
void RaceBoxDownloader::_onPacket(const UbxPacket& pkt) {
    if (pkt.cls != UBX_CLASS_RACEBOX) return;
    _lastRxMs = millis();

    switch (pkt.id) {

    case UBX_ID_DOWNLOAD:
        // 0xFF/0x23 response: 4-byte expected record count (LE)
        if (_state == State::REQUESTED && pkt.len >= 4) {
            _expectedCount = static_cast<uint32_t>(pkt.payload[0])
                           | (static_cast<uint32_t>(pkt.payload[1]) << 8)
                           | (static_cast<uint32_t>(pkt.payload[2]) << 16)
                           | (static_cast<uint32_t>(pkt.payload[3]) << 24);
            _state = State::RECEIVING;
            Serial.printf("[Downloader] Expecting %lu records\n", (unsigned long)_expectedCount);
        }
        break;

    case UBX_ID_HISTORY:
        // 0xFF/0x21: identical 80-byte payload to live 0xFF/0x01
        if (_state == State::RECEIVING && _onRecord) {
            RaceBoxData d{};
            if (raceBoxParse(pkt, d)) {
                _onRecord(d, _recordCount);
                _recordCount++;
            }
        }
        break;

    case UBX_ID_STATE_CHANGE:
        // 0xFF/0x26: recording session boundary (start/stop/pause/resume)
        if (pkt.len >= 1 && _stateChangeCb) {
            _stateChangeCb(pkt.payload[0]);
        }
        break;

    case UBX_ID_ACK:
        if (_state == State::UNLOCKING &&
            pkt.len >= 2 &&
            pkt.payload[0] == UBX_CLASS_RACEBOX &&
            pkt.payload[1] == UBX_ID_MEM_UNLOCK) {
            // Unlock ACK — now send the download trigger
            Serial.println("[Downloader] Memory unlocked — sending download trigger");
            _sendDownloadTrigger();
        } else if (_state == State::RECEIVING &&
            pkt.len >= 2 &&
            pkt.payload[0] == UBX_CLASS_RACEBOX &&
            pkt.payload[1] == UBX_ID_DOWNLOAD) {
            // 0xFF/0x02 with payload [0xFF, 0x23] = download complete
            _state = State::DONE;
            Serial.printf("[Downloader] Done — %lu/%lu records received\n",
                          (unsigned long)_recordCount,
                          (unsigned long)_expectedCount);
        }
        break;

    case UBX_ID_NACK:
        if (_state == State::UNLOCKING &&
            pkt.len >= 2 &&
            pkt.payload[0] == UBX_CLASS_RACEBOX &&
            pkt.payload[1] == UBX_ID_MEM_UNLOCK) {
            Serial.println("[Downloader] NACK — memory unlock failed (wrong security code?)");
            _state = State::ERROR;
        } else if (_state == State::REQUESTED &&
            pkt.len >= 2 &&
            pkt.payload[0] == UBX_CLASS_RACEBOX &&
            pkt.payload[1] == UBX_ID_DOWNLOAD) {
            // Only treat as download failure if specifically for 0xFF/0x23.
            // Ignore NACKs for other commands (stale FIFO data).
            Serial.println("[Downloader] NACK — download refused by device");
            _state = State::ERROR;
        } else {
            Serial.printf("[Downloader] Ignored NACK (cls=0x%02X id=0x%02X, state=%d)\n",
                          pkt.len >= 1 ? pkt.payload[0] : 0,
                          pkt.len >= 2 ? pkt.payload[1] : 0,
                          (int)_state);
        }
        break;

    default:
        break;
    }
}
