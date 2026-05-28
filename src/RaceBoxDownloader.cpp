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

    // Send download trigger: 0xFF/0x23 with empty payload
    UbxPacket trigger{};
    trigger.cls = UBX_CLASS_RACEBOX;
    trigger.id  = UBX_ID_DOWNLOAD;
    trigger.len = 0;

    if (_ble.sendCommand(trigger)) {
        _state    = State::REQUESTED;
        _lastRxMs = millis();
        Serial.println("[Downloader] Download trigger sent");
    } else {
        _state = State::ERROR;
        Serial.println("[Downloader] Failed to send trigger (not connected?)");
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
        // 0xFF/0x02 with payload [0xFF, 0x23] = download complete
        if (_state == State::RECEIVING &&
            pkt.len >= 2 &&
            pkt.payload[0] == UBX_CLASS_RACEBOX &&
            pkt.payload[1] == UBX_ID_DOWNLOAD) {
            _state = State::DONE;
            Serial.printf("[Downloader] Done — %lu/%lu records received\n",
                          (unsigned long)_recordCount,
                          (unsigned long)_expectedCount);
        }
        break;

    case UBX_ID_NACK:
        Serial.println("[Downloader] NACK — download failed");
        _state = State::ERROR;
        break;

    default:
        break;
    }
}
