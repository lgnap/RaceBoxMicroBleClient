#pragma once

#include <stdint.h>
#include <functional>
#include "ubx.h"
#include "RaceBoxData.h"

class RaceBoxBle;

// ── Callbacks ─────────────────────────────────────────────────────────────────

// Called for each downloaded history record. `index` is 0-based.
using HistoryCallback = std::function<void(const RaceBoxData& data, uint32_t index)>;

// Called when a 0xFF/0x26 State Change arrives during download.
using DownloadStateChangeCallback = std::function<void(uint8_t event)>;

/**
 * Downloads stored history from a RaceBox Mini S / Micro.
 *
 * Flow:
 *   1. begin()      — sends 0xFF/0x23 (empty) to trigger download
 *   2. Device replies with 0xFF/0x23 carrying a 4-byte expected record count
 *   3. Device streams 0xFF/0x21 records (identical payload to live 0xFF/0x01)
 *      interspersed with 0xFF/0x26 State Change messages marking session boundaries
 *   4. Device sends ACK (0xFF/0x02 with payload [0xFF, 0x23]) when done
 *
 * Usage:
 *   RaceBoxDownloader dl(ble, onRecord);
 *   dl.setStateChangeCallback(onStateChange);  // optional
 *   dl.begin();
 *   loop: ble.update(); dl.update();
 *   when dl.isDone() → all records received
 */
class RaceBoxDownloader {
public:
    RaceBoxDownloader(RaceBoxBle& ble, HistoryCallback onRecord);

    // Register as packet callback on `ble` and send the download trigger.
    // Call once after ble.isConnected() is true.
    void begin();

    // Drive timeout watchdog. Call every loop iteration.
    void update();

    // Optional: notified of recording session boundaries (start/stop/pause/resume)
    void setStateChangeCallback(DownloadStateChangeCallback cb) { _stateChangeCb = std::move(cb); }

    bool     isDone()          const { return _state == State::DONE; }
    bool     isError()         const { return _state == State::ERROR; }
    uint32_t recordCount()     const { return _recordCount; }
    uint32_t expectedCount()   const { return _expectedCount; }
    uint8_t  progressPercent() const {
        if (_expectedCount == 0) return 0;
        uint32_t pct = (_recordCount * 100UL) / _expectedCount;
        return pct > 100 ? 100 : static_cast<uint8_t>(pct);
    }

    // Called by packet callback — do not call directly.
    void _onPacket(const UbxPacket& pkt);

private:
    enum class State : uint8_t { IDLE, REQUESTED, RECEIVING, DONE, ERROR };

    RaceBoxBle&               _ble;
    HistoryCallback           _onRecord;
    DownloadStateChangeCallback _stateChangeCb;

    State    _state         = State::IDLE;
    uint32_t _expectedCount = 0;
    uint32_t _recordCount   = 0;
    uint32_t _lastRxMs      = 0;  // millis() of last received packet

    static constexpr uint32_t TIMEOUT_MS = 30000;  // 30s inactivity timeout
};
