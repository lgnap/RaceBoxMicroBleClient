#pragma once

#include <stdint.h>
#include <functional>
#include "ubx.h"

// Forward declaration to avoid circular includes
class RaceBoxBle;

// ── Enums ─────────────────────────────────────────────────────────────────────

enum class RecordingState : uint8_t {
    IDLE      = 0,
    RECORDING = 1,
    PAUSED    = 2,
    UNKNOWN   = 0xFF,
};

// Recording data rates supported by the RaceBox firmware.
// Note: HZ_1 is defined in the protocol but rejected by current RaceBox Micro
// firmware (device replies with NACK). Use HZ_5, HZ_10 or HZ_25.
enum class DataRate : uint8_t {
    HZ_1  = 1,   // ⚠ NACK on RaceBox Micro (not supported by device firmware)
    HZ_5  = 5,
    HZ_10 = 10,
    HZ_25 = 25,
};

// State change events (0xFF/0x26, byte 0)
enum class StateChangeEvent : uint8_t {
    RECORDING_START = 1,
    RECORDING_STOP  = 2,
    RECORDING_PAUSE = 3,
    RECORDING_RESUME = 4,
};

// Callback fired when a 0xFF/0x26 State Change packet arrives (during download).
using StateChangeCallback = std::function<void(StateChangeEvent event)>;

/**
 * Controls standalone recording on a RaceBox Mini S / Micro.
 *
 * Attach to a RaceBoxBle instance via begin(); the recorder registers itself
 * as the packet callback and handles 0xFF/0x22 (status response) and
 * 0xFF/0x26 (state change) messages.
 *
 * Typical usage:
 *   RaceBoxRecorder rec(ble);
 *   rec.begin();
 *   rec.queryStatus();           // async — result available after update()
 *   rec.startRecording(DataRate::HZ_25);
 *   ...
 *   rec.stopRecording();
 */
class RaceBoxRecorder {
public:
    explicit RaceBoxRecorder(RaceBoxBle& ble);

    // Register packet callback on `ble`. Call once after ble.begin().
    void begin();

    // Drive internal state (ACK timeout). Call every loop iteration.
    void update();

    // ── Commands (async — result via callbacks / state flags) ────────────────

    // Send 0xFF/0x22 to query recording status and record count.
    // Results available after the device replies (check state/recordCount).
    void queryStatus();

    // Send 0xFF/0x25 to start/configure recording.
    // stationaryFilter: suppress points when device is stationary
    // noFixFilter:      suppress points when there is no GPS fix
    // autoShutdownMin:  0 = disabled; otherwise shut down after N minutes of inactivity
    void startRecording(DataRate rate = DataRate::HZ_25,
                        bool stationaryFilter = false,
                        bool noFixFilter = false,
                        uint8_t autoShutdownMin = 0);

    // Send 0xFF/0x25 with stop command.
    void stopRecording();

    // ── State (updated asynchronously on receipt of device responses) ─────────
    RecordingState state()       const { return _state; }
    DataRate       dataRate()    const { return _dataRate; }
    uint32_t       recordCount() const { return _recordCount; }
    bool           lastAck()     const { return _lastAck; }  // true=ACK, false=NACK

    // Optional: called when a 0xFF/0x26 State Change arrives (during download)
    void setStateChangeCallback(StateChangeCallback cb) { _stateChangeCb = std::move(cb); }

    // Called by RaceBoxBle packet callback — do not call directly.
    void _onPacket(const UbxPacket& pkt);

private:
    RaceBoxBle&         _ble;
    RecordingState      _state       = RecordingState::UNKNOWN;
    DataRate            _dataRate    = DataRate::HZ_25;
    uint32_t            _recordCount = 0;
    bool                _lastAck     = false;
    uint32_t            _cmdSentMs   = 0;  // millis() when last command was sent
    StateChangeCallback _stateChangeCb;

    void _sendConfig(uint8_t command, DataRate rate,
                     bool stationaryFilter, bool noFixFilter,
                     uint8_t autoShutdownMin);
};
