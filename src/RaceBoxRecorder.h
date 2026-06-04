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

// Callback fired on each 0xFF/0x24 erase-progress notification (0–100 %).
// Also called with 100 when the final ACK arrives (erase complete).
using EraseProgressCallback = std::function<void(uint8_t percent)>;

/**
 * Controls standalone recording on a RaceBox Mini S / Micro.
 *
 * Attach to a RaceBoxBle instance via begin(); the recorder registers itself
 * as the packet callback and handles 0xFF/0x22 (status response) and
 * 0xFF/0x26 (state change) messages.
 *
 * Memory unlock:
 *   The RaceBox requires a security code unlock (0xFF/0x30) before accepting
 *   any recording command. Call setSecurityCode() once with your device's code
 *   and the Recorder will handle unlock transparently — including after every
 *   reconnection. Default code is 123456 (RaceBox factory default).
 *
 * Typical usage:
 *   RaceBoxRecorder rec(ble);
 *   rec.setSecurityCode(123456);   // once — handled automatically after this
 *   rec.begin();
 *   rec.queryStatus();             // async — result available after update()
 *   rec.startRecording(DataRate::HZ_25);   // unlock sent automatically
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

    // ── Security code ─────────────────────────────────────────────────────────

    // Set the memory security code (default: 123456).
    // Once set, startRecording() and stopRecording() will automatically send
    // the unlock command before the actual recording command.
    // The lock resets on every BLE reconnection — the Recorder handles this.
    void setSecurityCode(uint32_t code) { _securityCode = code; }

    // ── Commands (async — result via lastAck() / state()) ────────────────────

    // Send 0xFF/0x22 to query recording status and record count.
    // Results available after the device replies (check state/recordCount).
    void queryStatus();

    // Send 0xFF/0x25 to start recording (preceded by auto-unlock if code is set).
    // stationaryFilter: suppress points when stationary (speed < 1389 mm/s for 30s)
    // noFixFilter:      suppress points when GPS fix lost for 30s
    // autoShutdownMin:  0 = disabled; otherwise shut down after N minutes of inactivity
    void startRecording(DataRate rate = DataRate::HZ_25,
                        bool stationaryFilter = false,
                        bool noFixFilter = false,
                        uint8_t autoShutdownMin = 0);

    // Send 0xFF/0x25 with stop command (preceded by auto-unlock if code is set).
    void stopRecording();

    // Start a full memory erase (preceded by auto-unlock).
    // Progress reported via setEraseProgressCallback(). Can take several minutes.
    // Refused if BLE is not connected or an erase is already in progress.
    void eraseMemory();

    // Cancel an ongoing erase (sends 0xFF/0x24 with 1 byte).
    // No-op if not erasing.
    void cancelErase();

    // ── State (updated asynchronously on receipt of device responses) ─────────
    // state():       current recording state (from 0xFF/0x22 STATUS or 0xFF/0x26 STATE CHANGE)
    // memoryLevel(): memory usage in percent 0..100 (from 0xFF/0x22 STATUS)
    // recordCount(): number of stored records (from 0xFF/0x22 STATUS)
    // dataRate():    last known recording data rate (from 0xFF/0x26 STATE CHANGE)
    // lastAck():     true=last command ACKed, false=NACKed or timeout
    RecordingState state()       const { return _state; }
    uint8_t        memoryLevel() const { return _memoryLevel; }
    uint32_t       recordCount() const { return _recordCount; }
    uint32_t       memorySize()  const { return _memorySize; }
    DataRate       dataRate()    const { return _dataRate; }
    bool           lastAck()     const { return _lastAck; }  // true=ACK, false=NACK
    bool           isErasing()   const { return _isErasing; }
    uint8_t        eraseProgress() const { return _eraseProgress; }

    // Optional: called when a 0xFF/0x26 State Change arrives (during download)
    void setStateChangeCallback(StateChangeCallback cb) { _stateChangeCb = std::move(cb); }

    // Optional: called on each erase progress notification (0–100 %) and on completion.
    void setEraseProgressCallback(EraseProgressCallback cb) { _eraseProgressCb = std::move(cb); }

    // Called by RaceBoxBle packet callback — do not call directly.
    void _onPacket(const UbxPacket& pkt);

private:
    // Pending command type — queued while waiting for unlock ACK
    enum class _Pending : uint8_t { NONE, START, STOP, ERASE };

    RaceBoxBle&         _ble;
    RecordingState      _state       = RecordingState::UNKNOWN;
    uint8_t             _memoryLevel = 0;     // memory usage % (from STATUS)
    uint32_t            _recordCount = 0;     // stored records (from STATUS)
    uint32_t            _memorySize  = 0;     // total capacity in records (from STATUS)
    DataRate            _dataRate    = DataRate::HZ_25;  // from STATE CHANGE
    bool                _lastAck     = false;
    uint32_t            _cmdSentMs   = 0;
    StateChangeCallback   _stateChangeCb;
    EraseProgressCallback _eraseProgressCb;
    bool     _isErasing     = false;
    uint8_t  _eraseProgress = 0;

    uint32_t            _securityCode  = 123456;
    _Pending            _pendingCmd    = _Pending::NONE;

    // Pending start args (saved while waiting for unlock ACK)
    DataRate  _pendingRate            = DataRate::HZ_25;
    bool      _pendingStationary      = false;
    bool      _pendingNoFix           = false;
    uint8_t   _pendingAutoShutdown    = 0;

    bool _sendUnlock();
    void _sendConfig(uint8_t command, DataRate rate,
                     bool stationaryFilter, bool noFixFilter,
                     uint8_t autoShutdownMin);
    void _sendErase();
};
