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
    RECORDING_START  = 1,
    RECORDING_STOP   = 2,
    RECORDING_PAUSE  = 3,
    RECORDING_RESUME = 4,
};

// ── RecordingConfig ───────────────────────────────────────────────────────────

/**
 * Recording configuration — both the desired config (set via setConfig / startRecording)
 * and the config confirmed by the device (read back from 0xFF/0x26 STATE_CHANGE).
 *
 * The basic fields (rate, filters, shutdown) are used when sending commands.
 * The _confirmed_ fields are populated when the device echoes the config back
 * in a STATE_CHANGE start message, giving the actual values applied by the firmware.
 */
struct RecordingConfig {
    // ── Desired / sent config ────────────────────────────────────────────────
    DataRate rate              = DataRate::HZ_25;
    bool stationaryFilter      = false;  // suppress points when speed < threshold for interval
    bool noFixFilter           = false;  // suppress points when no GPS fix for interval
    uint8_t autoShutdownMin    = 0;      // 0=disabled; N=shutdown after N min of inactivity

    // ── Confirmed by device (from STATE_CHANGE read-back) ────────────────────
    // Zero until the first recording-start STATE_CHANGE is received.
    uint16_t stationarySpeedMmS  = 0;   // confirmed speed threshold (mm/s)
    uint16_t stationaryIntervalS = 0;   // confirmed stationary detection interval (s)
    uint16_t noFixIntervalS      = 0;   // confirmed no-fix detection interval (s)
    uint16_t autoShutdownSecs    = 0;   // confirmed auto-shutdown interval (s)
};

// ── Callbacks ─────────────────────────────────────────────────────────────────

// Called when a 0xFF/0x26 State Change packet arrives.
using StateChangeCallback = std::function<void(StateChangeEvent event)>;

// Called on each 0xFF/0x24 erase-progress notification (0–100 %).
// Also called with 100 when the final ACK arrives (erase complete).
using EraseProgressCallback = std::function<void(uint8_t percent)>;

// Called when a recording-start STATE_CHANGE confirms the applied config.
// Use confirmedConfig() to read the full confirmed values.
using ConfigConfirmedCallback = std::function<void(const RecordingConfig& confirmed)>;

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
 *   rec.setSecurityCode(123456);
 *   rec.begin();
 *
 *   // Option A — explicit config each call (backward compatible)
 *   rec.startRecording(DataRate::HZ_25, true, true, 5);
 *
 *   // Option B — set once, reuse
 *   rec.setConfig(DataRate::HZ_25, true, true, 5);
 *   rec.startRecording();   // uses stored config
 *   ...
 *   rec.stopRecording();
 *   rec.startRecording();   // same config, no need to repeat
 */
class RaceBoxRecorder {
public:
    explicit RaceBoxRecorder(RaceBoxBle& ble);

    // Register packet callback on `ble`. Call once after ble.begin().
    void begin();

    // Drive internal state (ACK timeout). Call every loop iteration.
    void update();

    // ── Security code ─────────────────────────────────────────────────────────

    void setSecurityCode(uint32_t code) { _securityCode = code; }

    // ── Recording config ──────────────────────────────────────────────────────

    // Pre-configure without starting. Subsequent startRecording() calls
    // (no-args form) will use this config.
    void setConfig(DataRate rate,
                   bool stationaryFilter = false,
                   bool noFixFilter      = false,
                   uint8_t autoShutdownMin = 0);

    // Current stored config (updated by setConfig() and startRecording(rate,...)).
    const RecordingConfig& config() const { return _config; }

    // Config confirmed by the device after the last recording-start STATE_CHANGE.
    // Zero until at least one recording session has started.
    const RecordingConfig& confirmedConfig() const { return _confirmedConfig; }

    // ── Commands (async — result via lastAck() / state()) ────────────────────

    // Send 0xFF/0x22 to query recording status and record count.
    void queryStatus();

    // Start recording using the stored config (set via setConfig() or a previous
    // startRecording(rate,...) call). Defaults to HZ_25/no-filters if never configured.
    void startRecording();

    // Start recording with an explicit config (also updates the stored config).
    void startRecording(DataRate rate,
                        bool stationaryFilter = false,
                        bool noFixFilter      = false,
                        uint8_t autoShutdownMin = 0);

    // Stop recording.
    void stopRecording();

    // Start a full memory erase (preceded by auto-unlock).
    // Progress reported via setEraseProgressCallback(). Can take several minutes.
    void eraseMemory();

    // Cancel an ongoing erase. No-op if not erasing.
    void cancelErase();

    // ── State ────────────────────────────────────────────────────────────────
    RecordingState state()         const { return _state; }
    uint8_t        memoryLevel()   const { return _memoryLevel; }
    uint32_t       recordCount()   const { return _recordCount; }
    uint32_t       memorySize()    const { return _memorySize; }
    DataRate       dataRate()      const { return _dataRate; }
    bool           lastAck()       const { return _lastAck; }
    bool           isErasing()     const { return _isErasing; }
    uint8_t        eraseProgress() const { return _eraseProgress; }

    // Returns true while an unlock→command round trip is in flight (not yet ACKed).
    // Use this to wait for a pending stop/start/erase before handing the BLE
    // packet callback to another driver (e.g. RaceBoxDownloader).
    bool           isPending()     const { return _pendingCmd != _Pending::NONE || _cmdSentMs != 0; }

    // ── Callbacks ─────────────────────────────────────────────────────────────

    // Called on every 0xFF/0x26 STATE_CHANGE (start/stop/pause/resume).
    void setStateChangeCallback(StateChangeCallback cb) { _stateChangeCb = std::move(cb); }

    // Called when recording starts and the device confirms the applied config.
    // Fires after the STATE_CHANGE is parsed — confirmedConfig() is already updated.
    void setConfigConfirmedCallback(ConfigConfirmedCallback cb) { _configConfirmedCb = std::move(cb); }

    // Called on each erase progress notification (0–100 %) and on completion.
    void setEraseProgressCallback(EraseProgressCallback cb) { _eraseProgressCb = std::move(cb); }

    // Called by RaceBoxBle packet callback — do not call directly.
    void _onPacket(const UbxPacket& pkt);

private:
    enum class _Pending : uint8_t { NONE, START, STOP, ERASE };

    RaceBoxBle&         _ble;
    RecordingState      _state       = RecordingState::UNKNOWN;
    uint8_t             _memoryLevel = 0;
    uint32_t            _recordCount = 0;
    uint32_t            _memorySize  = 0;
    DataRate            _dataRate    = DataRate::HZ_25;
    bool                _lastAck     = false;
    uint32_t            _cmdSentMs   = 0;

    RecordingConfig          _config;            // desired / stored config
    RecordingConfig          _confirmedConfig;   // last config confirmed by device
    StateChangeCallback      _stateChangeCb;
    ConfigConfirmedCallback  _configConfirmedCb;
    EraseProgressCallback    _eraseProgressCb;

    bool     _isErasing     = false;
    uint8_t  _eraseProgress = 0;

    uint32_t _securityCode = 123456;
    _Pending _pendingCmd   = _Pending::NONE;

    bool _sendUnlock();
    void _sendConfig(uint8_t command);
    void _sendErase();
};
