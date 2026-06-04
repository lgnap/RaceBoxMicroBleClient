#pragma once

#include <stdint.h>
#include <stddef.h>
#include <functional>
#include "ubx.h"
#include "RaceBoxData.h"
#include "RaceBoxFifo.h"

// ── BLE UUIDs (Nordic UART Service, used by RaceBox devices) ─────────────────
static const char* RACEBOX_SERVICE_UUID = "6E400001-B5A3-F393-E0A9-E50E24DCCA9E";
static const char* RACEBOX_RX_UUID      = "6E400002-B5A3-F393-E0A9-E50E24DCCA9E";  // client → device
static const char* RACEBOX_TX_UUID      = "6E400003-B5A3-F393-E0A9-E50E24DCCA9E";  // device → client

// ── Callbacks ─────────────────────────────────────────────────────────────────
// Called with each parsed live-data record (0xFF/0x01).
using RaceBoxLiveCallback = std::function<void(const RaceBoxData&)>;

// Called with any non-live UBX packet (0xFF/0x21, 0xFF/0x22, 0xFF/0x23, etc.).
// Used by RaceBoxDownloader and RaceBoxRecorder.
using RaceBoxPacketCallback = std::function<void(const UbxPacket&)>;

/**
 * BLE client for RaceBox Mini S / Micro.
 *
 * Scans for a RaceBox device, connects, subscribes to TX notifications, and
 * dispatches parsed packets:
 *   - Live data (0xFF/0x01) → RaceBoxLiveCallback
 *   - All other packets     → RaceBoxPacketCallback (for Downloader / Recorder)
 *
 * Multiple records may arrive in a single BLE notification (MTU ~244 bytes).
 * A RaceBoxFifo ring-buffer accumulates raw bytes between calls to update()
 * and extracts complete UBX frames one at a time.
 *
 * Usage:
 *   RaceBoxBle ble(onLiveData);
 *   ble.setDebugCallback([](const char* msg){ Serial.println(msg); }); // optional
 *   ble.setPacketCallback(onPacket);  // optional
 *   ble.begin();
 *   loop: ble.update();
 *
 * ── RaceBoxRecorder / RaceBoxDownloader coexistence ───────────────────────────
 * Both Recorder and Downloader register themselves via setPacketCallback().
 * Only ONE can be active at a time — calling begin() on the second silently
 * replaces the first callback. To switch between them at runtime, call begin()
 * again on the module you want to activate. See LibTest for a reference
 * implementation that toggles between recorder and downloader mode.
 */
class RaceBoxBle {
public:
    explicit RaceBoxBle(RaceBoxLiveCallback onLiveData = nullptr);

    // Call once in setup(). Initialises NimBLE and starts scan.
    void begin();

    // Call every loop iteration — drives reconnect logic and flushes the FIFO.
    void update();

    bool isConnected() const { return _connected; }

    // ── Packet routing ────────────────────────────────────────────────────────

    // Set callback for non-live packets (Recorder / Downloader use this).
    //
    // ⚠ Only one callback can be active at a time. Calling this a second time
    // (e.g. switching from Recorder to Downloader) replaces the previous
    // callback. A debug warning is emitted if a debug callback is set.
    void setPacketCallback(RaceBoxPacketCallback cb) {
        if (_packetCb && cb && _debugCb)
            _debugCb("[BLE] WARNING: overwriting existing packet callback — "
                     "only one of RaceBoxRecorder/RaceBoxDownloader can be active at a time");
        _packetCb = std::move(cb);
    }

    // Optional sniffer: fires for EVERY parsed UBX packet (live + non-live),
    // before the normal callbacks. Useful for protocol debugging.
    void setSniffCallback(RaceBoxPacketCallback cb) { _sniffCb = std::move(cb); }

    // ── Debug / diagnostics ───────────────────────────────────────────────────

    // Optional debug log callback. Receives a null-terminated string for each
    // internal event (connect, disconnect, scan, errors). Default: silent.
    // Replaces all Serial.println() calls — wire to Serial if you want output:
    //   ble.setDebugCallback([](const char* msg){ Serial.println(msg); });
    void setDebugCallback(std::function<void(const char*)> cb) { _debugCb = std::move(cb); }

    // Optional: called when the FIFO overflows (oldest bytes dropped).
    // ⚠ Invoked from the NimBLE task — keep it short and don't call
    // sendCommand() or other RaceBoxBle methods from within this callback.
    void setOverflowCallback(std::function<void(uint32_t)> cb) {
        _fifo.setOverflowCallback(std::move(cb));
    }

    // Total bytes dropped due to FIFO overflow since begin().
    uint32_t fifoOverflowCount() const { return _fifo.overflowCount(); }

    // Optional: called when a UBX frame fails checksum / framing validation.
    // Argument is the cumulative error count since begin().
    void setErrorCallback(std::function<void(uint32_t)> cb) { _errorCb = std::move(cb); }

    // Cumulative UBX decode errors since begin() (bad checksum, truncated frames).
    uint32_t decodeErrorCount() const { return _decodeErrorCount; }

    // Encode `pkt` as a UBX frame and write it to the RX characteristic.
    // Returns false if not connected or the characteristic is unavailable.
    bool sendCommand(const UbxPacket& pkt);

    // ── NimBLE bridge (public so static callbacks can call them) ─────────────
    static RaceBoxBle* _instance;
    void _handleNotification(const uint8_t* data, size_t len);
    void _handleConnect();
    void _handleDisconnect();
    void _handleScanResult(const char* name, const char* address, uint8_t addrType);

private:
    RaceBoxLiveCallback   _liveCb;
    RaceBoxPacketCallback _packetCb;
    RaceBoxPacketCallback _sniffCb;
    std::function<void(const char*)> _debugCb;
    std::function<void(uint32_t)>    _errorCb;
    uint32_t _decodeErrorCount = 0;

    bool _connected = false;
    bool _doConnect = false;
    bool _doScan    = false;

    RaceBoxFifo _fifo;

    // NimBLE RX characteristic handle (kept for sendCommand)
    void* _rxChar = nullptr;  // NimBLERemoteCharacteristic*

    // Drain the FIFO: extract and dispatch complete UBX frames.
    void _drainFifo();

    // Internal debug helper — formats and forwards to _debugCb if set.
    void _debug(const char* fmt, ...);
};
