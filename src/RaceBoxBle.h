#pragma once

#include <stdint.h>
#include <stddef.h>
#include <functional>
#include "ubx.h"
#include "RaceBoxData.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"

// ── BLE UUIDs (Nordic UART Service, used by RaceBox devices) ─────────────────
static const char* RACEBOX_SERVICE_UUID = "6E400001-B5A3-F393-E0A9-E50E24DCCA9E";
static const char* RACEBOX_RX_UUID      = "6E400002-B5A3-F393-E0A9-E50E24DCCA9E";  // client → device
static const char* RACEBOX_TX_UUID      = "6E400003-B5A3-F393-E0A9-E50E24DCCA9E";  // device → client

// FIFO size — enough for ~5 complete UBX frames (88 bytes each)
static constexpr size_t RACEBOX_FIFO_SIZE = 512;

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
 * A FIFO ring-buffer accumulates raw bytes between calls to update() and
 * extracts complete UBX frames one at a time.
 *
 * Usage:
 *   RaceBoxBle ble(onLiveData);
 *   ble.setPacketCallback(onPacket);  // optional
 *   ble.begin();
 *   loop: ble.update();
 */
class RaceBoxBle {
public:
    explicit RaceBoxBle(RaceBoxLiveCallback onLiveData = nullptr);

    // Call once in setup(). Initialises NimBLE and starts scan.
    void begin();

    // Call every loop iteration — drives reconnect logic and flushes the FIFO.
    void update();

    bool isConnected() const { return _connected; }

    // Set callback for non-live packets (Recorder / Downloader use this).
    void setPacketCallback(RaceBoxPacketCallback cb) { _packetCb = std::move(cb); }

    // Optional callback invoked when the FIFO overflows (oldest bytes dropped).
    // The argument is the cumulative dropped-byte count since begin().
    // Default: no callback (silent overflow — same behaviour as before).
    //
    // ⚠ Callback context: invoked from the NimBLE task, not from update().
    // Keep it short (e.g. set a flag). Do NOT call sendCommand() or any
    // other RaceBoxBle method from within this callback.
    void setOverflowCallback(std::function<void(uint32_t)> cb) { _overflowCb = std::move(cb); }

    // Total number of bytes dropped due to FIFO overflow since begin().
    uint32_t fifoOverflowCount() const { return _fifoOverflowCount; }

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
    std::function<void(uint32_t)> _overflowCb;
    uint32_t _fifoOverflowCount = 0;

    bool _connected = false;
    bool _doConnect = false;
    bool _doScan    = false;

    // ── FIFO ring buffer ──────────────────────────────────────────────────────
    // _fifoMutex protects all FIFO members. _fifoPush() is called from the
    // NimBLE task; _drainFifo() is called from the Arduino main loop via
    // update(). Both acquire the mutex before touching the FIFO.
    SemaphoreHandle_t _fifoMutex = nullptr;

    uint8_t _fifo[RACEBOX_FIFO_SIZE];
    size_t  _fifoHead = 0;  // write index
    size_t  _fifoTail = 0;  // read index
    size_t  _fifoLen  = 0;

    void   _fifoPush(const uint8_t* data, size_t len);
    bool   _fifoPeek(uint8_t* out, size_t len) const;
    void   _fifoDrop(size_t len);
    size_t _fifoAvail() const { return _fifoLen; }

    // Drain the FIFO: extract and dispatch complete UBX frames.
    void _drainFifo();

    // NimBLE RX characteristic handle (kept for sendCommand)
    void* _rxChar = nullptr;  // NimBLERemoteCharacteristic*
};
