#include "RaceBoxBle.h"
#include "RaceBoxParser.h"
#include <Arduino.h>
#include <NimBLEDevice.h>
#include <string.h>
#include <stdio.h>
#include <stdarg.h>

// ── Static instance ───────────────────────────────────────────────────────────
RaceBoxBle* RaceBoxBle::_instance = nullptr;

// ── NimBLE callback adapters ──────────────────────────────────────────────────
class RbClientCallbacks : public NimBLEClientCallbacks {
    void onConnect(NimBLEClient*)        override { RaceBoxBle::_instance->_handleConnect(); }
    void onDisconnect(NimBLEClient*, int) override { RaceBoxBle::_instance->_handleDisconnect(); }
};

class RbScanCallbacks : public NimBLEScanCallbacks {
    void onResult(const NimBLEAdvertisedDevice* dev) override {
        RaceBoxBle::_instance->_handleScanResult(
            dev->getName().c_str(),
            dev->getAddress().toString().c_str(),
            dev->getAddress().getType());
        if (dev->isAdvertisingService(NimBLEUUID(RACEBOX_SERVICE_UUID)))
            NimBLEDevice::getScan()->stop();
    }
};

static RbClientCallbacks clientCb;
static RbScanCallbacks   scanCb;
static NimBLEAddress     targetAddr;
static bool              targetFound = false;

// ── Constructor ───────────────────────────────────────────────────────────────
RaceBoxBle::RaceBoxBle(RaceBoxLiveCallback onLiveData)
    : _liveCb(std::move(onLiveData)) {
    _instance = this;
}

// ── _debug() ──────────────────────────────────────────────────────────────────
void RaceBoxBle::_debug(const char* fmt, ...) {
    if (!_debugCb) return;
    char buf[128];
    va_list args;
    va_start(args, fmt);
    vsnprintf(buf, sizeof(buf), fmt, args);
    va_end(args);
    _debugCb(buf);
}

// ── begin() ───────────────────────────────────────────────────────────────────
void RaceBoxBle::begin() {
    NimBLEDevice::init("");
    NimBLEDevice::setPower(ESP_PWR_LVL_P9);
    NimBLEScan* scan = NimBLEDevice::getScan();
    scan->setScanCallbacks(&scanCb, false);
    scan->setActiveScan(true);
    scan->setInterval(100);
    scan->setWindow(99);
    scan->start(0, false);
    _debug("[BLE] Scanning for RaceBox...");
}

// ── update() ─────────────────────────────────────────────────────────────────
void RaceBoxBle::update() {
    if (_doConnect && targetFound) {
        _doConnect = false;

        // Delete any lingering NimBLE client from the previous session.
        // createClient() adds a new entry; without this the old disconnected
        // client stays in NimBLE's list and can cause GATT handle confusion.
        NimBLEClient* stale = NimBLEDevice::getClientByPeerAddress(targetAddr);
        if (stale) NimBLEDevice::deleteClient(stale);

        NimBLEClient* client = NimBLEDevice::createClient();
        client->setClientCallbacks(&clientCb, false);

        if (!client->connect(targetAddr)) {
            _debug("[BLE] Connection failed, re-scanning...");
            NimBLEDevice::deleteClient(client);
            _doScan = true;
            return;
        }

        NimBLERemoteService* svc = client->getService(RACEBOX_SERVICE_UUID);
        if (!svc) {
            _debug("[BLE] RaceBox service not found");
            client->disconnect();
            _doScan = true;
            return;
        }

        // Subscribe to TX (device → client) notifications
        NimBLERemoteCharacteristic* tx = svc->getCharacteristic(RACEBOX_TX_UUID);
        if (!tx || !tx->canNotify()) {
            _debug("[BLE] TX characteristic not found");
            client->disconnect();
            _doScan = true;
            return;
        }
        tx->subscribe(true, [](NimBLERemoteCharacteristic*, uint8_t* d, size_t l, bool) {
            if (_instance) _instance->_handleNotification(d, l);
        });

        // Cache RX characteristic for sendCommand()
        NimBLERemoteCharacteristic* rx = svc->getCharacteristic(RACEBOX_RX_UUID);
        _rxChar = (rx && rx->canWrite()) ? static_cast<void*>(rx) : nullptr;

        _connected = true;
        char addr[32];
        snprintf(addr, sizeof(addr), "%s", targetAddr.toString().c_str());
        _debug("[BLE] Connected to %s", addr);
    }

    if (_doScan) {
        _doScan    = false;
        targetFound = false;
        NimBLEDevice::getScan()->start(0, false);
        _debug("[BLE] Re-scanning...");
    }

    // Drain any bytes accumulated since last update()
    _drainFifo();
}

// ── disconnect() ─────────────────────────────────────────────────────────────
void RaceBoxBle::disconnect() {
    if (!_connected) return;
    NimBLEClient* client = NimBLEDevice::getClientByPeerAddress(targetAddr);
    if (client) client->disconnect();
    // _handleDisconnect() will fire asynchronously, set _connected=false and _doScan=true
}

// ── sendCommand() ─────────────────────────────────────────────────────────────
bool RaceBoxBle::sendCommand(const UbxPacket& pkt) {
    if (!_connected || !_rxChar) return false;
    uint8_t buf[UBX_FRAME_OVERHEAD + UBX_MAX_PAYLOAD];
    size_t  len = ubxEncode(pkt, buf, sizeof(buf));
    if (len == 0) return false;
    auto* rx = static_cast<NimBLERemoteCharacteristic*>(_rxChar);
    return rx->writeValue(buf, len, false);
}

// ── NimBLE bridge handlers ────────────────────────────────────────────────────
void RaceBoxBle::_handleConnect() {}

void RaceBoxBle::_handleDisconnect() {
    _connected = false;
    _rxChar    = nullptr;
    _doScan    = true;
    // Discard any stale bytes from the previous session so they are not
    // mistakenly parsed as packets on the next connection.
    _fifo.clear();
    _debug("[BLE] Disconnected, re-scanning...");
}

void RaceBoxBle::_handleScanResult(const char* name, const char* address, uint8_t addrType) {
    if (String(name).startsWith("RaceBox")) {
        _debug("[BLE] Found: %s", name);
        targetAddr  = NimBLEAddress(std::string(address), addrType);
        targetFound = true;
        _doConnect  = true;
    }
}

void RaceBoxBle::_handleNotification(const uint8_t* data, size_t len) {
    // Only push — do NOT drain here.
    // _handleNotification() runs on the NimBLE FreeRTOS task; _drainFifo()
    // runs on the Arduino main loop via update(). The FIFO's internal mutex
    // ensures the push is safe across tasks.
    _fifo.push(data, len);
}

// ── _drainFifo() — extract and dispatch complete UBX frames ──────────────────
//
// Calls RaceBoxFifo::extractFrame() which acquires/releases the FIFO mutex
// internally. Callbacks are invoked outside the FIFO mutex so they can safely
// call sendCommand() without risk of deadlock.
void RaceBoxBle::_drainFifo() {
    uint8_t frame[UBX_FRAME_OVERHEAD + UBX_MAX_PAYLOAD];
    size_t  frameLen = 0;

    while (_fifo.extractFrame(frame, frameLen)) {
        UbxPacket pkt{};
        if (!ubxDecode(frame, frameLen, pkt)) {
            _decodeErrorCount++;
            if (_errorCb) _errorCb(_decodeErrorCount);
            continue;
        }

        if (_sniffCb) _sniffCb(pkt);  // sniffer fires first, for all packets

        if (pkt.cls == UBX_CLASS_RACEBOX && pkt.id == UBX_ID_LIVE) {
            if (_liveCb) {
                RaceBoxData d{};
                if (raceBoxParse(pkt, d)) _liveCb(d);
            }
        } else {
            if (_packetCb) _packetCb(pkt);
        }
    }
}
