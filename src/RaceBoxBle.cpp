#include "RaceBoxBle.h"
#include "RaceBoxParser.h"
#include <Arduino.h>
#include <NimBLEDevice.h>
#include <string.h>

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
    memset(_fifo, 0, sizeof(_fifo));
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
    Serial.println("[BLE] Scanning for RaceBox...");
}

// ── update() ─────────────────────────────────────────────────────────────────
void RaceBoxBle::update() {
    if (_doConnect && targetFound) {
        _doConnect = false;

        NimBLEClient* client = NimBLEDevice::createClient();
        client->setClientCallbacks(&clientCb, false);

        if (!client->connect(targetAddr)) {
            Serial.println("[BLE] Connection failed, re-scanning...");
            NimBLEDevice::deleteClient(client);
            _doScan = true;
            return;
        }

        NimBLERemoteService* svc = client->getService(RACEBOX_SERVICE_UUID);
        if (!svc) {
            Serial.println("[BLE] RaceBox service not found");
            client->disconnect();
            _doScan = true;
            return;
        }

        // Subscribe to TX (device → client) notifications
        NimBLERemoteCharacteristic* tx = svc->getCharacteristic(RACEBOX_TX_UUID);
        if (!tx || !tx->canNotify()) {
            Serial.println("[BLE] TX characteristic not found");
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
        Serial.printf("[BLE] Connected to %s\n", targetAddr.toString().c_str());
    }

    if (_doScan) {
        _doScan    = false;
        targetFound = false;
        NimBLEDevice::getScan()->start(0, false);
        Serial.println("[BLE] Re-scanning...");
    }

    // Drain any bytes accumulated since last update()
    _drainFifo();
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
    Serial.println("[BLE] Disconnected, re-scanning...");
}

void RaceBoxBle::_handleScanResult(const char* name, const char* address, uint8_t addrType) {
    if (String(name).startsWith("RaceBox")) {
        Serial.printf("[BLE] Found: %s\n", name);
        targetAddr  = NimBLEAddress(std::string(address), addrType);
        targetFound = true;
        _doConnect  = true;
    }
}

void RaceBoxBle::_handleNotification(const uint8_t* data, size_t len) {
    _fifoPush(data, len);
    // _drainFifo() is called from update() so we stay on the main task.
    // Call it here too so data is available as soon as possible when the
    // notification arrives on the NimBLE task.
    _drainFifo();
}

// ── FIFO ─────────────────────────────────────────────────────────────────────
void RaceBoxBle::_fifoPush(const uint8_t* data, size_t len) {
    for (size_t i = 0; i < len; i++) {
        if (_fifoLen >= RACEBOX_FIFO_SIZE) {
            // Overflow: drop oldest byte
            _fifoTail = (_fifoTail + 1) % RACEBOX_FIFO_SIZE;
            _fifoLen--;
        }
        _fifo[_fifoHead] = data[i];
        _fifoHead = (_fifoHead + 1) % RACEBOX_FIFO_SIZE;
        _fifoLen++;
    }
}

bool RaceBoxBle::_fifoPeek(uint8_t* out, size_t len) const {
    if (_fifoLen < len) return false;
    size_t idx = _fifoTail;
    for (size_t i = 0; i < len; i++) {
        out[i] = _fifo[idx];
        idx = (idx + 1) % RACEBOX_FIFO_SIZE;
    }
    return true;
}

void RaceBoxBle::_fifoDrop(size_t len) {
    if (len > _fifoLen) len = _fifoLen;
    _fifoTail = (_fifoTail + len) % RACEBOX_FIFO_SIZE;
    _fifoLen -= len;
}

// ── _drainFifo() — extract and dispatch complete UBX frames ──────────────────
void RaceBoxBle::_drainFifo() {
    while (_fifoLen >= UBX_FRAME_OVERHEAD) {
        // Scan for sync bytes
        uint8_t b0, b1;
        uint8_t peek1[1];
        // Read first byte from tail
        if (!_fifoPeek(peek1, 1)) break;
        b0 = peek1[0];
        if (b0 != UBX_SYNC_1) {
            _fifoDrop(1);
            continue;
        }
        uint8_t peek2[2];
        if (!_fifoPeek(peek2, 2)) break;
        b1 = peek2[1];
        if (b1 != UBX_SYNC_2) {
            _fifoDrop(1);
            continue;
        }

        // Read length field (bytes 4..5 of frame)
        if (_fifoLen < 6) break;
        uint8_t hdr[6];
        if (!_fifoPeek(hdr, 6)) break;
        uint16_t payloadLen = static_cast<uint16_t>(hdr[4]) | (static_cast<uint16_t>(hdr[5]) << 8);
        size_t frameLen = UBX_FRAME_OVERHEAD + payloadLen;

        if (payloadLen > UBX_MAX_PAYLOAD) {
            // Invalid length — discard sync byte and re-sync
            _fifoDrop(1);
            continue;
        }
        if (_fifoLen < frameLen) break;  // frame not yet complete

        // Extract full frame
        uint8_t frame[UBX_FRAME_OVERHEAD + UBX_MAX_PAYLOAD];
        _fifoPeek(frame, frameLen);
        _fifoDrop(frameLen);

        // Decode and dispatch
        UbxPacket pkt{};
        if (!ubxDecode(frame, frameLen, pkt)) continue;

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
