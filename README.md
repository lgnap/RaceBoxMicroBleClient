# RaceBoxMicroBleClient

Arduino/ESP32 BLE client library for [RaceBox Mini S and Micro](https://www.racebox.pro) devices.

Handles connection, UBX packet framing, live GNSS/IMU data parsing, standalone recording control, and full history download — with a clean, callback-based API.

---

## Table of Contents

- [Features](#features)
- [Compatibility](#compatibility)
- [Installation](#installation)
- [Quick Start](#quick-start)
- [Modules](#modules)
  - [RaceBoxBle — BLE transport](#raceboxble--ble-transport)
  - [RaceBoxData — data struct](#raceboxdata--data-struct)
  - [RaceBoxRecorder — recording control](#raceboxrecorder--recording-control)
  - [RaceBoxDownloader — history download](#raceboxdownloader--history-download)
- [Examples](#examples)
- [Notes & Caveats](#notes--caveats)
- [Testing](#testing)
- [License](#license)

---

## Features

| Module | What it does |
|--------|-------------|
| `RaceBoxBle` | BLE GATT client, FIFO ring buffer, reconnection, UBX framing |
| `RaceBoxParser` | Parses live (0xFF/0x01) and history (0xFF/0x21) packets into `RaceBoxData` |
| `RaceBoxRecorder` | Query status, start/stop standalone recording, erase memory |
| `RaceBoxDownloader` | Trigger and stream full history download with session boundaries |

---

## Compatibility

| Hardware | Status |
|----------|--------|
| RaceBox Micro | Tested |
| RaceBox Mini S | Compatible (same protocol) |
| ESP32 (any variant with BLE) | Required |

> **Not compatible with AVR / STM32 / RP2040** — depends on NimBLE (ESP32 only).

---

## Installation

### PlatformIO

Add to your `platformio.ini`:

```ini
[env:your_board]
platform = espressif32
framework = arduino
lib_deps =
    https://github.com/LGnap/RaceBoxMicroBleClient.git
```

### Arduino IDE

Download the repository as a ZIP and install via **Sketch → Include Library → Add .ZIP Library**.

---

## Quick Start

```cpp
#include <RaceBoxBle.h>

RaceBoxBle racebox([](const RaceBoxData& d) {
    if (d.fixStatus < 2) return;  // wait for GPS fix
    Serial.printf("lat=%.7f  lon=%.7f  spd=%.1f km/h  svs=%d\n",
                  d.latitude, d.longitude, d.speed, d.numSVs);
});

void setup() {
    Serial.begin(115200);
    racebox.begin();       // scan and connect automatically
}

void loop() {
    racebox.update();      // must be called every loop iteration
}
```

The library connects to the first RaceBox device it finds. Live data arrives at the configured data rate (1/5/10/25 Hz depending on device settings).

---

## Modules

### RaceBoxBle — BLE transport

```cpp
#include <RaceBoxBle.h>
```

`RaceBoxBle` manages the BLE connection and dispatches parsed packets to your callbacks.

```cpp
// Constructor: pass a live-data callback (or nullptr if not needed)
RaceBoxBle racebox([](const RaceBoxData& d) { /* called for each live point */ });

void setup() {
    racebox.begin();   // initialises NimBLE, starts scan
}

void loop() {
    racebox.update();  // drives reconnect + flushes FIFO — MUST be called every loop
}
```

#### Key methods

| Method | Description |
|--------|-------------|
| `begin()` | Initialise BLE and start scan. Call once in `setup()`. |
| `update()` | Process incoming data and drive reconnect. Call every `loop()`. |
| `isConnected()` | Returns `true` when a RaceBox is connected. |
| `sendCommand(pkt)` | Send a raw UBX packet to the device. |
| `setPacketCallback(cb)` | Receive all non-live UBX packets (used by Recorder / Downloader). |
| `setSniffCallback(cb)` | Receive every UBX packet before normal dispatch — for protocol debugging. |
| `setOverflowCallback(cb)` | Called if the internal FIFO overflows (slow callback warning). |
| `fifoOverflowCount()` | Total bytes dropped due to FIFO overflow since `begin()`. |

#### FIFO buffer

BLE notifications may carry multiple records in one burst. The library buffers incoming bytes in a 4096-byte FIFO and extracts complete UBX frames during `update()`. If your data callback is slow (e.g. writes to SD card taking > 300 ms), increase `RACEBOX_FIFO_SIZE` in `RaceBoxBle.h`.

---

### RaceBoxData — data struct

```cpp
#include <RaceBoxData.h>
```

All live and history records are delivered as a `RaceBoxData`:

```cpp
struct RaceBoxData {
    // Timestamp
    uint32_t iTOW;          // GPS time of week (ms)
    uint16_t year;
    uint8_t  month, day, hour, minute, second;
    uint8_t  validityFlags;

    // Fix quality
    uint8_t  fixStatus;     // 0=no fix, 2=2D fix, 3=3D fix
    uint8_t  numSVs;        // satellites used

    // Position
    float    latitude;      // degrees
    float    longitude;     // degrees
    float    altitude;      // m above mean sea level
    float    wgsAltitude;   // m above WGS84 ellipsoid

    // Motion
    float    speed;         // km/h (ground speed)
    float    heading;       // degrees (0–360)
    float    speedAccuracy; // km/h
    float    headingAccuracy; // degrees

    // Precision
    uint16_t pdop;          // position DOP × 100  (e.g. 120 = PDOP 1.20)

    // IMU
    int16_t  gForceX, gForceY, gForceZ;  // mG  (1 G = 1000 mG)
    int16_t  rotRateX, rotRateY, rotRateZ; // deg/s × 100

    // Battery (RaceBox Micro: raw byte = voltage × 10)
    uint8_t  batteryLevel;  // raw byte — interpret as voltage × 10 on Micro
    bool     charging;
};
```

> **Battery note (RaceBox Micro):** `batteryLevel` carries the supply voltage multiplied by 10. For example, `126` means 12.6 V. Display it as: `Serial.printf("%.1f V", d.batteryLevel / 10.0f);`

---

### RaceBoxRecorder — recording control

```cpp
#include <RaceBoxRecorder.h>
```

Controls standalone recording on the device. The recorder handles the memory unlock handshake automatically before every command.

```cpp
RaceBoxBle      racebox(nullptr);
RaceBoxRecorder rec(racebox);

void setup() {
    Serial.begin(115200);
    racebox.begin();
    rec.begin();                   // registers packet callback on racebox
    rec.setSecurityCode(123456);   // factory default — change if yours differs
}

void loop() {
    racebox.update();
    rec.update();   // drives ACK timeout watchdog
}
```

#### Commands (all async)

```cpp
// Query current status (state, memory level, record count)
rec.queryStatus();

// Start recording at 25 Hz
rec.startRecording(DataRate::HZ_25);

// Start with filters
rec.startRecording(
    DataRate::HZ_10,
    /* stationaryFilter */ true,   // suppress points when stationary > 30 s
    /* noFixFilter      */ true,   // suppress points when no GPS fix > 30 s
    /* autoShutdownMin  */ 5       // auto-shutdown after 5 min of inactivity
);

// Stop recording
rec.stopRecording();

// Erase all stored records (⚠ irreversible — takes several minutes)
rec.setEraseProgressCallback([](uint8_t pct) {
    Serial.printf("Erase: %d%%\n", pct);   // 100 = complete
});
rec.eraseMemory();

// Cancel an ongoing erase
rec.cancelErase();
```

#### State accessors (updated asynchronously on device response)

```cpp
rec.state()          // RecordingState: IDLE / RECORDING / PAUSED / UNKNOWN
rec.memoryLevel()    // memory used 0–100 %
rec.recordCount()    // number of stored records
rec.memorySize()     // total device capacity in records
rec.dataRate()       // current recording rate (from last STATE CHANGE)
rec.lastAck()        // true = last command ACKed, false = NACKed or timeout
rec.isErasing()      // true while erase is in progress
rec.eraseProgress()  // 0–100 %
```

#### Callbacks

```cpp
// Called when recording state changes (e.g. during concurrent download)
rec.setStateChangeCallback([](StateChangeEvent ev) {
    if (ev == StateChangeEvent::RECORDING_START) Serial.println("Recording started");
    if (ev == StateChangeEvent::RECORDING_STOP)  Serial.println("Recording stopped");
});

// Called on each erase progress notification (0–100 %)
rec.setEraseProgressCallback([](uint8_t pct) {
    Serial.printf("Erase: %d%%\n", pct);
});
```

#### Data rates

| Enum | Hz | Note |
|------|----|------|
| `DataRate::HZ_1`  | 1  | Not supported by RaceBox Micro firmware (NACK) |
| `DataRate::HZ_5`  | 5  | |
| `DataRate::HZ_10` | 10 | |
| `DataRate::HZ_25` | 25 | Default |

#### Memory unlock

The RaceBox requires a security code (0xFF/0x30) before accepting any recording command. The default code is `123456`. Call `setSecurityCode()` once — the Recorder sends it automatically before every command and after every reconnection.

---

### RaceBoxDownloader — history download

```cpp
#include <RaceBoxDownloader.h>
```

Downloads all stored records from the device. Records are streamed in the same format as live data and delivered one by one to your callback. Session boundaries (start/stop/pause/resume events) are reported via an optional state-change callback.

```cpp
RaceBoxBle        racebox(nullptr);
RaceBoxDownloader downloader(racebox, [](const RaceBoxData& d, uint32_t index) {
    Serial.printf("[%6lu] lat=%.7f lon=%.7f spd=%.1f km/h\n",
                  index, d.latitude, d.longitude, d.speed);
});

void setup() {
    Serial.begin(115200);
    racebox.begin();

    // Optional: detect session boundaries
    downloader.setStateChangeCallback([](uint8_t event) {
        // event: 0=stop, 1=start, 2=pause, 3=resume
        if (event == 1) Serial.println("--- session start ---");
        if (event == 0) Serial.println("--- session end ---");
    });

    // Wait until connected, then start download
    while (!racebox.isConnected()) { racebox.update(); delay(100); }
    downloader.begin();
}

void loop() {
    racebox.update();
    downloader.update();   // drives timeout watchdog

    if (downloader.isDone()) {
        Serial.printf("Done: %lu / %lu records\n",
                      downloader.recordCount(), downloader.expectedCount());
        while (true) delay(1000);
    }
    if (downloader.isError()) {
        Serial.println("Download error (timeout)");
        while (true) delay(1000);
    }
}
```

#### Methods

| Method | Description |
|--------|-------------|
| `begin()` | Send download trigger. Call once when connected. |
| `update()` | Drive inactivity timeout. Call every `loop()`. |
| `isDone()` | True when all records received and ACK arrived. |
| `isError()` | True on inactivity timeout (30 s without data). |
| `recordCount()` | Records received so far. |
| `expectedCount()` | Total records announced by the device. |
| `progressPercent()` | 0–100 %. |
| `setStateChangeCallback(cb)` | Receive session boundary events. |

#### Download speed

Typical throughput is **400–650 records/second** over BLE (88 bytes/record, ~100 kbps effective). A full 50 000-record memory downloads in under 2 minutes.

---

## Examples

| Example | Description |
|---------|-------------|
| `LiveData` | Connect and print live GNSS/IMU data to Serial |
| `DownloadHistory` | Trigger download and print all records to Serial |
| `DownloadBench` | Benchmark download speed with progress and session detection |
| `LibTest` | Full interactive harness (STATUS / REC START / DOWNLOAD / ERASE / RECBENCH …) |

Flash any example to an ESP32 board and open the Serial Monitor at **115200 baud**.

---

## Notes & Caveats

**Recorder and Downloader share the BLE packet callback.**
Only one can be active at a time. Calling `rec.begin()` registers the recorder; calling `downloader.begin()` re-registers the downloader. Use `LibTest` as a reference for toggling between them at runtime.

**No GPS fix = no records.**
The RaceBox does not store data without a 3D GPS fix (fixStatus = 3). Ensure the device has a clear sky view before expecting records.

**ERASE is irreversible and slow.**
Erasing the full memory takes several minutes. There is no per-session delete — only a full wipe. Monitor progress via `setEraseProgressCallback()` and cancel with `cancelErase()` if needed.

**BLE reconnection is automatic.**
`RaceBoxBle` will re-scan and reconnect after a disconnection. The Recorder will re-send the unlock command automatically after reconnect before the next command.

---

## Testing

The library includes a native (non-Arduino) test suite using the Unity framework:

```bash
make        # build and run all tests
make clean
```

See [TESTING.md](TESTING.md) for details.

---

## License

MIT
