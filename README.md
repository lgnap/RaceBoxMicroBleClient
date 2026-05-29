# RaceBoxMicroBleClient

Arduino/ESP32 BLE client library for [RaceBox Mini S and Micro](https://www.racebox.pro) devices.

## Features

- **UBX framing** — encode and decode UBX packets with Fletcher-16 checksum
- **Data parsing** — parse live data (0xFF/0x01) and history data (0xFF/0x21) into a `RaceBoxData` struct (GNSS position, speed, heading, altitude, IMU, battery)
- **BLE transport** — GATT client with FIFO buffer (device sends multiple records per notification)
- **Recording control** — query recording status (0xFF/0x22), configure and start/stop standalone recording (0xFF/0x25)
- **History download** — trigger download (0xFF/0x23), stream history records, handle state-change boundaries (0xFF/0x26)

## Compatibility

- RaceBox Mini S
- RaceBox Micro
- ESP32 (PlatformIO / Arduino framework)

## Protocol

Based on [RaceBox BLE Protocol Description rev 8](https://www.racebox.pro/products/mini-micro-protocol-documentation).

## Installation

### PlatformIO

```ini
[env:your_board]
lib_deps =
    https://github.com/LGnap/RaceBoxMicroBleClient.git
```

### Arduino IDE

Download the ZIP from the repository and install via **Sketch → Include Library → Add .ZIP Library**.

## Quick Start

```cpp
#include <RaceBoxBle.h>

RaceBoxBle racebox([](const RaceBoxData& d) {
    Serial.printf("lat=%.7f lon=%.7f spd=%.1f\n", d.latitude, d.longitude, d.speed);
});

void setup() {
    Serial.begin(115200);
    racebox.begin();
}

void loop() {
    racebox.update();
}
```

See [examples/](examples/) for complete usage.

## Examples

- **LiveData** — connect and print live GNSS/IMU data
- **DownloadHistory** — trigger standalone recording download and save to SPIFFS

## License

MIT
