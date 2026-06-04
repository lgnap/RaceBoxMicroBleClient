/**
 * DownloadHistory — RaceBoxMicroBleClient example
 *
 * Connects to a RaceBox Mini S / Micro, triggers a history download,
 * and writes all recorded data points to Serial.
 *
 * The download streams 0xFF/0x21 messages (identical payload to live
 * 0xFF/0x01 data) and intercepts 0xFF/0x26 State Change messages to
 * detect recording start/stop/pause boundaries.
 *
 * Wiring: any ESP32 board with built-in BLE.
 */
#include <RaceBoxBle.h>
#include <RaceBoxDownloader.h>

RaceBoxBle racebox(nullptr);  // no live-data callback needed for download

RaceBoxDownloader downloader(racebox, [](const RaceBoxData& d, uint32_t index) {
    Serial.printf("[%6lu] %04d-%02d-%02d %02d:%02d:%02d | "
                  "lat=%.7f lon=%.7f spd=%.1fkm/h\n",
                  index,
                  d.year, d.month, d.day,
                  d.hour, d.minute, d.second,
                  d.latitude, d.longitude, d.speed);
});

void setup() {
    Serial.begin(115200);
    racebox.setDebugCallback([](const char* msg) { Serial.println(msg); });
    racebox.begin();

    // Wait until connected
    while (!racebox.isConnected()) {
        racebox.update();
        delay(100);
    }
    Serial.println("Connected! Starting download...");
    downloader.begin();
}

void loop() {
    racebox.update();
    downloader.update();

    if (downloader.isDone()) {
        Serial.printf("Download complete: %lu records\n", downloader.recordCount());
        while (true) delay(1000);  // halt
    }
}
