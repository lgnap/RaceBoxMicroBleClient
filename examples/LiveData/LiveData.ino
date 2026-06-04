/**
 * LiveData — RaceBoxMicroBleClient example
 *
 * Connects to a RaceBox Mini S / Micro and prints live GNSS + IMU data
 * to the serial monitor at 115200 baud.
 *
 * Wiring: any ESP32 board with built-in BLE (e.g. TTGO LoRa32, ESP32 DevKit).
 */
#include <RaceBoxBle.h>

RaceBoxBle racebox([](const RaceBoxData& d) {
    if (d.fixStatus < 2) return;
    Serial.printf("[GPS] %04d-%02d-%02d %02d:%02d:%02d | "
                  "lat=%.7f lon=%.7f alt=%.1fm | "
                  "spd=%.1fkm/h hdg=%.1f | "
                  "svs=%d gX=%d gY=%d gZ=%d | batt=%d%%\n",
                  d.year, d.month, d.day,
                  d.hour, d.minute, d.second,
                  d.latitude, d.longitude, d.altitude,
                  d.speed, d.heading,
                  d.numSVs, d.gForceX, d.gForceY, d.gForceZ,
                  d.batteryLevel);
});

void setup() {
    Serial.begin(115200);
    racebox.setDebugCallback([](const char* msg) { Serial.println(msg); });
    racebox.begin();
}

void loop() {
    racebox.update();
    delay(10);
}
