/**
 * LibTest — RaceBoxMicroBleClient interactive test harness
 *
 * Exercises all library modules (BLE, Recorder, Downloader) via USB serial
 * commands at 115200 baud.  Flash to any ESP32 board with built-in BLE.
 *
 * ── Commands ─────────────────────────────────────────────────────────────────
 *   STATUS             Query recording status (state + record count)
 *   REC START [hz]     Start standalone recording (hz = 1|5|10|25, default 25)
 *   REC STOP           Stop standalone recording
 *   DOWNLOAD           Download stored history (switches to downloader mode)
 *   LIVE               Toggle live data printing ON/OFF
 *   FIFO               Print FIFO overflow count
 *   PING               Print BLE connection status
 *
 * ── Modes ────────────────────────────────────────────────────────────────────
 *   Default : Recorder mode — STATUS / REC START / REC STOP available
 *   After DOWNLOAD : Downloader mode — progress printed until done
 *   After download completes, DOWNLOAD again to re-trigger
 *
 * ── Notes ────────────────────────────────────────────────────────────────────
 *   Recorder and Downloader share the BLE packet callback, so only one is
 *   active at a time.  Switching to DOWNLOAD re-registers the downloader
 *   callback; issuing STATUS after that re-registers the recorder callback.
 *
 * ── Concurrent live + download test ──────────────────────────────────────────
 *   Live data (0xFF/0x01) and history download (0xFF/0x21) use different UBX
 *   IDs.  The lib dispatches them to separate callbacks:
 *     - live  → _liveCb  (RaceBoxBle constructor)
 *     - other → _packetCb (Recorder / Downloader)
 *   They CAN coexist in the lib.  Whether the RaceBox firmware supports
 *   streaming live data simultaneously with a history download is a device
 *   question — not a library limitation.
 *
 *   To verify: while driving (live data streaming), issue DOWNLOAD.
 *   Expected result if the device supports concurrent streams:
 *     [LIVE] lines keep arriving interleaved with [DL NNNNN] lines.
 *   If live stops during download, the device enforces exclusivity itself.
 * ─────────────────────────────────────────────────────────────────────────────
 */

#include <RaceBoxBle.h>
#include <RaceBoxRecorder.h>
#include <RaceBoxDownloader.h>

// ── Serial line buffer ────────────────────────────────────────────────────────
static constexpr size_t LINE_BUF = 64;
static char    lineBuf[LINE_BUF];
static uint8_t linePos = 0;

// ── State ─────────────────────────────────────────────────────────────────────
static bool _liveEnabled   = true;   // print live GPS data
static bool _downloadMode  = false;  // true while downloader is active
static bool _downloadDone  = false;

// ── Live data callback ────────────────────────────────────────────────────────
static void onLiveData(const RaceBoxData& d) {
    if (!_liveEnabled) return;
    Serial.printf(
        "[LIVE] fix=%d svs=%d | lat=%.7f lon=%.7f alt=%.1fm | "
        "spd=%.1fkm/h hdg=%.1f | "
        "gX=%+5d gY=%+5d gZ=%+5d mG | rZ=%+6d (0.01deg/s) | batt=%d%%\n",
        d.fixStatus, d.numSVs,
        d.latitude, d.longitude, d.altitude,
        d.speed, d.heading,
        d.gForceX, d.gForceY, d.gForceZ,
        d.rotRateZ,
        d.batteryLevel);
}

// ── BLE client ───────────────────────────────────────────────────────────────
RaceBoxBle racebox(onLiveData);

// ── Recorder (default active mode) ───────────────────────────────────────────
RaceBoxRecorder rec(racebox);

// ── Downloader (activated on DOWNLOAD command) ───────────────────────────────
RaceBoxDownloader dl(racebox, [](const RaceBoxData& d, uint32_t index) {
    Serial.printf("[DL %5lu] %04d-%02d-%02d %02d:%02d:%02d | "
                  "lat=%.7f lon=%.7f spd=%.1fkm/h\n",
                  (unsigned long)index,
                  d.year, d.month, d.day,
                  d.hour, d.minute, d.second,
                  d.latitude, d.longitude, d.speed);
});

// ── Helpers ───────────────────────────────────────────────────────────────────

static const char* recStateStr(RecordingState s) {
    switch (s) {
        case RecordingState::IDLE:      return "IDLE";
        case RecordingState::RECORDING: return "RECORDING";
        case RecordingState::PAUSED:    return "PAUSED";
        default:                        return "UNKNOWN";
    }
}

static void activateRecorder() {
    rec.begin();  // re-registers packet callback on racebox
    _downloadMode = false;
}

// ── Command dispatcher ────────────────────────────────────────────────────────

static void dispatch(const char* line) {
    if (strcmp(line, "PING") == 0) {
        Serial.printf("BLE %s\n", racebox.isConnected() ? "CONNECTED" : "DISCONNECTED");

    } else if (strcmp(line, "LIVE") == 0) {
        _liveEnabled = !_liveEnabled;
        Serial.printf("LIVE %s\n", _liveEnabled ? "ON" : "OFF");

    } else if (strcmp(line, "FIFO") == 0) {
        Serial.printf("FIFO overflow: %lu bytes dropped\n",
                      (unsigned long)racebox.fifoOverflowCount());

    } else if (strcmp(line, "STATUS") == 0) {
        if (!racebox.isConnected()) { Serial.println("ERROR not connected"); return; }
        if (_downloadMode) activateRecorder();
        rec.queryStatus();
        Serial.println("STATUS sent — waiting for device response...");

    } else if (strncmp(line, "REC START", 9) == 0) {
        if (!racebox.isConnected()) { Serial.println("ERROR not connected"); return; }
        if (_downloadMode) activateRecorder();

        // Parse optional Hz argument
        unsigned hz = 25;
        if (line[9] == ' ') sscanf(line + 10, "%u", &hz);
        DataRate rate;
        switch (hz) {
            case  1: rate = DataRate::HZ_1;  break;  // ⚠ may NACK on RaceBox Micro
            case  5: rate = DataRate::HZ_5;  break;
            case 10: rate = DataRate::HZ_10; break;
            default: rate = DataRate::HZ_25; hz = 25; break;
        }
        rec.startRecording(rate);
        Serial.printf("REC START %uHz sent\n", hz);

    } else if (strcmp(line, "REC STOP") == 0) {
        if (!racebox.isConnected()) { Serial.println("ERROR not connected"); return; }
        if (_downloadMode) activateRecorder();
        rec.stopRecording();
        Serial.println("REC STOP sent");

    } else if (strcmp(line, "DOWNLOAD") == 0) {
        if (!racebox.isConnected()) { Serial.println("ERROR not connected"); return; }
        dl.begin();  // registers downloader callback, sends 0xFF/0x23 trigger
        _downloadMode = true;
        _downloadDone = false;
        Serial.println("DOWNLOAD triggered — streaming records...");

    } else {
        Serial.printf("ERROR unknown command: %s\n", line);
        Serial.println("Commands: PING  LIVE  FIFO  STATUS  REC START [hz]  REC STOP  DOWNLOAD");
    }
}

// ── Setup ─────────────────────────────────────────────────────────────────────
void setup() {
    Serial.begin(115200);
    delay(500);
    Serial.println("=== RaceBoxMicroBleClient LibTest ===");
    Serial.println("Commands: PING  LIVE  FIFO  STATUS  REC START [hz]  REC STOP  DOWNLOAD");

    racebox.setOverflowCallback([](uint32_t dropped) {
        Serial.printf("[WARN] FIFO overflow — %lu bytes dropped (total)\n", (unsigned long)dropped);
    });

    racebox.begin();
    rec.begin();  // default: recorder mode

    rec.setStateChangeCallback([](StateChangeEvent ev) {
        const char* evStr;
        switch (ev) {
            case StateChangeEvent::RECORDING_START:  evStr = "RECORDING_START"; break;
            case StateChangeEvent::RECORDING_STOP:   evStr = "RECORDING_STOP";  break;
            case StateChangeEvent::RECORDING_PAUSE:  evStr = "RECORDING_PAUSE"; break;
            case StateChangeEvent::RECORDING_RESUME: evStr = "RECORDING_RESUME"; break;
            default:                                 evStr = "UNKNOWN"; break;
        }
        Serial.printf("[STATE] %s\n", evStr);
    });
}

// ── Loop ──────────────────────────────────────────────────────────────────────
void loop() {
    // ── Read serial commands ──────────────────────────────────────────────────
    while (Serial.available()) {
        int c = Serial.read();
        if (c == '\n' || c == '\r') {
            if (linePos > 0) {
                lineBuf[linePos] = '\0';
                dispatch(lineBuf);
                linePos = 0;
            }
        } else if (linePos < LINE_BUF - 1) {
            lineBuf[linePos++] = (char)c;
        }
    }

    // ── Drive BLE + active module ─────────────────────────────────────────────
    racebox.update();

    if (_downloadMode) {
        dl.update();

        // Print progress every 5 % milestone
        static uint8_t lastPct = 0xFF;
        uint8_t pct = dl.progressPercent();
        if (pct != lastPct && pct % 5 == 0) {
            lastPct = pct;
            Serial.printf("[DL] %u%% (%lu / %lu)\n",
                          pct,
                          (unsigned long)dl.recordCount(),
                          (unsigned long)dl.expectedCount());
        }

        if (dl.isDone() && !_downloadDone) {
            _downloadDone = true;
            Serial.printf("[DL] Done — %lu records downloaded\n",
                          (unsigned long)dl.recordCount());
            activateRecorder();  // return to recorder mode
        } else if (dl.isError() && !_downloadDone) {
            _downloadDone = true;
            Serial.println("[DL] ERROR — timeout or BLE disconnect");
            activateRecorder();
        }
    } else {
        rec.update();

        // Print recorder status response when it arrives
        static RecordingState lastState = RecordingState::UNKNOWN;
        static uint32_t lastCount = 0;
        if (rec.state() != lastState || rec.recordCount() != lastCount) {
            lastState = rec.state();
            lastCount = rec.recordCount();
            Serial.printf("[REC] state=%s records=%lu ack=%s\n",
                          recStateStr(rec.state()),
                          (unsigned long)rec.recordCount(),
                          rec.lastAck() ? "ACK" : "NACK");
        }
    }

    delay(10);
}
