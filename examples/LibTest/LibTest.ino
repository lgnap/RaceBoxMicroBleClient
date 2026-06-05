/**
 * LibTest — RaceBoxMicroBleClient interactive test harness
 *
 * Exercises all library modules (BLE, Recorder, Downloader) via USB serial
 * commands at 115200 baud.  Flash to any ESP32 board with built-in BLE.
 *
 * ── Commands ─────────────────────────────────────────────────────────────────
 *   STATUS             Query recording status (state + record count)
 *   REC START [hz]     Start standalone recording (hz = 5|10|25, default 25)
 *   REC STOP           Stop standalone recording
 *   ERASE              Erase all stored records (⚠ irreversible, takes minutes)
 *   ERASE CANCEL       Cancel an ongoing erase
 *   RECBENCH           Auto-chain 3 sessions (10s / 30s / 60s) with live status
 *   RECBENCH STOP      Abort RECBENCH early
 *   CONFIG TEST        Verify config is preserved across stop/start (no re-send)
 *   CONFIG VERIFY      Single-shot: use stored config, start, print confirmed config
 *   DOWNLOAD           Download stored history (switches to downloader mode)
 *   LIVE               Toggle live data printing ON/OFF
 *   RAWDUMP            Toggle raw UBX packet hex dump (protocol debugging)
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

// ── CONFIG TEST / CONFIG VERIFY state machine ─────────────────────────────────
// CONFIG TEST: explicit config → start → stop → no-args start → compare confirmed configs
// CONFIG VERIFY: set stored config → no-args start → print confirmed config → stop
enum class ConfigTestPhase : uint8_t {
    IDLE,
    CT_WAIT_START1,  // CONFIG TEST: waiting for 1st RECORDING_START
    CT_WAIT_STOP1,   // CONFIG TEST: waiting for RECORDING_STOP after 1st session
    CT_WAIT_START2,  // CONFIG TEST: waiting for 2nd RECORDING_START (no-args re-start)
    CT_WAIT_STOP2,   // CONFIG TEST: waiting for final RECORDING_STOP
    CV_WAIT_START,   // CONFIG VERIFY: waiting for RECORDING_START
    CV_WAIT_STOP,    // CONFIG VERIFY: waiting for RECORDING_STOP
};

static ConfigTestPhase _cfgTestPhase = ConfigTestPhase::IDLE;
static RecordingConfig _cfgConfirmed1;  // confirmed config from 1st session start
static RecordingConfig _cfgConfirmed2;  // confirmed config from 2nd session start (no-args)
static RecordingConfig _cfgCaptured;    // confirmed config for CONFIG VERIFY

// ── RECBENCH state machine ────────────────────────────────────────────────────
// RECBENCH chains N recording sessions with configurable durations.
// State machine driven by STATE CHANGE events + millis() timer.
enum class BenchRecState : uint8_t {
    IDLE,       // not running
    STARTING,   // rec.startRecording() sent, waiting for RECORDING_START event
    RECORDING,  // recording, timing until duration expires
    STOPPING,   // rec.stopRecording() sent, waiting for RECORDING_STOP event
    BETWEEN,    // brief pause (500ms) before next session
};

static const uint32_t BENCH_SESSION_DURATIONS_MS[] = { 10000, 30000, 60000 };
static constexpr uint8_t BENCH_SESSION_COUNT = 3;
static constexpr uint32_t BENCH_BETWEEN_MS   = 500;   // pause between sessions

static BenchRecState _benchRecState      = BenchRecState::IDLE;
static uint8_t       _benchRecSession    = 0;   // current session index (0-based)
static uint32_t      _benchRecStartMs    = 0;   // millis() when current phase began
static uint32_t      _benchRecordsAtSessionStart = 0;  // record count at session start (from STATUS)
static uint32_t      _benchRecLastStatusMs = 0; // millis() of last STATUS query during RECBENCH
static constexpr uint32_t BENCH_STATUS_INTERVAL_MS = 5000;  // query STATUS every 5s during recording

// ── Live data cache ───────────────────────────────────────────────────────────
static int16_t _lastBatteryRaw = -1;  // -1 = not yet received
// RaceBox Micro sends input voltage × 10 (e.g. 126 → 12.6 V, 50 → 5.0 V).
// Always display as voltage — do NOT interpret as percentage.
static void printBattery(uint8_t raw) {
    Serial.printf("%.1fV", raw / 10.0f);
}

// ── Live data callback ────────────────────────────────────────────────────────
static void onLiveData(const RaceBoxData& d) {
    _lastBatteryRaw = d.batteryLevel;
    if (!_liveEnabled) return;
    Serial.printf(
        "[LIVE] fix=%d svs=%d | lat=%.7f lon=%.7f alt=%.1fm | "
        "spd=%.1fkm/h hdg=%.1f | "
        "gX=%+5d gY=%+5d gZ=%+5d mG | rZ=%+6d (0.01deg/s) | batt=",
        d.fixStatus, d.numSVs,
        d.latitude, d.longitude, d.altitude,
        d.speed, d.heading,
        d.gForceX, d.gForceY, d.gForceZ,
        d.rotRateZ);
    printBattery(d.batteryLevel);
    Serial.println();
}

// ── BLE client ───────────────────────────────────────────────────────────────
RaceBoxBle racebox(onLiveData);

// ── Recorder (default active mode) ───────────────────────────────────────────
// Set your device's security code (RaceBox factory default: 123456).
// The Recorder sends unlock automatically before every REC START / REC STOP.
RaceBoxRecorder rec(racebox);
static constexpr uint32_t SECURITY_CODE = 123456;

// ── Downloader (activated on DOWNLOAD command) ───────────────────────────────
// Print 1 record out of DL_PRINT_EVERY to avoid FIFO overflow.
// Printing every record at 115200 baud (~7 ms/line × 25 records/s = overflow).
static constexpr uint32_t DL_PRINT_EVERY = 100;

RaceBoxDownloader dl(racebox, [](const RaceBoxData& d, uint32_t index) {
    if (index % DL_PRINT_EVERY == 0) {
        Serial.printf("[DL %5lu] %04d-%02d-%02d %02d:%02d:%02d | "
                      "lat=%.7f lon=%.7f spd=%.1fkm/h\n",
                      (unsigned long)index,
                      d.year, d.month, d.day,
                      d.hour, d.minute, d.second,
                      d.latitude, d.longitude, d.speed);
    }
});

// ── Raw packet debug dump ─────────────────────────────────────────────────────
static bool _rawDump = false;  // toggled with RAWDUMP command

static void dumpPacket(const char* tag, const UbxPacket& pkt) {
    Serial.printf("[RAW %s] cls=0x%02X id=0x%02X len=%u payload:",
                  tag, pkt.cls, pkt.id, pkt.len);
    for (uint16_t i = 0; i < pkt.len && i < 32; i++)
        Serial.printf(" %02X", pkt.payload[i]);
    Serial.println();
}

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

    } else if (strcmp(line, "RAWDUMP") == 0) {
        _rawDump = !_rawDump;
        if (_rawDump) {
            racebox.setSniffCallback([](const UbxPacket& pkt) { dumpPacket("RX", pkt); });
        } else {
            racebox.setSniffCallback(nullptr);
        }
        Serial.printf("RAWDUMP %s\n", _rawDump ? "ON (all UBX packets hex-dumped)" : "OFF");

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

    } else if (strcmp(line, "ERASE") == 0) {
        if (!racebox.isConnected()) { Serial.println("ERROR not connected"); return; }
        if (_downloadMode) activateRecorder();
        Serial.println("⚠  ERASE — erasing all stored records. This may take several minutes.");
        Serial.println("   Send ERASE CANCEL to abort.");
        rec.eraseMemory();

    } else if (strcmp(line, "ERASE CANCEL") == 0) {
        if (!rec.isErasing()) { Serial.println("No erase in progress"); return; }
        rec.cancelErase();

    } else if (strcmp(line, "RECBENCH") == 0) {
        if (!racebox.isConnected()) { Serial.println("ERROR not connected"); return; }
        if (_benchRecState != BenchRecState::IDLE) {
            Serial.println("ERROR RECBENCH already running (RECBENCH STOP to abort)");
            return;
        }
        if (_downloadMode) activateRecorder();
        _benchRecSession = 0;
        _benchRecState   = BenchRecState::STARTING;
        Serial.printf("[RECBENCH] Starting — %u sessions: ", BENCH_SESSION_COUNT);
        for (uint8_t i = 0; i < BENCH_SESSION_COUNT; i++) {
            Serial.printf("%lus%s",
                          (unsigned long)(BENCH_SESSION_DURATIONS_MS[i] / 1000),
                          i < BENCH_SESSION_COUNT - 1 ? " / " : "\n");
        }
        Serial.printf("[RECBENCH] Session 1/%u — sending REC START 25Hz...\n",
                      BENCH_SESSION_COUNT);
        rec.startRecording(DataRate::HZ_25);

    } else if (strcmp(line, "RECBENCH STOP") == 0) {
        if (_benchRecState == BenchRecState::IDLE) {
            Serial.println("RECBENCH not running");
        } else {
            _benchRecState = BenchRecState::IDLE;
            rec.stopRecording();
            Serial.println("[RECBENCH] Aborted");
        }

    } else if (strcmp(line, "CONFIG TEST") == 0) {
        if (!racebox.isConnected()) { Serial.println("ERROR not connected"); return; }
        if (_cfgTestPhase != ConfigTestPhase::IDLE) {
            Serial.println("ERROR CONFIG TEST already running"); return;
        }
        if (_benchRecState != BenchRecState::IDLE) {
            Serial.println("ERROR RECBENCH running — abort first with RECBENCH STOP"); return;
        }
        if (_downloadMode) activateRecorder();
        // Use a recognisable test config: HZ_10, stationary filter, no-fix off, 1 min shutdown
        _cfgTestPhase = ConfigTestPhase::CT_WAIT_START1;
        Serial.println("[CFGTEST] Starting phase 1 — explicit config: HZ_10, stationary=yes, shutdown=1min");
        rec.startRecording(DataRate::HZ_10, /*stationary=*/true, /*noFix=*/false, /*shutdownMin=*/1);

    } else if (strcmp(line, "CONFIG VERIFY") == 0) {
        if (!racebox.isConnected()) { Serial.println("ERROR not connected"); return; }
        if (_cfgTestPhase != ConfigTestPhase::IDLE) {
            Serial.println("ERROR CONFIG TEST already running"); return;
        }
        if (_downloadMode) activateRecorder();
        const RecordingConfig& cfg = rec.config();
        _cfgTestPhase = ConfigTestPhase::CV_WAIT_START;
        Serial.printf("[CFGVERIFY] Starting with stored config: rate=%dHz, stationary=%d, nofix=%d, shutdown=%umin\n",
                      (int)cfg.rate, (int)cfg.stationaryFilter, (int)cfg.noFixFilter, cfg.autoShutdownMin);
        rec.startRecording();  // no-args: uses stored _config

    } else if (strcmp(line, "DOWNLOAD") == 0) {
        if (!racebox.isConnected()) { Serial.println("ERROR not connected"); return; }
        dl.begin();  // registers downloader callback, sends 0xFF/0x23 trigger
        _downloadMode = true;
        _downloadDone = false;
        Serial.println("DOWNLOAD triggered — streaming records...");

    } else {
        Serial.printf("ERROR unknown command: %s\n", line);
        Serial.println("Commands: PING  LIVE  RAWDUMP  FIFO  STATUS  REC START [hz]  REC STOP");
        Serial.println("          ERASE  ERASE CANCEL  RECBENCH  RECBENCH STOP");
        Serial.println("          CONFIG TEST  CONFIG VERIFY  DOWNLOAD");
    }
}

// ── Setup ─────────────────────────────────────────────────────────────────────
void setup() {
    Serial.begin(115200);
    delay(500);
    Serial.println("=== RaceBoxMicroBleClient LibTest ===");
    Serial.println("Commands: PING  LIVE  RAWDUMP  FIFO  STATUS  REC START [hz]  REC STOP");
    Serial.println("          ERASE  ERASE CANCEL  RECBENCH  RECBENCH STOP");
    Serial.println("          CONFIG TEST  CONFIG VERIFY  DOWNLOAD");

    racebox.setDebugCallback([](const char* msg) { Serial.println(msg); });
    racebox.setErrorCallback([](uint32_t count) {
        Serial.printf("[BLE] UBX decode error #%lu\n", (unsigned long)count);
    });
    racebox.setOverflowCallback([](uint32_t dropped) {
        Serial.printf("[WARN] FIFO overflow — %lu bytes dropped (total)\n", (unsigned long)dropped);
    });

    racebox.begin();
    rec.setSecurityCode(SECURITY_CODE);
    rec.begin();  // default: recorder mode

    rec.setEraseProgressCallback([](uint8_t pct) {
        if (pct < 100)
            Serial.printf("[ERASE] %d%% ...\n", (int)pct);
        else
            Serial.println("[ERASE] Complete ✓ — memory erased. Run STATUS to confirm.");
    });

    // Capture confirmed config for CONFIG TEST / CONFIG VERIFY
    rec.setConfigConfirmedCallback([](const RecordingConfig& c) {
        if (_cfgTestPhase == ConfigTestPhase::CT_WAIT_START1) {
            _cfgConfirmed1 = c;
        } else if (_cfgTestPhase == ConfigTestPhase::CT_WAIT_START2) {
            _cfgConfirmed2 = c;
        } else if (_cfgTestPhase == ConfigTestPhase::CV_WAIT_START) {
            _cfgCaptured = c;
        }
    });

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

        // ── CONFIG TEST state machine ─────────────────────────────────────────
        if (_cfgTestPhase == ConfigTestPhase::CT_WAIT_START1 &&
            ev == StateChangeEvent::RECORDING_START) {
            Serial.println("[CFGTEST] Phase 1 START confirmed — stopping to re-start without config...");
            _cfgTestPhase = ConfigTestPhase::CT_WAIT_STOP1;
            rec.stopRecording();
            return;  // don't trigger RECBENCH
        }
        if (_cfgTestPhase == ConfigTestPhase::CT_WAIT_STOP1 &&
            ev == StateChangeEvent::RECORDING_STOP) {
            Serial.println("[CFGTEST] Phase 1 STOP — re-starting with stored config (no-args)...");
            _cfgTestPhase = ConfigTestPhase::CT_WAIT_START2;
            rec.startRecording();  // no-args: uses _config stored from phase 1
            return;
        }
        if (_cfgTestPhase == ConfigTestPhase::CT_WAIT_START2 &&
            ev == StateChangeEvent::RECORDING_START) {
            Serial.println("[CFGTEST] Phase 2 START confirmed — stopping...");
            _cfgTestPhase = ConfigTestPhase::CT_WAIT_STOP2;
            rec.stopRecording();
            return;
        }
        if (_cfgTestPhase == ConfigTestPhase::CT_WAIT_STOP2 &&
            ev == StateChangeEvent::RECORDING_STOP) {
            _cfgTestPhase = ConfigTestPhase::IDLE;
            // Compare both confirmed configs
            bool rateOk  = (_cfgConfirmed1.rate             == _cfgConfirmed2.rate);
            bool statOk  = (_cfgConfirmed1.stationaryFilter  == _cfgConfirmed2.stationaryFilter);
            bool nofixOk = (_cfgConfirmed1.noFixFilter       == _cfgConfirmed2.noFixFilter);
            bool shutOk  = (_cfgConfirmed1.autoShutdownSecs  == _cfgConfirmed2.autoShutdownSecs);
            bool pass    = rateOk && statOk && nofixOk && shutOk;
            Serial.println("[CFGTEST] =========== RESULT ===========");
            Serial.printf("[CFGTEST] Rate       : %d Hz → %d Hz %s\n",
                          (int)_cfgConfirmed1.rate, (int)_cfgConfirmed2.rate,
                          rateOk ? "OK" : "MISMATCH");
            Serial.printf("[CFGTEST] Stationary : %d → %d %s\n",
                          _cfgConfirmed1.stationaryFilter, _cfgConfirmed2.stationaryFilter,
                          statOk ? "OK" : "MISMATCH");
            Serial.printf("[CFGTEST] NoFix      : %d → %d %s\n",
                          _cfgConfirmed1.noFixFilter, _cfgConfirmed2.noFixFilter,
                          nofixOk ? "OK" : "MISMATCH");
            Serial.printf("[CFGTEST] Shutdown   : %u s → %u s %s\n",
                          _cfgConfirmed1.autoShutdownSecs, _cfgConfirmed2.autoShutdownSecs,
                          shutOk ? "OK" : "MISMATCH");
            Serial.println(pass
                ? "[CFGTEST] >>> PASS: config preserved across stop/start <<<"
                : "[CFGTEST] >>> FAIL: config changed between sessions <<<");
            Serial.println("[CFGTEST] ================================");
            return;
        }

        // ── CONFIG VERIFY state machine ───────────────────────────────────────
        if (_cfgTestPhase == ConfigTestPhase::CV_WAIT_START &&
            ev == StateChangeEvent::RECORDING_START) {
            Serial.println("[CFGVERIFY] START confirmed — stopping...");
            _cfgTestPhase = ConfigTestPhase::CV_WAIT_STOP;
            rec.stopRecording();
            return;
        }
        if (_cfgTestPhase == ConfigTestPhase::CV_WAIT_STOP &&
            ev == StateChangeEvent::RECORDING_STOP) {
            _cfgTestPhase = ConfigTestPhase::IDLE;
            Serial.println("[CFGVERIFY] ====== Confirmed Config from Device ======");
            Serial.printf("[CFGVERIFY] Rate             : %d Hz\n", (int)_cfgCaptured.rate);
            Serial.printf("[CFGVERIFY] StationaryFilter : %s",
                          _cfgCaptured.stationaryFilter ? "yes" : "no");
            if (_cfgCaptured.stationaryFilter)
                Serial.printf(" (speed<%u mm/s, interval=%u s)",
                              _cfgCaptured.stationarySpeedMmS, _cfgCaptured.stationaryIntervalS);
            Serial.println();
            Serial.printf("[CFGVERIFY] NoFixFilter      : %s",
                          _cfgCaptured.noFixFilter ? "yes" : "no");
            if (_cfgCaptured.noFixFilter)
                Serial.printf(" (interval=%u s)", _cfgCaptured.noFixIntervalS);
            Serial.println();
            Serial.printf("[CFGVERIFY] AutoShutdown     : %u s\n",
                          _cfgCaptured.autoShutdownSecs);
            Serial.println("[CFGVERIFY] ==========================================");
            return;
        }

        // ── RECBENCH state machine: advance on state change events ────────────
        if (_benchRecState == BenchRecState::STARTING &&
            ev == StateChangeEvent::RECORDING_START) {
            uint32_t durMs = BENCH_SESSION_DURATIONS_MS[_benchRecSession];
            _benchRecStartMs = millis();
            // Snapshot current count and immediately query to get fresh value
            _benchRecordsAtSessionStart = rec.recordCount();
            _benchRecLastStatusMs = millis();
            rec.queryStatus();  // will update rec.recordCount() async; snapshot updated below when response arrives
            Serial.printf("[RECBENCH] Session %u/%u — recording for %lu s (querying STATUS...)\n",
                          _benchRecSession + 1, BENCH_SESSION_COUNT,
                          (unsigned long)(durMs / 1000));
            _benchRecState = BenchRecState::RECORDING;
        }

        if (_benchRecState == BenchRecState::STOPPING &&
            ev == StateChangeEvent::RECORDING_STOP) {
            // Query final STATUS to get accurate record count before computing delta
            rec.queryStatus();
            Serial.printf("[RECBENCH] Session %u/%u — STOP received, querying final STATUS...\n",
                          _benchRecSession + 1, BENCH_SESSION_COUNT);
            // Delta will be printed by the [REC] status handler in loop() when response arrives
            _benchRecSession++;
            if (_benchRecSession >= BENCH_SESSION_COUNT) {
                Serial.printf("[RECBENCH] All %u sessions complete — see [REC] line for total records.\n",
                              BENCH_SESSION_COUNT);
                Serial.println("[RECBENCH] Now flash DownloadBench and run DOWNLOAD to verify boundaries.");
                _benchRecState = BenchRecState::IDLE;
            } else {
                // Brief pause before next session
                _benchRecStartMs = millis();
                _benchRecState   = BenchRecState::BETWEEN;
            }
        }
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

    // ── RECBENCH tick ─────────────────────────────────────────────────────────
    if (_benchRecState == BenchRecState::RECORDING) {
        uint32_t durMs = BENCH_SESSION_DURATIONS_MS[_benchRecSession];
        uint32_t elapsed = millis() - _benchRecStartMs;
        // Print countdown every second
        static uint32_t lastCountdown = 0;
        uint32_t remaining = (elapsed < durMs) ? (durMs - elapsed) / 1000 : 0;
        if (remaining != lastCountdown) {
            lastCountdown = remaining;
            if (remaining > 0) {
                Serial.printf("[RECBENCH] Session %u/%u — %lu s remaining",
                              _benchRecSession + 1, BENCH_SESSION_COUNT,
                              (unsigned long)remaining);
                if (_lastBatteryRaw >= 0) {
                    Serial.print(" | batt=");
                    printBattery((uint8_t)_lastBatteryRaw);
                }
                Serial.println();
            }
        }
        // Periodic STATUS query to keep rec.recordCount() up to date
        if (millis() - _benchRecLastStatusMs >= BENCH_STATUS_INTERVAL_MS) {
            _benchRecLastStatusMs = millis();
            rec.queryStatus();
        }
        if (elapsed >= durMs) {
            Serial.printf("[RECBENCH] Session %u/%u — duration reached, sending REC STOP...\n",
                          _benchRecSession + 1, BENCH_SESSION_COUNT);
            _benchRecState = BenchRecState::STOPPING;
            rec.stopRecording();
        }
    } else if (_benchRecState == BenchRecState::BETWEEN) {
        if (millis() - _benchRecStartMs >= BENCH_BETWEEN_MS) {
            _benchRecState = BenchRecState::STARTING;
            Serial.printf("[RECBENCH] Session %u/%u — sending REC START 25Hz...\n",
                          _benchRecSession + 1, BENCH_SESSION_COUNT);
            rec.startRecording(DataRate::HZ_25);
        }
    }

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
            uint32_t newCount = rec.recordCount();
            // RECBENCH: update session baseline on first STATUS after session start
            // (queryStatus() was sent right after RECORDING_START; this is the response)
            if (_benchRecState == BenchRecState::RECORDING && lastCount == _benchRecordsAtSessionStart) {
                _benchRecordsAtSessionStart = newCount;
            }
            lastState = rec.state();
            lastCount = newCount;
            Serial.printf("[REC] state=%s records=%lu/%lu mem=%u%% ack=%s",
                          recStateStr(rec.state()),
                          (unsigned long)rec.recordCount(),
                          (unsigned long)rec.memorySize(),
                          rec.memoryLevel(),
                          rec.lastAck() ? "ACK" : "NACK");
            // Show session delta if RECBENCH is active
            if (_benchRecState == BenchRecState::RECORDING || _benchRecState == BenchRecState::STOPPING) {
                uint32_t delta = (newCount >= _benchRecordsAtSessionStart)
                                 ? newCount - _benchRecordsAtSessionStart : 0;
                Serial.printf(" (+%lu this session)", (unsigned long)delta);
            }
            Serial.println();
        }
    }

    delay(10);
}
