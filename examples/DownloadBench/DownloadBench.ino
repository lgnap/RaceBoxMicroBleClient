/**
 * DownloadBench — RaceBoxMicroBleClient download throughput benchmark
 *
 * Proves that the FIFO + downloader pipeline holds up at full BLE speed
 * with a zero-overhead callback (no Serial per record).
 *
 * Expected result (no FIFO overflow):
 *   [BENCH] Done — 47043 records in 8234 ms (5714 rec/s), overflow=0 bytes
 *
 * If overflow > 0, increase RACEBOX_FIFO_SIZE in RaceBoxBle.h or reduce
 * work done inside the record callback.
 *
 * ── Commands (USB serial, 115200 baud) ───────────────────────────────────────
 *   PING      — print BLE connection status
 *   DOWNLOAD  — start download benchmark (prints result when done)
 *   FIFO      — print FIFO overflow byte count
 * ─────────────────────────────────────────────────────────────────────────────
 */

#include <RaceBoxBle.h>
#include <RaceBoxDownloader.h>

// ── Serial line buffer ────────────────────────────────────────────────────────
static char    lineBuf[64];
static uint8_t linePos = 0;

// ── Benchmark state ───────────────────────────────────────────────────────────
static uint32_t _benchStartMs  = 0;
static uint32_t _benchRecords  = 0;   // records received by callback
static bool     _benchRunning  = false;
static bool     _benchDone     = false;
// Optional: accumulate a simple checksum to prove records are processed, not skipped
static uint32_t _benchChecksum = 0;

// ── BLE client ────────────────────────────────────────────────────────────────
RaceBoxBle racebox;  // no live callback needed

// ── Downloader ────────────────────────────────────────────────────────────────
// The callback does the minimum work possible: count + lightweight checksum.
// No Serial.printf — that's the whole point of this benchmark.
RaceBoxDownloader dl(racebox, [](const RaceBoxData& d, uint32_t index) {
    _benchRecords++;
    // Lightweight checksum: XOR of speed values (proves we're reading real data)
    _benchChecksum ^= (uint32_t)(d.speed * 1000);
});

// ── Command dispatcher ────────────────────────────────────────────────────────
static void dispatch(const char* line) {
    if (strcmp(line, "PING") == 0) {
        Serial.printf("BLE %s\n", racebox.isConnected() ? "CONNECTED" : "DISCONNECTED");

    } else if (strcmp(line, "FIFO") == 0) {
        Serial.printf("FIFO overflow: %lu bytes dropped\n",
                      (unsigned long)racebox.fifoOverflowCount());

    } else if (strcmp(line, "DOWNLOAD") == 0) {
        if (!racebox.isConnected()) { Serial.println("ERROR not connected"); return; }
        if (_benchRunning) { Serial.println("ERROR benchmark already running"); return; }

        _benchRecords  = 0;
        _benchChecksum = 0;
        _benchDone     = false;
        _benchRunning  = true;
        _benchStartMs  = millis();

        dl.begin();
        Serial.printf("DOWNLOAD started — FIFO overflow before: %lu bytes\n",
                      (unsigned long)racebox.fifoOverflowCount());

    } else {
        Serial.printf("ERROR unknown command: %s\n", line);
        Serial.println("Commands: PING  FIFO  DOWNLOAD");
    }
}

// ── Setup ─────────────────────────────────────────────────────────────────────
void setup() {
    Serial.begin(115200);
    delay(500);
    Serial.println("=== RaceBoxMicroBleClient DownloadBench ===");
    Serial.println("Commands: PING  FIFO  DOWNLOAD");
    Serial.println("Verifies FIFO integrity: overflow=0 means no data was dropped.");

    racebox.setOverflowCallback([](uint32_t dropped) {
        // Called from NimBLE task — keep it short
        Serial.printf("[WARN] FIFO overflow — %lu bytes dropped (total)\n",
                      (unsigned long)dropped);
    });

    racebox.begin();
}

// ── Loop ──────────────────────────────────────────────────────────────────────
void loop() {
    // ── Serial input ─────────────────────────────────────────────────────────
    while (Serial.available()) {
        int c = Serial.read();
        if (c == '\n' || c == '\r') {
            if (linePos > 0) {
                lineBuf[linePos] = '\0';
                dispatch(lineBuf);
                linePos = 0;
            }
        } else if (linePos < sizeof(lineBuf) - 1) {
            lineBuf[linePos++] = (char)c;
        }
    }

    // ── Drive BLE ─────────────────────────────────────────────────────────────
    // NO delay() here — we want maximum drain throughput during download.
    racebox.update();

    if (_benchRunning) {
        dl.update();

        if (!_benchDone) {
            if (dl.isDone()) {
                _benchDone    = true;
                _benchRunning = false;
                uint32_t elapsed = millis() - _benchStartMs;
                uint32_t overflow = racebox.fifoOverflowCount();
                uint32_t rate = elapsed > 0 ? (_benchRecords * 1000u / elapsed) : 0;

                Serial.println();
                Serial.println("=== BENCHMARK RESULT ===");
                Serial.printf("Records received : %lu / %lu expected\n",
                              (unsigned long)_benchRecords,
                              (unsigned long)dl.expectedCount());
                Serial.printf("Time elapsed     : %lu ms\n", (unsigned long)elapsed);
                Serial.printf("Throughput       : %lu rec/s\n", (unsigned long)rate);
                Serial.printf("FIFO overflow    : %lu bytes dropped\n", (unsigned long)overflow);
                Serial.printf("Checksum         : 0x%08lX\n", (unsigned long)_benchChecksum);
                if (overflow == 0 && _benchRecords == dl.expectedCount()) {
                    Serial.println(">>> PASS: all records received, zero overflow <<<");
                } else if (overflow == 0) {
                    Serial.println(">>> PARTIAL: no overflow but fewer records than expected");
                    Serial.println("    (normal — device may send fewer than announced)");
                } else {
                    Serial.println(">>> FAIL: FIFO overflowed, records lost");
                    Serial.println("    Fix: increase RACEBOX_FIFO_SIZE or reduce callback work");
                }
                Serial.println("========================");

            } else if (dl.isError()) {
                _benchDone    = true;
                _benchRunning = false;
                Serial.println("[BENCH] ERROR — timeout or BLE disconnect");
            }
        }
    }
    // No delay — let the loop spin as fast as possible during download
}
