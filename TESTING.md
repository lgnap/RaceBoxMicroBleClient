# RaceBoxMicroBleClient — Test Protocol

## Native unit tests (CI)

Run on any host with g++ (no BLE hardware required):

```bash
cd test
make
```

**87 tests, 4 suites** — all must pass before any release:

| Suite | Tests | Covers |
|---|---|---|
| `test_ubx` | 13 | UBX frame encode/decode, Fletcher-16 checksum |
| `test_parser` | 29 | RaceBoxParser field extraction, boundary cases |
| `test_recorder` | 21 | RaceBoxRecorder state machine, ACK timeout |
| `test_downloader` | 24 | RaceBoxDownloader state machine, 30 s timeout |

---

## Physical device validation (on-device)

Requires: ESP32 board + RaceBox Micro with BLE visible.

### 1 — BLE connection

| Step | Expected |
|---|---|
| Power RaceBox Micro. Power ESP32 running the BikeTrace firmware. | `[BLE] scanning...` in serial output. |
| Wait for scan to find the device (≤ 30 s). | `[BLE] connecting to RaceBox ...` then `[BLE] connected`. |
| Walk away and back (signal test). | Brief disconnect triggers `[BLE] disconnected`, then automatic reconnect within 15 s. |

### 2 — Live data stream

| Step | Expected |
|---|---|
| While connected and outdoors (GNSS fix), observe serial output. | `[GPS] ...` lines appear at the configured data rate (10 or 25 Hz). |
| Speed field while stationary. | Speed ≈ 0.0 km/h. |
| Speed field while moving at known speed (e.g. car at 30 km/h). | Speed within ±2 km/h of reference. |
| Latitude/longitude while stationary. | Values stable to ≤ 0.0001° over 1 minute. |
| Battery level. | 0–100 %, plausible for charge state. |
| Charging flag. | `true` when USB connected, `false` on battery. |
| G-Force Z at rest (flat surface). | ≈ 1000 mG (≈ 1 G). |

### 3 — Recording control (RaceBoxRecorder)

| Step | Expected |
|---|---|
| Call `rec.queryStatus()`. | Status response arrives; `rec.state()` is `IDLE` or `RECORDING`. |
| Call `rec.startRecording(DataRate::HZ_25)`. | ACK received; `rec.lastAck()` = true. Device starts recording. |
| Call `rec.queryStatus()` after 5 s. | `rec.state()` = `RECORDING`, `rec.recordCount()` > 0. |
| Call `rec.stopRecording()`. | ACK received; subsequent `queryStatus()` shows `IDLE`. |
| Disconnect BLE mid-command (pull power). | No crash; state machine accepts next connect gracefully. |

### 4 — History download (RaceBoxDownloader)

Precondition: at least one session recorded (step 3).

| Step | Expected |
|---|---|
| Instantiate `RaceBoxDownloader` with a record callback, call `begin()`. | `[Downloader] Download trigger sent`; `expectedCount()` set within 2 s. |
| Monitor `progressPercent()` while records stream. | Increases monotonically from 0 % to 100 %. |
| Wait for `isDone()`. | All records received; `recordCount()` == `expectedCount()`. |
| Verify record data. | First record timestamp matches recording start time (± 1 s). Lat/lon within 50 m of known recording location. |
| State-change callback. | Fires with `RECORDING_START` event at session boundary. |
| Timeout test: start download, disable BLE (power off device). | After 30 s, `isError()` = true; no crash. |

### 5 — UBX framing integrity

| Step | Expected |
|---|---|
| Receive 500 consecutive live records while driving. | Zero parse failures logged; `raceBoxParse` returns true for every record. |
| Trigger a long BLE notification burst (fast acceleration data). | FIFO does not overflow (check `ble.fifoOverflowCount()` == 0). |

### 6 — Edge conditions

| Step | Expected |
|---|---|
| RaceBox Micro with full memory (erase to test erase command). | `UBX_ID_ERASE` command sends without crash. |
| GNSS cold start (no fix for > 2 min). | `fixStatus` = 0; `numSVs` = 0; no crash in callbacks. |
| Battery at 0 % (simulated by reading raw byte 0x00 from log). | `batteryLevel` = 0; no underflow. |

---

## Release checklist

- [ ] `make` in `test/` → **87/87 tests pass**
- [ ] Live data stream verified outdoors (section 2)
- [ ] Record start/stop cycle confirmed (section 3)
- [ ] Full history download matches expected count (section 4)
- [ ] No FIFO overflow over 500 live records (section 5)
- [ ] CHANGELOG updated with version and date
