#pragma once
// Minimal Arduino stubs for native (host g++) compilation.
// Covers all modules: ubx, RaceBoxParser, RaceBoxRecorder, RaceBoxDownloader.
#include <stdint.h>
#include <stddef.h>
#include <string.h>
#include <stdio.h>
#include <stdarg.h>

// ── millis() — controllable fake clock ───────────────────────────────────────
// Test files can manipulate _fakeMillis directly to simulate time passing.
inline uint32_t _fakeMillis = 0;
inline uint32_t millis() { return _fakeMillis; }

// ── Serial stub — no-op (all log output suppressed in native tests) ───────────
struct _FakeSerial {
    void println(const char*) {}
    void println(int) {}
    template<typename... Args>
    void printf(const char*, Args...) {}
};
inline _FakeSerial Serial;
