#pragma once

#include <stdint.h>
#include <stddef.h>
#include <functional>
#include <freertos/FreeRTOS.h>
#include <freertos/semphr.h>
#include "ubx.h"

// FIFO capacity — enough for a burst of history records during download.
// A full BLE notification is 244 bytes (~2-3 records of 88 bytes each).
// The device streams at ~100 kbps during download; 4096 bytes ≈ 46 records
// of headroom, covering ~320 ms with a fast callback.
// Increase if your callback blocks (e.g. SD writes > 300 ms).
static constexpr size_t RACEBOX_FIFO_SIZE = 4096;

/**
 * Thread-safe ring buffer that accumulates raw BLE bytes and extracts
 * complete UBX frames on demand.
 *
 * push() is safe to call from any FreeRTOS task (e.g. the NimBLE notification
 * handler). extractFrame() must be called from a single consumer task (the
 * Arduino main loop via RaceBoxBle::update()).
 *
 * extractFrame() scans the byte stream for UBX sync bytes (0xB5 0x62) and
 * returns one complete, checksum-valid frame per call. Garbage bytes before a
 * valid sync header are silently discarded.
 */
class RaceBoxFifo {
public:
    RaceBoxFifo();

    // Push raw bytes into the FIFO. Thread-safe (acquires internal mutex).
    // If the FIFO is full the oldest bytes are dropped and the overflow
    // callback is fired outside the mutex after the push completes.
    void push(const uint8_t* data, size_t len);

    // Extract one complete UBX frame from the accumulated byte stream.
    // Thread-safe: acquires/releases the internal mutex for the extraction
    // phase; the caller may safely call push() or sendCommand() after return.
    // Returns true and writes the raw frame to frame[0..frameLen-1].
    // frame[] must be at least (UBX_FRAME_OVERHEAD + UBX_MAX_PAYLOAD) bytes.
    // Returns false if no complete frame is available yet.
    bool extractFrame(uint8_t* frame, size_t& frameLen);

    // Number of bytes currently buffered.
    size_t available() const { return _len; }

    // Cumulative bytes dropped due to FIFO overflow since construction.
    uint32_t overflowCount() const { return _overflowCount; }

    // Optional: called (outside the mutex, from push()'s caller context) when
    // the FIFO overflows. Keep the callback short — it runs on the NimBLE task.
    void setOverflowCallback(std::function<void(uint32_t)> cb) {
        _overflowCb = std::move(cb);
    }

private:
    SemaphoreHandle_t _mutex;

    uint8_t  _buf[RACEBOX_FIFO_SIZE];
    size_t   _head          = 0;
    size_t   _tail          = 0;
    size_t   _len           = 0;
    uint32_t _overflowCount = 0;

    std::function<void(uint32_t)> _overflowCb;

    // Internal helpers — must be called with _mutex held.
    bool _peek(uint8_t* out, size_t len) const;
    void _drop(size_t len);
};
