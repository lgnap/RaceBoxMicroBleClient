#include "RaceBoxFifo.h"
#include <string.h>

// ── Constructor ───────────────────────────────────────────────────────────────
RaceBoxFifo::RaceBoxFifo() {
    _mutex = xSemaphoreCreateMutex();
    memset(_buf, 0, sizeof(_buf));
}

// ── push() ────────────────────────────────────────────────────────────────────
void RaceBoxFifo::push(const uint8_t* data, size_t len) {
    if (_mutex == nullptr) return;
    bool overflowed = false;

    xSemaphoreTake(_mutex, portMAX_DELAY);
    for (size_t i = 0; i < len; i++) {
        if (_len >= RACEBOX_FIFO_SIZE) {
            // Overflow: silently drop the oldest byte
            _tail = (_tail + 1) % RACEBOX_FIFO_SIZE;
            _len--;
            _overflowCount++;
            overflowed = true;
        }
        _buf[_head] = data[i];
        _head = (_head + 1) % RACEBOX_FIFO_SIZE;
        _len++;
    }
    xSemaphoreGive(_mutex);

    // Notify outside the mutex — callback must not call back into push()
    if (overflowed && _overflowCb) _overflowCb(_overflowCount);
}

// ── clear() ───────────────────────────────────────────────────────────────────
void RaceBoxFifo::clear() {
    xSemaphoreTake(_mutex, portMAX_DELAY);
    _head = 0;
    _tail = 0;
    _len  = 0;
    xSemaphoreGive(_mutex);
}

// ── extractFrame() ────────────────────────────────────────────────────────────
bool RaceBoxFifo::extractFrame(uint8_t* frame, size_t& frameLen) {
    if (_mutex == nullptr) { frameLen = 0; return false; }
    frameLen = 0;

    xSemaphoreTake(_mutex, portMAX_DELAY);

    while (_len >= UBX_FRAME_OVERHEAD) {
        // Need at least sync1
        uint8_t sync1;
        if (!_peek(&sync1, 1)) break;
        if (sync1 != UBX_SYNC_1) { _drop(1); continue; }

        // Need sync1 + sync2
        uint8_t sync[2];
        if (!_peek(sync, 2)) break;
        if (sync[1] != UBX_SYNC_2) { _drop(1); continue; }

        // Need full header (6 bytes) to read payload length
        uint8_t hdr[6];
        if (!_peek(hdr, 6)) break;
        uint16_t payloadLen = static_cast<uint16_t>(hdr[4]) |
                              (static_cast<uint16_t>(hdr[5]) << 8);
        size_t flen = UBX_FRAME_OVERHEAD + payloadLen;

        if (payloadLen > UBX_MAX_PAYLOAD) { _drop(1); continue; }
        if (_len < flen) break;  // frame not yet complete

        _peek(frame, flen);
        _drop(flen);
        frameLen = flen;
        break;
    }

    xSemaphoreGive(_mutex);
    return frameLen > 0;
}

// ── Internal helpers (must be called with _mutex held) ────────────────────────

bool RaceBoxFifo::_peek(uint8_t* out, size_t len) const {
    if (_len < len) return false;
    size_t idx = _tail;
    for (size_t i = 0; i < len; i++) {
        out[i] = _buf[idx];
        idx = (idx + 1) % RACEBOX_FIFO_SIZE;
    }
    return true;
}

void RaceBoxFifo::_drop(size_t len) {
    if (len > _len) len = _len;
    _tail = (_tail + len) % RACEBOX_FIFO_SIZE;
    _len -= len;
}
