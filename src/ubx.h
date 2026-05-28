#pragma once

#include <stdint.h>
#include <stddef.h>

// ── UBX sync bytes ────────────────────────────────────────────────────────────
static constexpr uint8_t UBX_SYNC_1 = 0xB5;
static constexpr uint8_t UBX_SYNC_2 = 0x62;

// ── RaceBox custom class ──────────────────────────────────────────────────────
static constexpr uint8_t UBX_CLASS_RACEBOX = 0xFF;

// ── RaceBox message IDs ───────────────────────────────────────────────────────
static constexpr uint8_t UBX_ID_LIVE          = 0x01;  // live data (0xFF/0x01)
static constexpr uint8_t UBX_ID_ACK           = 0x02;  // acknowledgement
static constexpr uint8_t UBX_ID_NACK          = 0x03;  // negative acknowledgement
static constexpr uint8_t UBX_ID_HISTORY       = 0x21;  // history download record
static constexpr uint8_t UBX_ID_REC_STATUS    = 0x22;  // recording status query/response
static constexpr uint8_t UBX_ID_DOWNLOAD      = 0x23;  // trigger / count response
static constexpr uint8_t UBX_ID_ERASE         = 0x24;  // erase stored data
static constexpr uint8_t UBX_ID_REC_CONFIG    = 0x25;  // recording config + start/stop
static constexpr uint8_t UBX_ID_STATE_CHANGE  = 0x26;  // recording boundary marker
static constexpr uint8_t UBX_ID_GNSS_CONFIG   = 0x27;  // GNSS platform config
static constexpr uint8_t UBX_ID_MEM_UNLOCK    = 0x30;  // memory security unlock

// ── Frame overhead (bytes) ────────────────────────────────────────────────────
// Full frame: sync(2) + class(1) + id(1) + length(2) + payload(N) + checksum(2)
static constexpr size_t UBX_FRAME_OVERHEAD = 8;
static constexpr size_t UBX_MAX_PAYLOAD    = 256;

/**
 * Decoded UBX packet.
 * `len` is the payload length in bytes (matches the 16-bit length field).
 * `payload[0..len-1]` contains the raw payload bytes.
 */
struct UbxPacket {
    uint8_t  cls;
    uint8_t  id;
    uint16_t len;
    uint8_t  payload[UBX_MAX_PAYLOAD];
};

/**
 * Compute Fletcher-16 checksum over bytes [startIdx .. startIdx+count-1].
 * For a full UBX frame the checksum covers bytes 2..(frameLen-3).
 */
void ubxChecksum(const uint8_t* data, size_t startIdx, size_t count,
                 uint8_t& ckA, uint8_t& ckB);

/**
 * Decode a UBX frame from `buf` (length `len`).
 * Verifies sync bytes, payload length, and Fletcher-16 checksum.
 * Returns true and fills `out` on success; returns false otherwise.
 */
bool ubxDecode(const uint8_t* buf, size_t len, UbxPacket& out);

/**
 * Encode `pkt` as a complete UBX frame into `out` (capacity `outLen`).
 * Returns the number of bytes written, or 0 if `out` is too small.
 */
size_t ubxEncode(const UbxPacket& pkt, uint8_t* out, size_t outLen);
