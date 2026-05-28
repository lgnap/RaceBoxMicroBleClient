#pragma once
#include "ubx.h"
#include "RaceBoxData.h"

/**
 * Parse a RaceBox live (0xFF/0x01) or history (0xFF/0x21) UBX packet
 * into a RaceBoxData struct.
 *
 * Both message types carry an identical 80-byte payload — a single parser
 * handles both use-cases.
 *
 * Returns true on success, false if the packet class/ID is not supported
 * or the payload is too short.
 *
 * Payload offset reference (RaceBox BLE Protocol rev 8):
 *   [0..3]   iTOW              uint32 LE, ms
 *   [4..5]   year              uint16 LE
 *   [6]      month, [7] day, [8] hour, [9] minute, [10] second
 *   [11]     validityFlags
 *   [20]     fixStatus         0=no fix, 2=2D, 3=3D
 *   [23]     numSVs
 *   [24..27] longitude         int32 LE, 1e-7 deg
 *   [28..31] latitude          int32 LE, 1e-7 deg
 *   [32..35] wgsAltitude       int32 LE, mm above WGS84 ellipsoid
 *   [36..39] altitude          int32 LE, mm above MSL
 *   [48..51] speed             uint32 LE, mm/s (ground speed)
 *   [52..55] heading           uint32 LE, 1e-5 deg
 *   [56..59] speedAccuracy     uint32 LE, mm/s
 *   [60..61] gForceX           int16 LE, mG
 *   [62..63] gForceY           int16 LE, mG
 *   [64..65] gForceZ           int16 LE, mG
 *   [66..67] rotRateX          int16 LE, 0.01 deg/s
 *   [68..69] rotRateY          int16 LE, 0.01 deg/s
 *   [70..71] rotRateZ          int16 LE, 0.01 deg/s
 *   [72]     batteryLevel      uint8, %
 *   [73]     batteryStatus     uint8, bit 0 = charging
 */
bool raceBoxParse(const UbxPacket& pkt, RaceBoxData& out);
