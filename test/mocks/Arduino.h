#pragma once
// Minimal Arduino type stubs for native (host g++) compilation.
// Provides only the types needed by the natively-tested modules
// (ubx.h, ubx.cpp, RaceBoxParser.h, RaceBoxParser.cpp).
// Arduino-dependent modules (RaceBoxBle, RaceBoxRecorder, RaceBoxDownloader)
// are excluded from native CI compilation.
#include <stdint.h>
#include <stddef.h>
#include <string.h>
