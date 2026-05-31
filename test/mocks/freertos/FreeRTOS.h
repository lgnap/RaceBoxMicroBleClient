#pragma once
// Minimal FreeRTOS stub for native (host g++) compilation.
// Provides only the types needed by RaceBoxBle.h.
#include <stdint.h>
typedef void* SemaphoreHandle_t;
typedef uint32_t TickType_t;
#define portMAX_DELAY ((TickType_t)0xFFFFFFFFUL)
