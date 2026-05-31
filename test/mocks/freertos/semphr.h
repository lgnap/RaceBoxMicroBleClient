#pragma once
// Minimal FreeRTOS semaphore stub for native compilation.
#include "FreeRTOS.h"
static inline SemaphoreHandle_t xSemaphoreCreateMutex()                           { return (void*)1; }
static inline int  xSemaphoreTake(SemaphoreHandle_t, TickType_t)                  { return 1; }
static inline int  xSemaphoreGive(SemaphoreHandle_t)                              { return 1; }
