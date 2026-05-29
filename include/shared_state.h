#pragma once

// =============================================================
// shared_state.h — спільний стан між ядрами
// =============================================================

#include <cstdint>

extern "C" {
    #include "freertos/FreeRTOS.h"
    #include "freertos/semphr.h"
}

struct SharedState
{
    // ── UI → motion ──────────────────────────────────────────────

    uint32_t spindleSpeed = 300;    // об/хв
    float    wireDiameter = 0.20f;  // мм
    float    windingWidth = 30.0f;  // мм
    bool     dirForward   = true;
    bool     running      = false;

    // ── Позиціювання каретки ─────────────────────────────────────
    // Коли moveToActive = true — motion ігнорує педаль і
    // переміщує каретку до targetPos на фіксованій швидкості.
    // Після досягнення — скидає moveToActive в false.

    bool     moveToActive = false;  // активне переміщення до позиції
    float    targetPos    = 0.0f;   // ціль переміщення (мм)

    // ── motion → UI ──────────────────────────────────────────────

    uint32_t turns       = 0;       // лічильник витків
    float    carriagePos = 0.0f;    // поточна позиція каретки (мм)
};

extern SharedState       shared;
extern SemaphoreHandle_t sharedMutex;

inline void shared_lock()   { xSemaphoreTake(sharedMutex, portMAX_DELAY); }
inline void shared_unlock() { xSemaphoreGive(sharedMutex); }

void shared_state_init();