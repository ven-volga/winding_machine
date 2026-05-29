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

    uint32_t spindleSpeed = 300;
    float    wireDiameter = 0.20f;
    float    windingWidth = 30.0f;
    bool     dirForward   = true;
    bool     running      = false;

    // Автореверс:
    //   true  = нонстоп (мотає поки педаль натиснута)
    //   false = пошаровий (один шар → стоп → педаль → зворотній шар → стоп)
    bool     autoReverse  = true;

    // ── Позиціювання каретки ─────────────────────────────────────

    bool     moveToActive = false;
    float    targetPos    = 0.0f;

    // ── motion → UI ──────────────────────────────────────────────

    uint32_t turns        = 0;
    float    carriagePos  = 0.0f;

    // Напрямок руху каретки в сервісному режимі.
    // Оновлюється в service.cpp при кожному кроці.
    // UI показує стрілку на основі цього поля.
    bool     serviceCarriageDir = true;  // true = вправо, false = вліво
    bool     serviceCarriageActive = false; // чи рухається каретка зараз
};

extern SharedState       shared;
extern SemaphoreHandle_t sharedMutex;

inline void shared_lock()   { xSemaphoreTake(sharedMutex, portMAX_DELAY); }
inline void shared_unlock() { xSemaphoreGive(sharedMutex); }

void shared_state_init();