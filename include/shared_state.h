#pragma once

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
    bool     autoReverse  = true;

    // Кількість витків для намотки (задається в UI)
    uint32_t targetTurns  = 0;      // 0 = не задано (мотає без ліміту)

    // ── Позиціювання ─────────────────────────────────────────────

    bool     moveToActive = false;
    float    targetPos    = 0.0f;

    // ── motion → UI ──────────────────────────────────────────────

    uint32_t turns        = 0;
    float    carriagePos  = 0.0f;

    // Сервісний режим
    bool     serviceCarriageDir    = true;
    bool     serviceCarriageActive = false;
};

extern SharedState       shared;
extern SemaphoreHandle_t sharedMutex;

inline void shared_lock()   { xSemaphoreTake(sharedMutex, portMAX_DELAY); }
inline void shared_unlock() { xSemaphoreGive(sharedMutex); }

void shared_state_init();