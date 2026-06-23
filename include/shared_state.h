#pragma once

// =============================================================
// shared_state.h — спільний стан між ядрами
//
// shared_lock() тепер повертає bool і логує таймаут.
// На етапі налагодження це допомагає ловити дедлоки.
// Після стабілізації можна повернути portMAX_DELAY.
// =============================================================

#include <cstdint>
#include <cstdio>

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
    uint32_t targetTurns  = 0;

    // ── Позиціювання ─────────────────────────────────────────────

    bool     moveToActive = false;
    float    targetPos    = 0.0f;

    // ── motion → UI ──────────────────────────────────────────────

    uint32_t turns        = 0;
    float    carriagePos  = 0.0f;

    // ── Сервісний режим ──────────────────────────────────────────

    bool     serviceCarriageDir    = true;
    bool     serviceCarriageActive = false;

    // Утримання шпинделя після зупинки (тумблер SW_SPINDLE_HOLD)
    // true = залишати spindle_enable(true) після зупинки
    bool     spindleHold = false;
};

extern SharedState       shared;
extern SemaphoreHandle_t sharedMutex;

// shared_lock з таймаутом 100мс для налагодження.
// Якщо таймаут — логує помилку і повертає false.
// Завжди перевіряй повернуте значення в критичних місцях.
inline bool shared_lock()
{
    if (xSemaphoreTake(sharedMutex, pdMS_TO_TICKS(100)) == pdTRUE)
        return true;
    printf("[ERROR] shared_lock timeout! Check for deadlock.\n");
    return false;
}

inline void shared_unlock()
{
    xSemaphoreGive(sharedMutex);
}

void shared_state_init();