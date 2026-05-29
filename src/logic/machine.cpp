// machine.cpp
//
// Логіка верхнього рівня намотки:
//   - педаль: тримаєш → мотає, відпустив → зупиняє
//   - автоматичний реверс каретки при досягненні windingWidth
//   - сервісний режим: кнопки мають пріоритет над педаллю
//
// Швидкість береться з shared.spindleSpeed —
// UI змінює її напряму через енкодер під час роботи.

#include "machine.h"
#include "motion.h"
#include "service.h"
#include "shared_state.h"
#include "pins.h"
#include "config.h"

extern "C" {
    #include "freertos/FreeRTOS.h"
    #include "freertos/task.h"
    #include "driver/gpio.h"
}

// ─────────────────────────────────────────
// Debounce педалі
// ─────────────────────────────────────────

#define PEDAL_DEBOUNCE_MS  30

static bool     pedalState   = false;
static bool     lastRawPedal = false;
static uint32_t pedalDebTime = 0;

static bool pedal_read()
{
    uint32_t now = (uint32_t)(xTaskGetTickCount() * portTICK_PERIOD_MS);
    bool raw = (gpio_get_level((gpio_num_t)PEDAL_PIN) == 0);

    if (raw != lastRawPedal)
    {
        lastRawPedal = raw;
        pedalDebTime = now;
    }

    if ((now - pedalDebTime) >= PEDAL_DEBOUNCE_MS)
        pedalState = raw;

    return pedalState;
}

// ─────────────────────────────────────────
// Публічні функції
// ─────────────────────────────────────────

void machine_init()
{
    gpio_config_t conf = {};
    conf.pin_bit_mask  = (1ULL << PEDAL_PIN);
    conf.mode          = GPIO_MODE_INPUT;
    conf.pull_up_en    = GPIO_PULLUP_ENABLE;
    conf.pull_down_en  = GPIO_PULLDOWN_DISABLE;
    conf.intr_type     = GPIO_INTR_DISABLE;
    gpio_config(&conf);
}

void machine_update()
{
    // Сервісний режим має пріоритет
    if (service_active()) return;

    bool pedal = pedal_read();

    // Читаємо параметри з shared
    shared_lock();
    bool     running = shared.running;
    float    pos     = shared.carriagePos;
    float    width   = shared.windingWidth;
    bool     dir     = shared.dirForward;
    uint32_t speed   = shared.spindleSpeed;
    float    dia     = shared.wireDiameter;
    shared_unlock();

    // ── Педаль: старт/стоп ───────────────────────────────────────
    if (pedal && !running)
    {
        // Передаємо актуальні параметри в motion перед стартом
        motion_set_speed(speed);
        motion_set_wire_diameter(dia);
        motion_start();

        shared_lock();
        shared.running = true;
        shared_unlock();
    }
    else if (!pedal && running)
    {
        motion_stop();
        // shared.running скинеться в motion після гальмування
    }

    // ── Оновлення швидкості під час роботи ──────────────────────
    // UI змінює shared.spindleSpeed через енкодер —
    // передаємо нове значення в motion плавно
    if (running)
        motion_set_speed(speed);

    // ── Автоматичний реверс каретки ──────────────────────────────
    if (running)
    {
        bool needReverse = false;

        if (dir && pos >= width)
            needReverse = true;
        else if (!dir && pos <= 0.0f)
            needReverse = true;

        if (needReverse)
        {
            shared_lock();
            shared.dirForward = !dir;
            shared_unlock();
        }
    }
}