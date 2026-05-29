// machine.cpp
//
// autoReverse = true  → реверсує і мотає далі (нонстоп)
// autoReverse = false → досяг кінця → зупинка → чекає
//                       відпускання педалі і нового натискання

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

#define PEDAL_DEBOUNCE_MS  30

static bool     pedalState   = false;
static bool     lastRawPedal = false;
static uint32_t pedalDebTime = 0;

// Прапорець що шар завершено в пошаровому режимі.
// true = чекаємо відпускання педалі перед стартом наступного шару.
// Без цього: motion_stop() ще гальмує → running=true →
// педаль натиснута → motion_start() знову → зупинки немає.
static bool layerDone = false;

static bool pedal_read()
{
    uint32_t now = (uint32_t)(xTaskGetTickCount() * portTICK_PERIOD_MS);
    bool raw = (gpio_get_level((gpio_num_t)PEDAL_PIN) == 0);
    if (raw != lastRawPedal) { lastRawPedal = raw; pedalDebTime = now; }
    if ((now - pedalDebTime) >= PEDAL_DEBOUNCE_MS) pedalState = raw;
    return pedalState;
}

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
    if (service_active()) return;

    bool pedal = pedal_read();

    shared_lock();
    bool     running     = shared.running;
    float    pos         = shared.carriagePos;
    float    width       = shared.windingWidth;
    bool     dir         = shared.dirForward;
    uint32_t speed       = shared.spindleSpeed;
    float    dia         = shared.wireDiameter;
    bool     autoRev     = shared.autoReverse;
    uint32_t turns       = shared.turns;
    uint32_t targetTurns = shared.targetTurns;
    shared_unlock();

    // ── Якщо шар завершено — чекаємо відпускання педалі ─────────
    // Це блокує motion_start() поки педаль не відпустять
    if (layerDone)
    {
        if (!pedal)
        {
            // Педаль відпустили — готові до наступного шару
            layerDone = false;
        }
        // Поки педаль натиснута або layerDone — нічого не робимо
        return;
    }

    // ── Педаль: старт/стоп ───────────────────────────────────────
    if (pedal && !running)
    {
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
    }

    // Оновлення швидкості під час роботи
    if (running)
        motion_set_speed(speed);

    // ── Зупинка по кількості витків ──────────────────────────────
    if (running && targetTurns > 0 && turns >= targetTurns)
    {
        motion_stop();
        layerDone = true;  // чекаємо відпускання педалі

        shared_lock();
        shared.running = false;
        shared_unlock();
        return;
    }

    // ── Межа шару ────────────────────────────────────────────────
    if (running)
    {
        bool endReached = (dir && pos >= width) || (!dir && pos <= 0.0f);

        if (endReached)
        {
            if (autoRev)
            {
                // Нонстоп — реверсуємо і продовжуємо
                shared_lock();
                shared.dirForward = !dir;
                shared_unlock();
            }
            else
            {
                // Пошаровий — зупиняємо і чекаємо відпускання педалі
                motion_stop();
                layerDone = true;

                // Змінюємо напрямок для наступного шару
                shared_lock();
                shared.dirForward = !dir;
                shared.running    = false;
                shared_unlock();
            }
        }
    }
}