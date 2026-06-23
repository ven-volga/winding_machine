// machine.cpp
//
// autoReverse = true  → реверсує і мотає далі (нонстоп)
// autoReverse = false → досяг кінця → зупинка → чекає педаль
//
// SW_SPINDLE_HOLD (тумблер):
//   ON  → при зупинці шпиндель тримає момент
//   OFF → шпиндель вільний

#include "machine.h"
#include "motion.h"
#include "service.h"
#include "stepper.h"
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
static bool     layerDone    = false;

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

    // ── Чекаємо відпускання педалі після завершення шару ─────────
    if (layerDone)
    {
        if (!pedal) layerDone = false;
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

    if (running) motion_set_speed(speed);

    // ── Зупинка по кількості витків ──────────────────────────────
    if (running && targetTurns > 0 && turns >= targetTurns)
    {
        motion_stop();
        layerDone = true;

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
                shared_lock();
                shared.dirForward = !dir;
                shared_unlock();
            }
            else
            {
                motion_stop();
                layerDone = true;

                shared_lock();
                shared.dirForward = !dir;
                shared.running    = false;
                shared_unlock();
            }
        }
    }
}