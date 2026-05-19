// machine.cpp
//
// Логіка верхнього рівня машини:
// - ініціалізація педалі
// - debounce педалі
// - старт/стоп руху
//
// machine_update() викликається з RTOS задачі на ядрі 0 (main.cpp).

#include "machine.h"
#include "machine_state.h"
#include "motion/motion.h"
#include "pins.h"

#include "driver/gpio.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

// ─────────────────────────────────────────
// Налаштування debounce
// ─────────────────────────────────────────

// Скільки мілісекунд сигнал має бути стабільним
// щоб ми його прийняли як реальний стан педалі.
// 30 мс — достатньо для механічного контакту.
#define DEBOUNCE_MS 30

// Поточний підтверджений стан педалі
static bool pedalState = false;

// Час (мс) коли педаль востаннє змінила стан
static uint32_t lastChangeTime = 0;

// Сирий стан педалі на попередній ітерації
static bool lastRawState = false;

// ─────────────────────────────────────────
// machine_init
// ─────────────────────────────────────────

void machine_init()
{
    // Налаштовуємо GPIO педалі:
    // - вхід
    // - внутрішня підтяжка до VCC (PULLUP)
    //
    // Це означає що коли педаль НЕ натиснута — на піні HIGH.
    // Коли педаль натиснута (замикає на GND) — на піні LOW.
    // Тобто логіка інвертована: LOW = натиснута.

    gpio_config_t pedal_conf = {
        .pin_bit_mask = (1ULL << PEDAL_PIN),
        .mode         = GPIO_MODE_INPUT,
        .pull_up_en   = GPIO_PULLUP_ENABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type    = GPIO_INTR_DISABLE,
    };
    gpio_config(&pedal_conf);
}

// ─────────────────────────────────────────
// pedal_debounced
//
// Повертає true якщо педаль натиснута (після debounce).
// Викликати кожну ітерацію machine_update().
// ─────────────────────────────────────────

static bool pedal_debounced()
{
    // Читаємо сирий сигнал.
    // LOW = натиснута (бо PULLUP + замикання на GND).
    bool raw = (gpio_get_level((gpio_num_t)PEDAL_PIN) == 0);

    uint32_t now = (uint32_t)(xTaskGetTickCount() * portTICK_PERIOD_MS);

    if (raw != lastRawState)
    {
        // Сигнал змінився — запам'ятовуємо час зміни
        lastRawState = raw;
        lastChangeTime = now;
    }

    // Якщо сигнал стабільний довше DEBOUNCE_MS — приймаємо його
    if ((now - lastChangeTime) >= DEBOUNCE_MS)
    {
        pedalState = raw;
    }

    return pedalState;
}

// ─────────────────────────────────────────
// machine_update
//
// Викликається кожні ~10 мс з RTOS задачі.
// ─────────────────────────────────────────

void machine_update()
{
    bool pressed = pedal_debounced();

    if (pressed)
    {
        // Педаль затиснута — запускаємо рух
        motion_start();
    }
    else
    {
        // Педаль відпущена — зупиняємо
        motion_stop();
    }

    // TODO (наступні кроки):
    // - автоматичний реверс каретки при досягненні windingWidth
    // - лічильник витків
    // - homing
    // - обробка кінцевиків
}