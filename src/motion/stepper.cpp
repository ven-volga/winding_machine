// stepper.cpp
//
// Низькорівневе керування кроковиками.
//
// ═══════════════════════════════════════════════════════════
// ЗАРАЗ: заглушка
//   GPIO не чіпаємо — логуємо кроки в Serial.
//   Синхронізацію і логіку можна тестувати без заліза.
//
// ПІСЛЯ ПРИХОДУ TMC2209:
//   Замінити тіла spindle_step() і carriage_step()
//   на реальні GPIO виклики (розкоментувати блоки нижче).
//   stepper_init() — розкоментувати gpio_config.
//   Все інше залишається без змін.
// ═══════════════════════════════════════════════════════════

#include "stepper.h"
#include "pins.h"

#include <cstdio>

extern "C" {
    #include "driver/gpio.h"
    #include "esp_rom_sys.h"
}

// ─────────────────────────────────────────
// Поточний стан драйверів
// ─────────────────────────────────────────

static bool spindleDir  = true;   // true = CW
static bool carriageDir = true;   // true = вперед

// Лічильники для Serial логу
static uint32_t spindleSteps  = 0;
static uint32_t carriageSteps = 0;

// ─────────────────────────────────────────
// stepper_init
// ─────────────────────────────────────────

void stepper_init()
{
    // ── ЗАГЛУШКА ────────────────────────────────────────────────
    printf("[STEPPER] init (stub mode — no real GPIO)\n");

    // ── РЕАЛЬНИЙ КОД (розкоментувати після TMC2209) ─────────────
    /*
    uint64_t pins =
        (1ULL << SPINDLE_STEP_PIN)  |
        (1ULL << SPINDLE_DIR_PIN)   |
        (1ULL << SPINDLE_EN_PIN)    |
        (1ULL << CARRIAGE_STEP_PIN) |
        (1ULL << CARRIAGE_DIR_PIN)  |
        (1ULL << CARRIAGE_EN_PIN);

    gpio_config_t conf = {};
    conf.pin_bit_mask  = pins;
    conf.mode          = GPIO_MODE_OUTPUT;
    conf.pull_up_en    = GPIO_PULLUP_DISABLE;
    conf.pull_down_en  = GPIO_PULLDOWN_DISABLE;
    conf.intr_type     = GPIO_INTR_DISABLE;
    gpio_config(&conf);

    // EN активний LOW — вимикаємо до старту
    gpio_set_level((gpio_num_t)SPINDLE_EN_PIN,  1);
    gpio_set_level((gpio_num_t)CARRIAGE_EN_PIN, 1);

    // Початковий напрямок
    gpio_set_level((gpio_num_t)SPINDLE_DIR_PIN,  1);
    gpio_set_level((gpio_num_t)CARRIAGE_DIR_PIN, 1);
    */
}

// ─────────────────────────────────────────
// Шпиндель
// ─────────────────────────────────────────

void spindle_set_dir(bool forward)
{
    spindleDir = forward;

    // ── РЕАЛЬНИЙ КОД ────────────────────────────────────────────
    // gpio_set_level((gpio_num_t)SPINDLE_DIR_PIN, forward ? 1 : 0);
}

void spindle_enable(bool en)
{
    // EN активний LOW на TMC2209
    // ── РЕАЛЬНИЙ КОД ────────────────────────────────────────────
    // gpio_set_level((gpio_num_t)SPINDLE_EN_PIN, en ? 0 : 1);

    printf("[STEPPER] spindle %s\n", en ? "ENABLED" : "DISABLED");
}

void spindle_step()
{
    spindleSteps++;

    // ── ЗАГЛУШКА — логуємо кожні 200 кроків ─────────────────────
    if (spindleSteps % 200 == 0)
    {
        printf("[SPINDLE] steps=%lu dir=%s\n",
               (unsigned long)spindleSteps,
               spindleDir ? "CW" : "CCW");
    }

    // ── РЕАЛЬНИЙ КОД ────────────────────────────────────────────
    // STEP імпульс: HIGH → затримка → LOW
    // gpio_set_level((gpio_num_t)SPINDLE_STEP_PIN, 1);
    // esp_rom_delay_us(2);   // мінімум 1мкс по datasheet TMC2209
    // gpio_set_level((gpio_num_t)SPINDLE_STEP_PIN, 0);
    // esp_rom_delay_us(2);
}

// ─────────────────────────────────────────
// Каретка укладчика
// ─────────────────────────────────────────

void carriage_set_dir(bool forward)
{
    carriageDir = forward;

    // ── РЕАЛЬНИЙ КОД ────────────────────────────────────────────
    // gpio_set_level((gpio_num_t)CARRIAGE_DIR_PIN, forward ? 1 : 0);
}

void carriage_enable(bool en)
{
    // ── РЕАЛЬНИЙ КОД ────────────────────────────────────────────
    // gpio_set_level((gpio_num_t)CARRIAGE_EN_PIN, en ? 0 : 1);

    printf("[STEPPER] carriage %s\n", en ? "ENABLED" : "DISABLED");
}

void carriage_step()
{
    carriageSteps++;

    // ── ЗАГЛУШКА — логуємо кожні 200 кроків ─────────────────────
    if (carriageSteps % 200 == 0)
    {
        printf("[CARRIAGE] steps=%lu dir=%s\n",
               (unsigned long)carriageSteps,
               carriageDir ? "FWD" : "REV");
    }

    // ── РЕАЛЬНИЙ КОД ────────────────────────────────────────────
    // gpio_set_level((gpio_num_t)CARRIAGE_STEP_PIN, 1);
    // esp_rom_delay_us(2);
    // gpio_set_level((gpio_num_t)CARRIAGE_STEP_PIN, 0);
    // esp_rom_delay_us(2);
}