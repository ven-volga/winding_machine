// stepper.cpp
//
// Реальне керування кроковиками через TMC2209.
//
// Підключення:
//   SPINDLE:  STEP=25, DIR=26, EN=27
//   CARRIAGE: STEP=14, DIR=12, EN=13
//
// TMC2209 особливості:
//   EN активний LOW — gpio HIGH = драйвер вимкнено
//                     gpio LOW  = драйвер увімкнено
//
//   STEP імпульс мінімум 1мкс HIGH + 1мкс LOW (datasheet).
//   Ми використовуємо 2мкс + 2мкс — з запасом.
//
//   DIR має бути стабільним мінімум 20нс до STEP імпульсу.
//   В нашому коді DIR встановлюється окремо перед кроками —
//   затримка між set_dir і step достатня.
//
// Важливо при першому запуску:
//   Перевір напрямок обертання шпинделя.
//   Якщо крутиться не в той бік — поміняй місцями
//   два дроти однієї обмотки мотора АБО інвертуй логіку:
//   #define SPINDLE_DIR_INVERT  1  (додати в config.h)

#include "stepper.h"
#include "pins.h"

extern "C" {
    #include "driver/gpio.h"
    #include "esp_rom_sys.h"
}

// ─────────────────────────────────────────
// Поточний стан
// ─────────────────────────────────────────

static bool spindleDir  = true;
static bool carriageDir = true;

// ─────────────────────────────────────────
// stepper_init
// ─────────────────────────────────────────

void stepper_init()
{
    // Налаштовуємо всі 6 пінів як виходи
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

    // EN HIGH = драйвер вимкнено (активний LOW)
    // Вмикаємо тільки коли треба рухатись
    gpio_set_level((gpio_num_t)SPINDLE_EN_PIN,  1);
    gpio_set_level((gpio_num_t)CARRIAGE_EN_PIN, 1);

    // STEP LOW в спокої
    gpio_set_level((gpio_num_t)SPINDLE_STEP_PIN,  0);
    gpio_set_level((gpio_num_t)CARRIAGE_STEP_PIN, 0);

    // Початковий напрямок — вперед
    gpio_set_level((gpio_num_t)SPINDLE_DIR_PIN,  1);
    gpio_set_level((gpio_num_t)CARRIAGE_DIR_PIN, 1);
}

// ─────────────────────────────────────────
// Шпиндель
// ─────────────────────────────────────────

void spindle_set_dir(bool forward)
{
    spindleDir = forward;
    gpio_set_level((gpio_num_t)SPINDLE_DIR_PIN, forward ? 1 : 0);
}

void spindle_enable(bool en)
{
    // EN активний LOW: en=true → LOW (увімкнено)
    //                  en=false → HIGH (вимкнено)
    gpio_set_level((gpio_num_t)SPINDLE_EN_PIN, en ? 0 : 1);
}

void spindle_step()
{
    // STEP імпульс: HIGH 2мкс → LOW 2мкс
    gpio_set_level((gpio_num_t)SPINDLE_STEP_PIN, 1);
    esp_rom_delay_us(2);
    gpio_set_level((gpio_num_t)SPINDLE_STEP_PIN, 0);
    esp_rom_delay_us(2);
}

// ─────────────────────────────────────────
// Каретка
// ─────────────────────────────────────────

void carriage_set_dir(bool forward)
{
    carriageDir = forward;
    gpio_set_level((gpio_num_t)CARRIAGE_DIR_PIN, forward ? 1 : 0);
}

void carriage_enable(bool en)
{
    gpio_set_level((gpio_num_t)CARRIAGE_EN_PIN, en ? 0 : 1);
}

void carriage_step()
{
    gpio_set_level((gpio_num_t)CARRIAGE_STEP_PIN, 1);
    esp_rom_delay_us(2);
    gpio_set_level((gpio_num_t)CARRIAGE_STEP_PIN, 0);
    esp_rom_delay_us(2);
}