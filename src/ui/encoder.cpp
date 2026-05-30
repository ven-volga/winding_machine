// encoder.cpp
//
// Квадратурний енкодер з кнопкою через polling.
//
// Архітектура:
//   encoder_update() викликається з esp_timer кожну 1мс.
//   Функція ТІЛЬКИ читає стан пінів і накопичує кроки.
//   Ніякої бізнес-логіки, ніяких mutex, ніяких shared.
//
//   UI цикл (ui_update) читає накопичені кроки і події кнопки,
//   потім вирішує що з ними робити.

#include "encoder.h"
#include "pins.h"
#include "config.h"

extern "C" {
    #include "freertos/FreeRTOS.h"
    #include "freertos/task.h"
    #include "driver/gpio.h"
}

// ─────────────────────────────────────────
// Gray-code таблиця переходів
// Індекс: (prev_state << 2) | curr_state
// +1 = CW, -1 = CCW, 0 = шум
// ─────────────────────────────────────────

static const int8_t ENCODER_TABLE[16] =
{
     0, -1, +1,  0,
    +1,  0,  0, -1,
    -1,  0,  0, +1,
     0, +1, -1,  0
};

// ─────────────────────────────────────────
// Стан енкодера
// ─────────────────────────────────────────

static uint8_t  encoderState = 0;
static int8_t   encoderAccum = 0;   // накопичувач до порогу 4
static uint32_t lastStepMs   = 0;

// ─────────────────────────────────────────
// Публічні змінні
//
// encSteps — volatile int32_t накопичувач.
//   Таймер пише, UI читає і скидає.
//   Використовуємо int32_t щоб не губити кроки
//   при швидкому крутінні.
// ─────────────────────────────────────────

volatile int32_t encSteps = 0;
bool             evClick  = false;
bool             evLong   = false;

// ─────────────────────────────────────────
// Стан кнопки
// ─────────────────────────────────────────

typedef enum
{
    BTN_IDLE,
    BTN_DEBOUNCE_PRESS,
    BTN_PRESSED,
    BTN_LONG_SENT,
} BtnState;

static BtnState btnState = BTN_IDLE;
static uint32_t btnTimer = 0;

// ─────────────────────────────────────────
// Публічні функції
// ─────────────────────────────────────────

void encoder_init()
{
    gpio_config_t conf = {};
    conf.pin_bit_mask  = (1ULL << ENCODER_A_PIN)   |
                         (1ULL << ENCODER_B_PIN)    |
                         (1ULL << ENCODER_BTN_PIN);
    conf.mode          = GPIO_MODE_INPUT;
    conf.pull_up_en    = GPIO_PULLUP_ENABLE;
    conf.pull_down_en  = GPIO_PULLDOWN_DISABLE;
    conf.intr_type     = GPIO_INTR_DISABLE;
    gpio_config(&conf);

    uint8_t a    = gpio_get_level((gpio_num_t)ENCODER_A_PIN);
    uint8_t b    = gpio_get_level((gpio_num_t)ENCODER_B_PIN);
    encoderState = (a << 1) | b;
}

void encoder_update()
{
    uint32_t now = (uint32_t)(xTaskGetTickCount() * portTICK_PERIOD_MS);

    // ── Обертання (Gray-code) ────────────────────────────────────
    uint8_t a    = gpio_get_level((gpio_num_t)ENCODER_A_PIN);
    uint8_t b    = gpio_get_level((gpio_num_t)ENCODER_B_PIN);
    uint8_t curr = (a << 1) | b;
    uint8_t idx  = (encoderState << 2) | curr;
    int8_t  step = ENCODER_TABLE[idx];

    if (step != 0)
    {
        encoderAccum += step;
        encoderState  = curr;

        // Повний клік = 4 переходи
        if (encoderAccum >= 4)
        {
            if ((now - lastStepMs) >= ENCODER_STEP_MS)
            {
                // Атомарне накопичення — просто +=
                // UI прочитає і скине коли буде готовий
                encSteps = encSteps + 1;
                lastStepMs = now;
            }
            encoderAccum = 0;
        }
        else if (encoderAccum <= -4)
        {
            if ((now - lastStepMs) >= ENCODER_STEP_MS)
            {
                encSteps = encSteps - 1;
                lastStepMs = now;
            }
            encoderAccum = 0;
        }
    }
    else
    {
        encoderState = curr;
    }

    // ── Кнопка ───────────────────────────────────────────────────
    bool pressed = (gpio_get_level((gpio_num_t)ENCODER_BTN_PIN) == 0);

    switch (btnState)
    {
        case BTN_IDLE:
            if (pressed) { btnTimer = now; btnState = BTN_DEBOUNCE_PRESS; }
            break;

        case BTN_DEBOUNCE_PRESS:
            if ((now - btnTimer) >= BTN_DEBOUNCE_MS) {
                if (pressed) { btnTimer = now; btnState = BTN_PRESSED; }
                else         { btnState = BTN_IDLE; }
            }
            break;

        case BTN_PRESSED:
            if (!pressed) {
                evClick  = true;
                btnState = BTN_IDLE;
            } else if ((now - btnTimer) >= LONG_PRESS_MS) {
                evLong   = true;
                btnState = BTN_LONG_SENT;
            }
            break;

        case BTN_LONG_SENT:
            if (!pressed) btnState = BTN_IDLE;
            break;
    }
}