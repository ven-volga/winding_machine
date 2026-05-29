// encoder.cpp
//
// Квадратурний енкодер з кнопкою через polling.
//
// Підключення:
//   A   → ENCODER_A_PIN   (PULLUP)
//   B   → ENCODER_B_PIN   (PULLUP)
//   BTN → ENCODER_BTN_PIN (PULLUP, замикає на GND)
//   GND → GND
//
// Рекомендація по залізу:
//   0.1мкФ кераміка між A і GND, між B і GND —
//   прибирає більшість механічного брязкоту.

#include "encoder.h"
#include "../../include/pins.h"
#include "../../include/config.h"

extern "C" {
    #include "freertos/FreeRTOS.h"
    #include "freertos/task.h"
    #include "driver/gpio.h"
}

// ─────────────────────────────────────────
// Gray-code таблиця переходів
//
// Індекс: (prev_state << 2) | curr_state
// +1 = CW (вправо), -1 = CCW (вліво), 0 = шум
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

static uint8_t  encoderState     = 0;
static int8_t   encoderAccum     = 0;
static uint32_t lastStepMs       = 0;

// ─────────────────────────────────────────
// Стан кнопки
// ─────────────────────────────────────────

typedef enum
{
    BTN_IDLE,
    BTN_DEBOUNCE_PRESS,
    BTN_PRESSED,
    BTN_LONG_SENT,      // довгий вже відправлено, чекаємо відпускання
} BtnState;

static BtnState btnState = BTN_IDLE;
static uint32_t btnTimer = 0;

// ─────────────────────────────────────────
// Публічні змінні — результати encoder_update()
// ─────────────────────────────────────────

int8_t encDelta = 0;
bool   evClick  = false;
bool   evLong   = false;

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

    // Читаємо початковий стан щоб уникнути хибного першого кроку
    uint8_t a    = gpio_get_level((gpio_num_t)ENCODER_A_PIN);
    uint8_t b    = gpio_get_level((gpio_num_t)ENCODER_B_PIN);
    encoderState = (a << 1) | b;
}

void encoder_update()
{
    // Скидаємо події попереднього циклу
    encDelta = 0;
    evClick  = false;
    evLong   = false;

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
                encDelta  = +1;
                lastStepMs = now;
            }
            encoderAccum = 0;
        }
        else if (encoderAccum <= -4)
        {
            if ((now - lastStepMs) >= ENCODER_STEP_MS)
            {
                encDelta  = -1;
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
            if (pressed) {
                btnTimer = now;
                btnState = BTN_DEBOUNCE_PRESS;
            }
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