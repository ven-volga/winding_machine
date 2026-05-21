// encoder.cpp
//
// Обробка квадратурного енкодера з кнопкою через polling.
//
// Чому polling замість ISR:
//   ISR переривав I2C транзакцію до LCD посередині і давав артефакти.
//   Polling викликається з ui_task кожні 10 мс — цього достатньо
//   для надійного відстеження обертання енкодера рукою.
//   Людина не може крутити енкодер швидше ніж 10-20 імпульсів/сек,
//   тому 10 мс polling повністю достатньо.
//
// Підключення:
//   A   → ENCODER_A_PIN   (PULLUP, замикає на GND)
//   B   → ENCODER_B_PIN   (PULLUP, замикає на GND)
//   BTN → ENCODER_BTN_PIN (PULLUP, замикає на GND)
//   GND → GND
//
// Як визначається напрямок (таблиця станів):
//   Зберігаємо попередній стан {A,B} і порівнюємо з поточним.
//   Певні переходи між станами відповідають руху вправо або вліво.
//   Це надійніше ніж просто читати B при зміні A.

#include "encoder.h"
#include "pins.h"
#include "config.h"

#include "driver/gpio.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

// ─────────────────────────────────────────
// Таблиця станів квадратурного енкодера
// ─────────────────────────────────────────
//
// Кожен стан — це {A, B} у вигляді 2-бітного числа: (A<<1)|B
// Переходи між станами при обертанні:
//   Вправо: 00→01→11→10→00  (+1 за кожен крок)
//   Вліво:  00→10→11→01→00  (-1 за кожен крок)
//
// Таблиця кодує результат для кожного переходу [prev][curr]:
//  0 = немає руху або невалідний перехід
// +1 = крок вправо
// -1 = крок вліво

static const int8_t ENCODER_TABLE[4][4] = {
//curr: 00   01   10   11
      {  0,  -1,  +1,   0 },  // prev: 00
      { +1,   0,   0,  -1 },  // prev: 01
      { -1,   0,   0,  +1 },  // prev: 10
      {  0,  +1,  -1,   0 },  // prev: 11
};

// ─────────────────────────────────────────
// Внутрішні змінні
// ─────────────────────────────────────────

// Накопичений дельта від обертання
static int8_t encoderDelta = 0;

// Попередній стан {A,B}
static uint8_t lastEncState = 0;

// ── Кнопка ──────────────────────────────

typedef enum {
    BTN_IDLE,
    BTN_PRESSED,
    BTN_LONG_WAIT,
} BtnState;

static BtnState btnState        = BTN_IDLE;
static uint32_t btnPressTime    = 0;
static uint32_t btnDebounceTime = 0;
static bool     rawBtnLast      = false;

static bool eventClick     = false;
static bool eventLongPress = false;

// ─────────────────────────────────────────
// Публічні функції
// ─────────────────────────────────────────

void encoder_init()
{
    // Всі три піни: вхід з підтяжкою, без переривань
    gpio_config_t conf = {
        .pin_bit_mask = (1ULL << ENCODER_A_PIN) |
                        (1ULL << ENCODER_B_PIN)  |
                        (1ULL << ENCODER_BTN_PIN),
        .mode         = GPIO_MODE_INPUT,
        .pull_up_en   = GPIO_PULLUP_ENABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type    = GPIO_INTR_DISABLE,  // без переривань — тільки polling
    };
    gpio_config(&conf);

    // Читаємо початковий стан щоб уникнути хибного першого кроку
    uint8_t a = gpio_get_level((gpio_num_t)ENCODER_A_PIN) ? 1 : 0;
    uint8_t b = gpio_get_level((gpio_num_t)ENCODER_B_PIN) ? 1 : 0;
    lastEncState = (a << 1) | b;
}

void encoder_update()
{
    uint32_t now = (uint32_t)(xTaskGetTickCount() * portTICK_PERIOD_MS);

    // ── Обертання енкодера (таблиця станів) ─────────────────────
    uint8_t a = gpio_get_level((gpio_num_t)ENCODER_A_PIN) ? 1 : 0;
    uint8_t b = gpio_get_level((gpio_num_t)ENCODER_B_PIN) ? 1 : 0;
    uint8_t currState = (a << 1) | b;

    if (currState != lastEncState)
    {
        // Визначаємо напрямок за таблицею переходів
        int8_t step = ENCODER_TABLE[lastEncState][currState];
        encoderDelta += step;
        lastEncState = currState;
    }

    // ── Кнопка з debounce і детектором довгого натиску ──────────
    bool rawBtn = (gpio_get_level((gpio_num_t)ENCODER_BTN_PIN) == 0);

    switch (btnState)
    {
        case BTN_IDLE:
            if (rawBtn && rawBtn != rawBtnLast)
            {
                btnDebounceTime = now;
                btnState = BTN_PRESSED;
            }
            break;

        case BTN_PRESSED:
            if ((now - btnDebounceTime) >= BTN_DEBOUNCE_MS)
            {
                if (rawBtn)
                {
                    btnPressTime = now;
                    btnState = BTN_LONG_WAIT;
                }
                else
                {
                    btnState = BTN_IDLE; // шум
                }
            }
            break;

        case BTN_LONG_WAIT:
            if (!rawBtn)
            {
                // Відпустили — перевіряємо скільки тримали
                if ((now - btnPressTime) < LONG_PRESS_MS)
                    eventClick = true;      // короткий натиск
                btnState = BTN_IDLE;
            }
            else if ((now - btnPressTime) >= LONG_PRESS_MS)
            {
                eventLongPress = true;      // довгий натиск
                btnState = BTN_IDLE;
            }
            break;
    }

    rawBtnLast = rawBtn;
}

int8_t encoder_get_delta()
{
    int8_t delta = encoderDelta;
    encoderDelta = 0;
    return delta;
}

bool encoder_get_click()
{
    bool val = eventClick;
    eventClick = false;
    return val;
}

bool encoder_get_long_press()
{
    bool val = eventLongPress;
    eventLongPress = false;
    return val;
}