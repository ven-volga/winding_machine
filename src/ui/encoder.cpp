// encoder.cpp
//
// Обробка квадратурного енкодера з кнопкою.
// Використовує переривання GPIO для точного відстеження обертання.
//
// Підключення:
//   A   → ENCODER_A_PIN  (з підтяжкою PULLUP)
//   B   → ENCODER_B_PIN  (з підтяжкою PULLUP)
//   BTN → ENCODER_BTN_PIN (з підтяжкою PULLUP, замикає на GND)
//   GND → GND
//
// Як працює квадратурний енкодер:
//   При обертанні A і B генерують імпульси зі зсувом 90°.
//   За зміною A читаємо стан B — це визначає напрямок.
//   A змінився, B=0 → вправо (+1)
//   A змінився, B=1 → вліво (-1)

#include "encoder.h"
#include "pins.h"

#include "driver/gpio.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_attr.h"

// ─────────────────────────────────────────
// Внутрішні змінні
// ─────────────────────────────────────────

// Накопичений дельта від обертання (volatile бо змінюється в ISR)
static volatile int8_t encoderDelta = 0;

// ── Кнопка ──────────────────────────────

// Debounce для кнопки (мс)
#define BTN_DEBOUNCE_MS  30

// Стани кнопки
typedef enum {
    BTN_IDLE,       // не натиснута
    BTN_PRESSED,    // натиснута, чекаємо відпускання
    BTN_LONG_WAIT,  // чекаємо довгого натиску
} BtnState;

static BtnState btnState       = BTN_IDLE;
static uint32_t btnPressTime   = 0;    // час натискання (мс)
static bool     rawBtnLast     = false;
static uint32_t btnDebounceTime = 0;

// Прапорці подій — скидаються після читання
static volatile bool eventClick     = false;
static volatile bool eventLongPress = false;

// ─────────────────────────────────────────
// ISR — переривання на зміну сигналу A
// ─────────────────────────────────────────

// IRAM_ATTR — функція має жити в IRAM (швидка пам'ять)
// бо викликається з переривання
static void IRAM_ATTR encoder_isr(void* arg)
{
    // Читаємо стан B в момент зміни A
    int b = gpio_get_level((gpio_num_t)ENCODER_B_PIN);

    if (b == 0)
        encoderDelta = encoderDelta + 1;  // вправо
    else
        encoderDelta = encoderDelta - 1;  // вліво
}

// ─────────────────────────────────────────
// Публічні функції
// ─────────────────────────────────────────

void encoder_init()
{
    // Налаштування піну A — вхід з підтяжкою, переривання на будь-яку зміну
    gpio_config_t enc_conf = {
        .pin_bit_mask = (1ULL << ENCODER_A_PIN) | (1ULL << ENCODER_B_PIN),
        .mode         = GPIO_MODE_INPUT,
        .pull_up_en   = GPIO_PULLUP_ENABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type    = GPIO_INTR_DISABLE,
    };
    gpio_config(&enc_conf);

    // Налаштування кнопки
    gpio_config_t btn_conf = {
        .pin_bit_mask = (1ULL << ENCODER_BTN_PIN),
        .mode         = GPIO_MODE_INPUT,
        .pull_up_en   = GPIO_PULLUP_ENABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type    = GPIO_INTR_DISABLE,
    };
    gpio_config(&btn_conf);

    // Переривання тільки на пін A (визначає напрямок через B)
    gpio_set_intr_type((gpio_num_t)ENCODER_A_PIN, GPIO_INTR_ANYEDGE);
    gpio_install_isr_service(0);
    gpio_isr_handler_add((gpio_num_t)ENCODER_A_PIN, encoder_isr, NULL);
}

void encoder_update()
{
    uint32_t now = (uint32_t)(xTaskGetTickCount() * portTICK_PERIOD_MS);

    // ── Обробка кнопки ───────────────────────────────────────────
    // LOW = натиснута (PULLUP + замикання на GND)
    bool rawBtn = (gpio_get_level((gpio_num_t)ENCODER_BTN_PIN) == 0);

    switch (btnState)
    {
        case BTN_IDLE:
            if (rawBtn && rawBtn != rawBtnLast)
            {
                // Щойно натиснули — запускаємо debounce
                btnDebounceTime = now;
                btnState = BTN_PRESSED;
            }
            break;

        case BTN_PRESSED:
            if ((now - btnDebounceTime) >= BTN_DEBOUNCE_MS)
            {
                if (rawBtn)
                {
                    // Підтверджено натискання — запам'ятовуємо час
                    btnPressTime = now;
                    btnState = BTN_LONG_WAIT;
                }
                else
                {
                    // Відпустили за час debounce — шум, ігноруємо
                    btnState = BTN_IDLE;
                }
            }
            break;

        case BTN_LONG_WAIT:
            if (!rawBtn)
            {
                // Відпустили — короткий натиск
                uint32_t held = now - btnPressTime;
                if (held < LONG_PRESS_MS)
                    eventClick = true;
                // Якщо held >= LONG_PRESS_MS — довгий вже спрацював нижче
                btnState = BTN_IDLE;
            }
            else if ((now - btnPressTime) >= LONG_PRESS_MS)
            {
                // Тримають довше LONG_PRESS_MS — довгий натиск
                eventLongPress = true;
                // Чекаємо поки відпустять (переходимо в IDLE при відпусканні)
                btnState = BTN_IDLE;
            }
            break;
    }

    rawBtnLast = rawBtn;
}

int8_t encoder_get_delta()
{
    // Атомарно читаємо і скидаємо.
    // spinlock захищає від одночасного доступу з ISR.
    // Мux має бути статичною змінною — не можна брати адресу тимчасового об'єкту.
    static portMUX_TYPE mux = portMUX_INITIALIZER_UNLOCKED;
    portENTER_CRITICAL(&mux);
    int8_t delta = encoderDelta;
    encoderDelta = 0;
    portEXIT_CRITICAL(&mux);
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