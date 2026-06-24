// service.cpp
//
// Сервісний режим: ручне керування моторами.
// Активний ТІЛЬКИ коли педаль НЕ натиснута.
//
// Шпиндель CW/CCW → крутить + рахує витки
// Каретка L/R     → рухає каретку + оновлює позицію і напрямок
//
// Сервісна намотка (BTN_SERVICE_WIND):
//   Тримаєш → мотає в напрямку з UI (dirForward) БЕЗ рахунку витків
//   Каретка може виходити за межі 0 і windingWidth — без обмежень,
//   без реверсу. Потрібно для пошуку точної точки старту на
//   нерівному каркасі (виставляєш каретку, мотаєш повільно,
//   дивишся візуально де дріт ляже на щоку каркаса).
//
// Утримання шпинделя (SW_SPINDLE_HOLD) застосовується тут так само
// як в machine.cpp — коли сервісні кнопки не натиснуті.

#include "service.h"
#include "stepper.h"
#include "motion.h"
#include "shared_state.h"
#include "pins.h"
#include "config.h"

#include <cstdint>

extern "C" {
    #include "freertos/FreeRTOS.h"
    #include "freertos/task.h"
    #include "driver/gpio.h"
    #include "esp_rom_sys.h"
}

#define SERVICE_BTN_DEBOUNCE_MS  20

struct ServiceBtn
{
    gpio_num_t pin;
    bool       pressed;
    bool       lastRaw;
    uint32_t   debounceTime;
};

// 4 кнопки керування + кнопка сервісної намотки
static ServiceBtn buttons[5] = {
    { (gpio_num_t)BTN_SPINDLE_CW,     false, false, 0 },
    { (gpio_num_t)BTN_SPINDLE_CCW,    false, false, 0 },
    { (gpio_num_t)BTN_CARRIAGE_LEFT,  false, false, 0 },
    { (gpio_num_t)BTN_CARRIAGE_RIGHT, false, false, 0 },
    { (gpio_num_t)BTN_SERVICE_WIND,   false, false, 0 },
};

static bool     spindleRunning      = false;
static bool     carriageRunning     = false;
static uint32_t spindleStepCount    = 0;
static uint32_t spindleStepDelayUs  = 0;
static uint32_t carriageStepDelayUs = 0;

// Стан сервісної намотки
static bool     serviceWindActive   = false;
static uint32_t serviceWindStepCount = 0;

static const uint32_t STEPS_PER_TURN =
    (uint32_t)((float)(MOTOR_STEPS * MICROSTEPS) * SPINDLE_RATIO);

static void btn_update(ServiceBtn& btn)
{
    uint32_t now = (uint32_t)(xTaskGetTickCount() * portTICK_PERIOD_MS);
    bool raw = (gpio_get_level(btn.pin) == 0);
    if (raw != btn.lastRaw) { btn.lastRaw = raw; btn.debounceTime = now; }
    if ((now - btn.debounceTime) >= SERVICE_BTN_DEBOUNCE_MS) btn.pressed = raw;
}

// Читає тумблер утримання шпинделя
static bool spindle_hold_read()
{
    return (gpio_get_level((gpio_num_t)SW_SPINDLE_HOLD) == 0);
}

void service_init()
{
    gpio_config_t conf = {};
    conf.pin_bit_mask  = (1ULL << BTN_SPINDLE_CW)    |
                         (1ULL << BTN_SPINDLE_CCW)   |
                         (1ULL << BTN_CARRIAGE_LEFT) |
                         (1ULL << BTN_CARRIAGE_RIGHT)|
                         (1ULL << BTN_SERVICE_WIND)  |
                         (1ULL << SW_SPINDLE_HOLD);
    conf.mode          = GPIO_MODE_INPUT;
    conf.pull_up_en    = GPIO_PULLUP_ENABLE;
    conf.pull_down_en  = GPIO_PULLDOWN_DISABLE;
    conf.intr_type     = GPIO_INTR_DISABLE;
    gpio_config(&conf);

    float spindleStepsPerSec =
        (float)(MOTOR_STEPS * MICROSTEPS) * SPINDLE_RATIO *
        (float)SERVICE_SPINDLE_SPEED_RPM / 60.0f;
    spindleStepDelayUs = (uint32_t)(1000000.0f / spindleStepsPerSec);

    float carriageStepsPerSec =
        (float)(MOTOR_STEPS * MICROSTEPS) *
        (float)SERVICE_CARRIAGE_SPEED_RPM / 60.0f;
    carriageStepDelayUs = (uint32_t)(1000000.0f / carriageStepsPerSec);
}

void service_update()
{
    shared_lock();
    bool running = shared.running;
    shared_unlock();

    bool hold = spindle_hold_read();

    // Оновлюємо shared.spindleHold — motion.cpp читає це при зупинці
    shared_lock();
    shared.spindleHold = hold;
    shared_unlock();

    if (running)
    {
        shared_lock();
        shared.serviceCarriageActive = false;
        shared_unlock();
        return;
    }

    for (auto& btn : buttons) btn_update(btn);

    bool cwBtn      = buttons[0].pressed;
    bool ccwBtn     = buttons[1].pressed;
    bool leftBtn    = buttons[2].pressed;
    bool rightBtn   = buttons[3].pressed;
    bool serviceBtn = buttons[4].pressed;

    float stepMm = T8_LEAD_MM / (float)(MOTOR_STEPS * MICROSTEPS);

    // ── Сервісна намотка ─────────────────────────────────────────
    // Має пріоритет над звичайними кнопками — якщо натиснута,
    // решта сервісних кнопок ігноруються
    if (serviceBtn)
    {
        if (!serviceWindActive)
        {
            shared_lock();
            bool dir = shared.dirForward;
            shared_unlock();

            spindle_set_dir(true);   // шпиндель завжди CW
            spindle_enable(true);
            carriage_set_dir(dir);
            carriage_enable(true);

            serviceWindActive    = true;
            serviceWindStepCount = 0;

            shared_lock();
            shared.serviceCarriageActive = true;
            shared.serviceCarriageDir    = dir;
            shared_unlock();
        }

        shared_lock();
        bool dir = shared.dirForward;
        shared_unlock();

        // Крок шпинделя
        spindle_step();
        esp_rom_delay_us(spindleStepDelayUs);

        // Bresenham-подібна синхронізація каретки —
        // спрощена через STEPS_PER_TURN, без обмежень по межах
        serviceWindStepCount++;

        shared_lock();
        float diameter = shared.wireDiameter;
        shared_unlock();

        float spindleStepsPerTurn  = (float)(MOTOR_STEPS * MICROSTEPS) * SPINDLE_RATIO;
        float carriageStepsPerTurn = (diameter / T8_LEAD_MM) * (float)(MOTOR_STEPS * MICROSTEPS);
        float ratio = carriageStepsPerTurn / spindleStepsPerTurn;

        static float accum = 0.0f;
        accum += ratio;
        if (accum >= 1.0f)
        {
            accum -= 1.0f;
            carriage_step();

            shared_lock();
            shared.carriagePos += dir ? stepMm : -stepMm;
            shared.serviceCarriageDir = dir;
            shared_unlock();
        }

        // НЕ рахуємо витки, НЕ обмежуємо позицію каретки
        return;
    }
    else if (serviceWindActive)
    {
        // Відпустили кнопку сервісної намотки
        // spindle_enable встановиться в блоці else нижче
        carriage_enable(false);
        serviceWindActive = false;

        shared_lock();
        shared.serviceCarriageActive = false;
        shared_unlock();
    }

    // ── Шпиндель CW/CCW ──────────────────────────────────────────

    if (cwBtn && !ccwBtn)
    {
        if (!spindleRunning)
        {
            spindle_set_dir(true);
            spindle_enable(true);
            spindleRunning   = true;
            spindleStepCount = 0;
        }
        spindle_step();
        esp_rom_delay_us(spindleStepDelayUs);
        spindleStepCount++;
        if (spindleStepCount >= STEPS_PER_TURN)
        {
            spindleStepCount = 0;
            shared_lock();
            shared.turns++;
            shared_unlock();
        }
    }
    else if (ccwBtn && !cwBtn)
    {
        if (!spindleRunning)
        {
            spindle_set_dir(false);
            spindle_enable(true);
            spindleRunning   = true;
            spindleStepCount = 0;
        }
        spindle_step();
        esp_rom_delay_us(spindleStepDelayUs);
        spindleStepCount++;
        if (spindleStepCount >= STEPS_PER_TURN)
        {
            spindleStepCount = 0;
            shared_lock();
            if (shared.turns > 0) shared.turns--;
            shared_unlock();
        }
    }
    else
    {
        if (spindleRunning)
        {
            spindleRunning   = false;
            spindleStepCount = 0;
        }

        // Утримання перевіряємо КОЖЕН ЦИКЛ поки шпиндель не крутиться.
        // Не тільки в момент зупинки — це гарантує актуальний стан тумблера.
        if (!serviceWindActive)
            spindle_enable(hold);
    }

    // ── Каретка L/R ──────────────────────────────────────────────

    if (leftBtn && !rightBtn)
    {
        if (!carriageRunning)
        {
            carriage_set_dir(false);
            carriage_enable(true);
            carriageRunning = true;
        }
        carriage_step();

        shared_lock();
        shared.carriagePos          -= stepMm;
        shared.serviceCarriageDir    = false;
        shared.serviceCarriageActive = true;
        shared_unlock();

        esp_rom_delay_us(carriageStepDelayUs);
    }
    else if (rightBtn && !leftBtn)
    {
        if (!carriageRunning)
        {
            carriage_set_dir(true);
            carriage_enable(true);
            carriageRunning = true;
        }
        carriage_step();

        shared_lock();
        shared.carriagePos          += stepMm;
        shared.serviceCarriageDir    = true;
        shared.serviceCarriageActive = true;
        shared_unlock();

        esp_rom_delay_us(carriageStepDelayUs);
    }
    else
    {
        if (carriageRunning)
        {
            carriage_enable(false);
            carriageRunning = false;
        }
        shared_lock();
        shared.serviceCarriageActive = false;
        shared_unlock();
    }
}

bool service_active()
{
    shared_lock();
    bool running = shared.running;
    shared_unlock();
    if (running) return false;
    return buttons[0].pressed || buttons[1].pressed ||
           buttons[2].pressed || buttons[3].pressed ||
           buttons[4].pressed;
}