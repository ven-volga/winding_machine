// service.cpp
//
// Сервісний режим: ручне керування моторами.
//
// Активний ТІЛЬКИ коли педаль НЕ натиснута (shared.running = false).
// Під час намотки всі кнопки заблоковані.
//
// Шпиндель CW/CCW:
//   - крутить шпиндель
//   - рахує витки (додає при CW, віднімає при CCW)
//   - каретка НЕ рухається синхронно
//
// Каретка L/R:
//   - рухає каретку і оновлює carriagePos
//   - витки НЕ рахуються

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

// ─────────────────────────────────────────
// Debounce кнопок
// ─────────────────────────────────────────

#define SERVICE_BTN_DEBOUNCE_MS  20

struct ServiceBtn
{
    gpio_num_t pin;
    bool       pressed;
    bool       lastRaw;
    uint32_t   debounceTime;
};

static ServiceBtn buttons[4] = {
    { (gpio_num_t)BTN_SPINDLE_CW,     false, false, 0 },
    { (gpio_num_t)BTN_SPINDLE_CCW,    false, false, 0 },
    { (gpio_num_t)BTN_CARRIAGE_LEFT,  false, false, 0 },
    { (gpio_num_t)BTN_CARRIAGE_RIGHT, false, false, 0 },
};

// ─────────────────────────────────────────
// Стан моторів
// ─────────────────────────────────────────

static bool spindleRunning  = false;
static bool carriageRunning = false;

// Лічильник кроків шпинделя для підрахунку витків
static uint32_t spindleStepCount = 0;
static const uint32_t STEPS_PER_TURN =
    (uint32_t)((float)(MOTOR_STEPS * MICROSTEPS) * SPINDLE_RATIO);

// Затримка кроку каретки в сервісному режимі
// Фіксована повільна швидкість
static uint32_t carriageStepDelayUs = 0;

// ─────────────────────────────────────────
// Локальні функції
// ─────────────────────────────────────────

static void btn_update(ServiceBtn& btn)
{
    uint32_t now = (uint32_t)(xTaskGetTickCount() * portTICK_PERIOD_MS);
    bool raw = (gpio_get_level(btn.pin) == 0);

    if (raw != btn.lastRaw)
    {
        btn.lastRaw      = raw;
        btn.debounceTime = now;
    }

    if ((now - btn.debounceTime) >= SERVICE_BTN_DEBOUNCE_MS)
        btn.pressed = raw;
}

// ─────────────────────────────────────────
// Публічні функції
// ─────────────────────────────────────────

void service_init()
{
    gpio_config_t conf = {};
    conf.pin_bit_mask  = (1ULL << BTN_SPINDLE_CW)    |
                         (1ULL << BTN_SPINDLE_CCW)   |
                         (1ULL << BTN_CARRIAGE_LEFT) |
                         (1ULL << BTN_CARRIAGE_RIGHT);
    conf.mode          = GPIO_MODE_INPUT;
    conf.pull_up_en    = GPIO_PULLUP_ENABLE;
    conf.pull_down_en  = GPIO_PULLDOWN_DISABLE;
    conf.intr_type     = GPIO_INTR_DISABLE;
    gpio_config(&conf);

    // Затримка для каретки в сервісному режимі
    // CARRIAGE_HOME_SPEED_RPM — та сама фіксована швидкість
    float stepsPerSecond =
        (float)(MOTOR_STEPS * MICROSTEPS) *
        (float)CARRIAGE_HOME_SPEED_RPM / 60.0f;
    carriageStepDelayUs = (uint32_t)(1000000.0f / stepsPerSecond);
}

void service_update()
{
    // ── Перевіряємо чи активна намотка ──────────────────────────
    // Якщо педаль натиснута — всі кнопки заблоковані
    shared_lock();
    bool running = shared.running;
    shared_unlock();

    if (running) return;

    // Оновлюємо кнопки
    for (auto& btn : buttons) btn_update(btn);

    bool cwBtn    = buttons[0].pressed;
    bool ccwBtn   = buttons[1].pressed;
    bool leftBtn  = buttons[2].pressed;
    bool rightBtn = buttons[3].pressed;

    // Швидкість шпинделя з shared
    shared_lock();
    uint32_t speed = shared.spindleSpeed;
    shared_unlock();

    // ── Шпиндель ─────────────────────────────────────────────────

    if (cwBtn && !ccwBtn)
    {
        if (!spindleRunning)
        {
            spindle_set_dir(true);
            spindle_enable(true);
            motion_set_speed(speed);
            spindleRunning  = true;
            spindleStepCount = 0;
        }

        // Крок шпинделя
        spindle_step();
        spindleStepCount++;

        // Рахуємо витки
        if (spindleStepCount >= STEPS_PER_TURN)
        {
            spindleStepCount = 0;
            shared_lock();
            shared.turns++;
            shared_unlock();
        }

        motion_set_speed(speed); // оновлюємо швидкість з енкодера
    }
    else if (ccwBtn && !cwBtn)
    {
        if (!spindleRunning)
        {
            spindle_set_dir(false);
            spindle_enable(true);
            motion_set_speed(speed);
            spindleRunning   = true;
            spindleStepCount = 0;
        }

        spindle_step();
        spindleStepCount++;

        // Відраховуємо витки назад
        if (spindleStepCount >= STEPS_PER_TURN)
        {
            spindleStepCount = 0;
            shared_lock();
            if (shared.turns > 0) shared.turns--;
            shared_unlock();
        }

        motion_set_speed(speed);
    }
    else
    {
        if (spindleRunning)
        {
            spindle_enable(false);
            spindleRunning   = false;
            spindleStepCount = 0;
        }
    }

    // ── Каретка ──────────────────────────────────────────────────

    float stepMm = T8_LEAD_MM / (float)(MOTOR_STEPS * MICROSTEPS);

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
        shared.carriagePos -= stepMm;
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
        shared.carriagePos += stepMm;
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
    }
}

bool service_active()
{
    // Активний тільки якщо кнопка натиснута І намотка не йде
    shared_lock();
    bool running = shared.running;
    shared_unlock();

    if (running) return false;

    return buttons[0].pressed || buttons[1].pressed ||
           buttons[2].pressed || buttons[3].pressed;
}