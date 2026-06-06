// service.cpp
//
// Сервісний режим: ручне керування моторами.
// Активний ТІЛЬКИ коли педаль НЕ натиснута.
//
// Шпиндель CW/CCW → крутить + рахує витки
// Каретка L/R     → рухає каретку + оновлює позицію і напрямок

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

static ServiceBtn buttons[4] = {
    { (gpio_num_t)BTN_SPINDLE_CW,     false, false, 0 },
    { (gpio_num_t)BTN_SPINDLE_CCW,    false, false, 0 },
    { (gpio_num_t)BTN_CARRIAGE_LEFT,  false, false, 0 },
    { (gpio_num_t)BTN_CARRIAGE_RIGHT, false, false, 0 },
};

static bool     spindleRunning      = false;
static bool     carriageRunning     = false;
static uint32_t spindleStepCount    = 0;
static uint32_t spindleStepDelayUs  = 0;
static uint32_t carriageStepDelayUs = 0;

static const uint32_t STEPS_PER_TURN =
    (uint32_t)((float)(MOTOR_STEPS * MICROSTEPS) * SPINDLE_RATIO);

static void btn_update(ServiceBtn& btn)
{
    uint32_t now = (uint32_t)(xTaskGetTickCount() * portTICK_PERIOD_MS);
    bool raw = (gpio_get_level(btn.pin) == 0);
    if (raw != btn.lastRaw) { btn.lastRaw = raw; btn.debounceTime = now; }
    if ((now - btn.debounceTime) >= SERVICE_BTN_DEBOUNCE_MS) btn.pressed = raw;
}

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

    // Затримка шпинделя — SPINDLE_RATIO враховує передачу 1:2
    float spindleStepsPerSec =
        (float)(MOTOR_STEPS * MICROSTEPS) * SPINDLE_RATIO *
        (float)SERVICE_SPINDLE_SPEED_RPM / 60.0f;
    spindleStepDelayUs = (uint32_t)(1000000.0f / spindleStepsPerSec);

    // Затримка каретки
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

    // Кнопки заблоковані під час намотки
    if (running)
    {
        // Скидаємо serviceCarriageActive якщо намотка щойно запустилась
        shared_lock();
        shared.serviceCarriageActive = false;
        shared_unlock();
        return;
    }

    for (auto& btn : buttons) btn_update(btn);

    bool cwBtn    = buttons[0].pressed;
    bool ccwBtn   = buttons[1].pressed;
    bool leftBtn  = buttons[2].pressed;
    bool rightBtn = buttons[3].pressed;

    float stepMm = T8_LEAD_MM / (float)(MOTOR_STEPS * MICROSTEPS);

    // ── Шпиндель ─────────────────────────────────────────────────

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
            spindle_enable(false);
            spindleRunning   = false;
            spindleStepCount = 0;
        }
    }

    // ── Каретка ──────────────────────────────────────────────────

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
        shared.serviceCarriageDir    = false;  // вліво
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
        shared.serviceCarriageDir    = true;   // вправо
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
           buttons[2].pressed || buttons[3].pressed;
}