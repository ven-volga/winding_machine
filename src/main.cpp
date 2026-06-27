// main.cpp
//
// Ядро 0 — motion_task: service, machine, motion
// Ядро 1 — ui_task:     LCD + енкодер
//
// АВАРІЙНА ЗУПИНКА (ESTOP_PIN):
//   Перевіряється на початку кожної ітерації motion_task.
//   При спрацюванні — миттєва зупинка всіх моторів без гальмування.
//   Працює і в сервісному режимі і під час намотки.
//   Після відпускання ESTOP — система готова до роботи знову.

#include "ui.h"
#include "motion.h"
#include "machine.h"
#include "hall.h"
#include "service.h"
#include "stepper.h"
#include "shared_state.h"
#include "pins.h"

extern "C" {
    #include "freertos/FreeRTOS.h"
    #include "freertos/task.h"
    #include "esp_task_wdt.h"
    #include "driver/gpio.h"
}

// ─────────────────────────────────────────
// UI задача — ядро 1
// ─────────────────────────────────────────

static void ui_task(void*)
{
    ui_init();

    while (true)
    {
        ui_update();
        vTaskDelay(pdMS_TO_TICKS(5));
    }
}

// ─────────────────────────────────────────
// Motion задача — ядро 0
// ─────────────────────────────────────────

static bool estopActive = false;

static void motion_task(void*)
{
    // Ініціалізація ESTOP піна
    gpio_config_t conf = {};
    conf.pin_bit_mask  = (1ULL << ESTOP_PIN);
    conf.mode          = GPIO_MODE_INPUT;
    conf.pull_up_en    = GPIO_PULLUP_ENABLE;
    conf.pull_down_en  = GPIO_PULLDOWN_DISABLE;
    conf.intr_type     = GPIO_INTR_DISABLE;
    gpio_config(&conf);

    hall_init();
    service_init();
    machine_init();
    motion_init();

    while (true)
    {
        // ── Аварійна зупинка ─────────────────────────────────────
        // LOW на піні = спрацювання (кнопка/кінцевик замкнуто на GND)
        bool estop = (gpio_get_level((gpio_num_t)ESTOP_PIN) == 0);

        if (estop)
        {
            if (!estopActive)
            {
                // Перше спрацювання — миттєво вимикаємо все
                estopActive = true;

                spindle_enable(false);
                carriage_enable(false);
                motion_emergency_stop();

                shared_lock();
                shared.running = false;
                shared_unlock();
            }

            // Поки ESTOP активний — нічого не робимо
            vTaskDelay(pdMS_TO_TICKS(10));
            continue;
        }
        else
        {
            // ESTOP відпущено — скидаємо прапорець
            estopActive = false;
        }

        // ── Звичайна робота ──────────────────────────────────────
        service_update();
        machine_update();
        motion_update();
    }
}

// ─────────────────────────────────────────
// app_main
// ─────────────────────────────────────────

extern "C" void app_main()
{
    esp_task_wdt_deinit();

    shared_state_init();

    xTaskCreatePinnedToCore(ui_task,     "ui",     8192, nullptr, 3, nullptr, 1);
    xTaskCreatePinnedToCore(motion_task, "motion", 4096, nullptr, 5, nullptr, 0);
}