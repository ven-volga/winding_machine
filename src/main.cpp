// main.cpp
//
// Ядро 0 — motion_task: service, machine, motion
// Ядро 1 — ui_task:     LCD + енкодер

#include "ui.h"
#include "motion.h"
#include "machine.h"
#include "service.h"
#include "shared_state.h"
#include "esp_task_wdt.h"

extern "C" {
    #include "freertos/FreeRTOS.h"
    #include "freertos/task.h"
    #include "esp_task_wdt.h"
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
//
// motion_update() використовує esp_rom_delay_us — блокує ядро.
// IDLE0 задача не може виконатись → watchdog спрацьовує.
//
// Рішення: реєструємо motion_task у watchdog і скидаємо
// його вручну кожну ітерацію. Так watchdog знає що ми живі
// навіть якщо IDLE0 не виконується.
// ─────────────────────────────────────────

static void motion_task(void*)
{
    // Реєструємо задачу у watchdog
    // esp_task_wdt_add(NULL);

    service_init();
    machine_init();
    motion_init();

    while (true)
    {
        service_update();
        machine_update();
        motion_update();

        // Скидаємо watchdog — повідомляємо що задача жива
        // esp_task_wdt_reset();
    }
}

// ─────────────────────────────────────────
// app_main
// ─────────────────────────────────────────

extern "C" void app_main()
{
    // Вимикаємо watchdog повністю для розробки
    // Після стабілізації можна повернути з правильним таймаутом
    esp_task_wdt_deinit();

    shared_state_init();

    xTaskCreatePinnedToCore(ui_task,     "ui",     8192, nullptr, 3, nullptr, 1);
    xTaskCreatePinnedToCore(motion_task, "motion", 4096, nullptr, 5, nullptr, 0);
}