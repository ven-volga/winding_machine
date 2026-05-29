// main.cpp
//
// Точка входу.
//
// Ядро 0 — motion_task:
//   service_update() — сервісні кнопки
//   machine_update() — педаль, реверс, швидкість
//   motion_update()  — кроки, синхронізація
//
// Ядро 1 — ui_task:
//   esp_timer (1мс)  — опитування енкодера, зміна швидкості
//   ui_update (50мс) — LCD оновлення

#include "ui.h"
#include "motion.h"
#include "machine.h"
#include "service.h"
#include "shared_state.h"

extern "C" {
    #include "freertos/FreeRTOS.h"
    #include "freertos/task.h"
}

// ─────────────────────────────────────────
// UI задача — ядро 1
//
// Енкодер опитується окремим esp_timer (1мс) — у ui_init().
// Тут тільки LCD оновлення кожні 50мс.
// ─────────────────────────────────────────

static void ui_task(void*)
{
    ui_init(); // запускає і LCD і таймер енкодера

    while (true)
    {
        ui_update();
        vTaskDelay(pdMS_TO_TICKS(50)); // LCD кожні 50мс
    }
}

// ─────────────────────────────────────────
// Motion задача — ядро 0
// ─────────────────────────────────────────

static void motion_task(void*)
{
    service_init();
    machine_init();
    motion_init();

    while (true)
    {
        service_update();
        machine_update();
        motion_update();
        taskYIELD();
    }
}

// ─────────────────────────────────────────
// app_main
// ─────────────────────────────────────────

extern "C" void app_main()
{
    shared_state_init();

    xTaskCreatePinnedToCore(ui_task,     "ui",     8192, nullptr, 3, nullptr, 1);
    xTaskCreatePinnedToCore(motion_task, "motion", 4096, nullptr, 5, nullptr, 0);
}