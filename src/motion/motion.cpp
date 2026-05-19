// motion.cpp
//
// Модуль руху: генерація кроків шпинделя і синхронна
// подача каретки укладача.
//
// Як працює синхронізація:
//   За кожен оберт шпинделя каретка має зсунутись
//   рівно на один діаметр дроту (wireDiameter мм).
//   Оскільки крок T8 гвинта = 2 мм/оберт мотора,
//   а мотор каретки без редукції — рахуємо скільки
//   кроків каретки потрібно на кожен крок шпинделя.
//
// Формула (виводиться один раз в motion_update_ratio):
//
//   кроків_каретки_за_оберт_шпинделя =
//       (wireDiameter / T8_LEAD_MM) * MOTOR_STEPS * MICROSTEPS * SPINDLE_RATIO
//
//   на кожен крок шпинделя додаємо до акумулятора:
//       carriageAccumulator += кроків_каретки_за_оберт / (MOTOR_STEPS * MICROSTEPS * SPINDLE_RATIO)
//
//   коли акумулятор >= 1.0 — робимо крок каретки і віднімаємо 1.0
//
// Це дозволяє точно дотримуватись дробового співвідношення
// без накопиченої похибки.

#include "motion.h"
#include "stepper.h"
#include "machine_state.h"
#include "config.h"
#include "pins.h"

#include "driver/gpio.h"
#include "esp_rom_sys.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

// ─────────────────────────────────────────
// Внутрішні змінні
// ─────────────────────────────────────────

// Накопичувач дробового кроку каретки.
// Додаємо до нього частку за кожен крок шпинделя.
// Коли перевищує 1.0 — робимо реальний крок каретки.
static float carriageAccumulator = 0.0f;

// Скільки "часток кроку каретки" додавати за кожен крок шпинделя.
// Перераховується при зміні wireDiameter.
static float carriageStepRatio = 0.0f;

// Затримка між кроками шпинделя в мікросекундах.
// Визначає швидкість намотки.
// Перераховується при зміні spindleSpeed.
static uint32_t spindleStepDelayUs = 1000;

// ─────────────────────────────────────────
// Локальні функції
// ─────────────────────────────────────────

// Перерахувати співвідношення кроків каретки до шпинделя.
// Викликати при зміні wireDiameter.
static void update_carriage_ratio()
{
    // Кількість мікрокроків шпинделя на один його оберт.
    // Мотор шпинделя через ремінь 1:2, тому мотор робить
    // SPINDLE_RATIO оберти на 1 оберт шпинделя.
    float spindleStepsPerTurn = (float)(MOTOR_STEPS * MICROSTEPS) * SPINDLE_RATIO;

    // За один оберт шпинделя каретка має пройти wireDiameter мм.
    // T8_LEAD_MM — це скільки мм проходить каретка за 1 оберт мотора.
    // Тому кроків мотора каретки на 1 оберт шпинделя:
    //   (wireDiameter / T8_LEAD_MM) * MOTOR_STEPS * MICROSTEPS
    float carriageStepsPerSpindleTurn =
        (machineState.wireDiameter / T8_LEAD_MM) * (float)(MOTOR_STEPS * MICROSTEPS);

    // Скільки кроків каретки на 1 крок шпинделя (дробове число)
    carriageStepRatio = carriageStepsPerSpindleTurn / spindleStepsPerTurn;
}

// Перерахувати затримку кроку шпинделя зі швидкості (об/хв).
// Викликати при зміні spindleSpeed.
static void update_spindle_delay()
{
    if (machineState.spindleSpeed == 0)
    {
        spindleStepDelayUs = 10000; // захист від ділення на 0
        return;
    }

    // Кількість мікрокроків на оберт шпинделя
    float stepsPerTurn = (float)(MOTOR_STEPS * MICROSTEPS) * SPINDLE_RATIO;

    // Кроків за секунду при заданій швидкості (об/хв)
    float stepsPerSecond = stepsPerTurn * (float)machineState.spindleSpeed / 60.0f;

    // Мікросекунд між кроками
    spindleStepDelayUs = (uint32_t)(1000000.0f / stepsPerSecond);
}

// ─────────────────────────────────────────
// Публічні функції
// ─────────────────────────────────────────

void motion_init()
{
    stepper_init();

    // Початковий розрахунок співвідношень
    update_carriage_ratio();
    update_spindle_delay();
}

void motion_start()
{
    machineState.running = true;
}

void motion_stop()
{
    machineState.running = false;
}

void motion_set_spindle_speed(uint32_t rpm)
{
    machineState.spindleSpeed = rpm;
    update_spindle_delay();
}

void motion_set_wire_diameter(float diameter)
{
    machineState.wireDiameter = diameter;
    update_carriage_ratio();
}

// motion_update — головний цикл руху.
//
// Викликається з RTOS задачі на ядрі 0 в щільному циклі.
// Кожен виклик генерує ОДИН крок шпинделя (якщо running)
// і за потреби крок каретки.
//
// Затримка між викликами = spindleStepDelayUs (busy-wait через esp_rom_delay_us).
// Так забезпечується точна швидкість без drift від планувальника.

void motion_update()
{
    if (!machineState.running)
        return;

    // ── 1. Встановити напрямок каретки ──────────────────────────
    gpio_set_level(
        (gpio_num_t)CARRIAGE_DIR_PIN,
        machineState.directionForward ? 1 : 0
    );

    // ── 2. Крок шпинделя ─────────────────────────────────────────
    spindle_step();

    // ── 3. Накопичуємо частку кроку каретки ─────────────────────
    carriageAccumulator += carriageStepRatio;

    // ── 4. Якщо накопичилось >= 1.0 — крок каретки ──────────────
    while (carriageAccumulator >= 1.0f)
    {
        carriage_step();
        carriageAccumulator -= 1.0f;

        // Оновлюємо позицію в мм для відображення на дисплеї
        float stepMm = T8_LEAD_MM / (float)(MOTOR_STEPS * MICROSTEPS);
        if (machineState.directionForward)
            machineState.carriagePosition += stepMm;
        else
            machineState.carriagePosition -= stepMm;
    }

    // ── 5. Чекаємо до наступного кроку шпинделя ─────────────────
    // esp_rom_delay_us — точний busy-wait, не залежить від планувальника
    esp_rom_delay_us(spindleStepDelayUs);
}