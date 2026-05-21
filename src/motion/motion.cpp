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
// Як працює acceleration ramp:
//   Замість миттєвого переходу на цільову швидкість —
//   поступово зменшуємо затримку між кроками від
//   ACCEL_START_DELAY_US до цільового spindleStepDelayUs.
//   На зупинці — навпаки, збільшуємо затримку до ACCEL_START_DELAY_US.
//   Це знімає ривки на старті і запобігає пропуску кроків.

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
// Налаштування розгону
// ─────────────────────────────────────────

// Початкова затримка між кроками при старті (мкс).
// 8000 мкс ≈ 15-20 об/хв — досить повільно щоб мотор
// гарантовано не пропустив перший крок.
#define ACCEL_START_DELAY_US  8000

// На скільки мкс зменшувати затримку за кожен крок під час розгону.
// 10 мкс на крок — хороший баланс для NEMA17.
#define ACCEL_STEP_US         10

// ─────────────────────────────────────────
// Внутрішні змінні
// ─────────────────────────────────────────

// Накопичувач дробового кроку каретки.
static float carriageAccumulator = 0.0f;

// Скільки "часток кроку каретки" додавати за кожен крок шпинделя.
static float carriageStepRatio = 0.0f;

// Цільова затримка між кроками (розрахована зі spindleSpeed).
static uint32_t targetStepDelayUs = 8000;

// Поточна затримка між кроками (змінюється під час розгону/гальмування).
static uint32_t currentStepDelayUs = 8000;

// Стани розгону
typedef enum {
    RAMP_IDLE,         // стоїть
    RAMP_ACCELERATING, // розганяється
    RAMP_RUNNING,      // крейсерська швидкість
    RAMP_DECELERATING, // гальмує
} RampState;

static RampState rampState = RAMP_IDLE;

// ─────────────────────────────────────────
// Локальні функції
// ─────────────────────────────────────────

static void update_carriage_ratio()
{
    float spindleStepsPerTurn = (float)(MOTOR_STEPS * MICROSTEPS) * SPINDLE_RATIO;
    float carriageStepsPerSpindleTurn =
        (machineState.wireDiameter / T8_LEAD_MM) * (float)(MOTOR_STEPS * MICROSTEPS);
    carriageStepRatio = carriageStepsPerSpindleTurn / spindleStepsPerTurn;
}

static void update_spindle_delay()
{
    if (machineState.spindleSpeed == 0)
    {
        targetStepDelayUs = 10000;
        return;
    }
    float stepsPerTurn   = (float)(MOTOR_STEPS * MICROSTEPS) * SPINDLE_RATIO;
    float stepsPerSecond = stepsPerTurn * (float)machineState.spindleSpeed / 60.0f;
    targetStepDelayUs    = (uint32_t)(1000000.0f / stepsPerSecond);
}

// Оновити поточну затримку відповідно до стану розгону.
// Викликається кожен крок шпинделя.
static void ramp_update()
{
    switch (rampState)
    {
        case RAMP_IDLE:
            break;

        case RAMP_ACCELERATING:
            // Зменшуємо затримку (прискорюємось) до цільової
            if (currentStepDelayUs > targetStepDelayUs + ACCEL_STEP_US)
                currentStepDelayUs -= ACCEL_STEP_US;
            else
            {
                currentStepDelayUs = targetStepDelayUs;
                rampState = RAMP_RUNNING;
            }
            break;

        case RAMP_RUNNING:
            // Якщо змінили швидкість через UI — плавно підлаштовуємось
            if (currentStepDelayUs < targetStepDelayUs)
                currentStepDelayUs += ACCEL_STEP_US;  // гальмуємо
            else if (currentStepDelayUs > targetStepDelayUs + ACCEL_STEP_US)
                currentStepDelayUs -= ACCEL_STEP_US;  // прискорюємось
            break;

        case RAMP_DECELERATING:
            // Збільшуємо затримку (гальмуємо) до початкової
            if (currentStepDelayUs < ACCEL_START_DELAY_US - ACCEL_STEP_US)
                currentStepDelayUs += ACCEL_STEP_US;
            else
            {
                // Повністю зупинились
                currentStepDelayUs   = ACCEL_START_DELAY_US;
                rampState            = RAMP_IDLE;
                machineState.running = false;
            }
            break;
    }
}

// ─────────────────────────────────────────
// Публічні функції
// ─────────────────────────────────────────

void motion_init()
{
    stepper_init();
    update_carriage_ratio();
    update_spindle_delay();
    currentStepDelayUs = ACCEL_START_DELAY_US;
}

void motion_start()
{
    if (rampState == RAMP_IDLE)
    {
        // Починаємо з повільної швидкості і розганяємось
        currentStepDelayUs   = ACCEL_START_DELAY_US;
        machineState.running = true;
        rampState            = RAMP_ACCELERATING;
    }
    else if (rampState == RAMP_DECELERATING)
    {
        // Педаль знову натиснули під час гальмування —
        // повертаємось до розгону з поточної швидкості
        rampState = RAMP_ACCELERATING;
    }
}

void motion_stop()
{
    if (rampState == RAMP_ACCELERATING || rampState == RAMP_RUNNING)
    {
        // Плавне гальмування замість миттєвої зупинки
        rampState = RAMP_DECELERATING;
        // machineState.running скинеться після завершення гальмування
    }
}

void motion_set_spindle_speed(uint32_t rpm)
{
    machineState.spindleSpeed = rpm;
    update_spindle_delay();
    // Якщо вже крутимось — ramp_update() підхопить нову ціль плавно
}

void motion_set_wire_diameter(float diameter)
{
    machineState.wireDiameter = diameter;
    update_carriage_ratio();
}

// motion_update — головний цикл руху.
//
// Викликається з RTOS задачі на ядрі 0 в щільному циклі.
// Кожен виклик:
//   1. Оновлює поточну затримку (ramp)
//   2. Генерує крок шпинделя
//   3. За потреби — крок каретки
//   4. Чекає currentStepDelayUs

void motion_update()
{
    // Якщо повністю зупинились — не молотимо порожній цикл
    if (rampState == RAMP_IDLE)
    {
        vTaskDelay(pdMS_TO_TICKS(10));
        return;
    }

    // ── 1. Оновити швидкість (розгін/гальмування) ───────────────
    ramp_update();

    // ── 2. Встановити напрямок каретки ──────────────────────────
    gpio_set_level(
        (gpio_num_t)CARRIAGE_DIR_PIN,
        machineState.directionForward ? 1 : 0
    );

    // ── 3. Крок шпинделя ─────────────────────────────────────────
    spindle_step();

    // ── 4. Накопичуємо частку кроку каретки ─────────────────────
    carriageAccumulator += carriageStepRatio;

    // ── 5. Якщо накопичилось >= 1.0 — крок каретки ──────────────
    while (carriageAccumulator >= 1.0f)
    {
        carriage_step();
        carriageAccumulator -= 1.0f;

        float stepMm = T8_LEAD_MM / (float)(MOTOR_STEPS * MICROSTEPS);
        if (machineState.directionForward)
            machineState.carriagePosition += stepMm;
        else
            machineState.carriagePosition -= stepMm;
    }

    // ── 6. Чекаємо до наступного кроку ──────────────────────────
    esp_rom_delay_us(currentStepDelayUs);
}