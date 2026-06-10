// motion.cpp
//
// Два покращення:
//
// 1. BRESENHAM замість float accumulator для каретки
//    Замість:
//      carriageAccumulator += ratio (float)
//    Використовуємо цілочисельний алгоритм Bresenham:
//      error += numerator
//      if (error >= denominator) { step; error -= denominator; }
//    Переваги:
//      - немає накопичення похибки float
//      - рівномірніший розподіл кроків (важливо для тонкого дроту)
//      - без float у внутрішньому циклі
//
// 2. РОЗГІН ПО ЧАСУ замість по кроках
//    Замість: currentDelayUs -= ACCEL_STEP_US на кожен крок
//    Використовуємо: кожні ACCEL_INTERVAL_MS змінюємо затримку
//    Переваги:
//      - стабільний профіль розгону незалежно від навантаження CPU
//      - передбачуваний час розгону/гальмування
//
// Шпиндель ЗАВЖДИ CW. dirForward = напрямок ТІЛЬКИ каретки.

#include "motion.h"
#include "stepper.h"
#include "shared_state.h"
#include "config.h"
#include <cmath>

extern "C" {
    #include "freertos/FreeRTOS.h"
    #include "freertos/task.h"
    #include "esp_rom_sys.h"
}

// ─────────────────────────────────────────
// Стани розгону
// ─────────────────────────────────────────

typedef enum
{
    RAMP_IDLE,
    RAMP_ACCELERATING,
    RAMP_RUNNING,
    RAMP_DECELERATING,
} RampState;

static RampState rampState = RAMP_IDLE;

// ─────────────────────────────────────────
// Bresenham для каретки
//
// Синхронізація: на кожен крок шпинделя каретка
// має пройти (wireDiameter / T8_LEAD_MM) / SPINDLE_RATIO кроків.
//
// Представляємо це як дріб: numerator / denominator
// де обидва — цілі числа.
//
// Наприклад для діаметру 0.20мм:
//   ratio = 0.20 / 2.0 / 2.0 = 0.05
//   numerator   = 1
//   denominator = 20
//   тобто 1 крок каретки на кожні 20 кроків шпинделя
// ─────────────────────────────────────────

static int32_t  breshError       = 0;
static int32_t  breshNumerator   = 1;
static int32_t  breshDenominator = 20;

// Оновити коефіцієнти Bresenham при зміні діаметру дроту.
// Масштабуємо на BRESH_SCALE щоб уникнути втрати точності
// при дуже малих співвідношеннях.
#define BRESH_SCALE 100000

static void update_bresenham(float diameter)
{
    // Кроків каретки на крок шпинделя (дробове число)
    float spindleStepsPerTurn  = (float)(MOTOR_STEPS * MICROSTEPS) * SPINDLE_RATIO;
    float carriageStepsPerTurn = (diameter / T8_LEAD_MM) * (float)(MOTOR_STEPS * MICROSTEPS);
    float ratio = carriageStepsPerTurn / spindleStepsPerTurn;

    // Перетворюємо в цілочисельний дріб через масштабування
    breshNumerator   = (int32_t)(ratio * BRESH_SCALE);
    breshDenominator = BRESH_SCALE;
    breshError       = 0;  // скидаємо помилку при зміні параметрів
}

// ─────────────────────────────────────────
// Розгін по часу
//
// Кожні ACCEL_INTERVAL_MS міліскунд змінюємо currentDelayUs
// на ACCEL_STEP_US — незалежно від швидкості циклу.
// ─────────────────────────────────────────

static uint32_t targetDelayUs  = ACCEL_START_DELAY_US;
static uint32_t currentDelayUs = ACCEL_START_DELAY_US;
static uint32_t lastRampMs     = 0;  // час останньої зміни затримки

// ─────────────────────────────────────────
// Інші внутрішні змінні
// ─────────────────────────────────────────

static uint32_t localSpeed    = 100;
static float    localDiameter = 0.20f;
static bool     localDir      = true;

// ─────────────────────────────────────────
// MoveTo
// ─────────────────────────────────────────

static uint32_t homeMoveDelayUs = 0;

static void update_home_delay()
{
    float stepsPerSecond =
        (float)(MOTOR_STEPS * MICROSTEPS) *
        (float)CARRIAGE_HOME_SPEED_RPM / 60.0f;
    homeMoveDelayUs = (uint32_t)(1000000.0f / stepsPerSecond);
}

static bool moveto_step()
{
    shared_lock();
    float target = shared.targetPos;
    float pos    = shared.carriagePos;
    shared_unlock();

    float diff   = target - pos;
    float stepMm = T8_LEAD_MM / (float)(MOTOR_STEPS * MICROSTEPS);

    if (fabsf(diff) < stepMm)
    {
        shared_lock();
        shared.moveToActive = false;
        shared.carriagePos  = target;
        shared_unlock();
        carriage_enable(false);
        return false;
    }

    bool dir = (diff > 0);
    carriage_set_dir(dir);
    carriage_enable(true);
    carriage_step();

    shared_lock();
    shared.carriagePos += dir ? stepMm : -stepMm;
    shared_unlock();

    esp_rom_delay_us(homeMoveDelayUs);
    return true;
}

// ─────────────────────────────────────────
// Локальні функції
// ─────────────────────────────────────────

static void update_target_delay(uint32_t rpm)
{
    if (rpm == 0) { targetDelayUs = 10000; return; }
    float stepsPerTurn   = (float)(MOTOR_STEPS * MICROSTEPS) * SPINDLE_RATIO;
    float stepsPerSecond = stepsPerTurn * (float)rpm / 60.0f;
    targetDelayUs = (uint32_t)(1000000.0f / stepsPerSecond);
}

// Розгін по часу: викликається на кожному кроці але
// реально змінює затримку тільки кожні ACCEL_INTERVAL_MS мс
static void ramp_update()
{
    uint32_t now = (uint32_t)(xTaskGetTickCount() * portTICK_PERIOD_MS);
    bool timeToUpdate = (now - lastRampMs) >= ACCEL_INTERVAL_MS;

    switch (rampState)
    {
        case RAMP_IDLE:
            break;

        case RAMP_ACCELERATING:
            if (timeToUpdate)
            {
                lastRampMs = now;
                if (currentDelayUs > targetDelayUs + ACCEL_STEP_US)
                    currentDelayUs -= ACCEL_STEP_US;
                else
                {
                    currentDelayUs = targetDelayUs;
                    rampState = RAMP_RUNNING;
                }
            }
            break;

        case RAMP_RUNNING:
            // Плавне підстроювання якщо змінили швидкість через UI
            if (timeToUpdate)
            {
                lastRampMs = now;
                if      (currentDelayUs < targetDelayUs)
                    currentDelayUs += ACCEL_STEP_US;
                else if (currentDelayUs > targetDelayUs + ACCEL_STEP_US)
                    currentDelayUs -= ACCEL_STEP_US;
            }
            break;

        case RAMP_DECELERATING:
            if (timeToUpdate)
            {
                lastRampMs = now;
                if (currentDelayUs < ACCEL_START_DELAY_US - ACCEL_STEP_US)
                    currentDelayUs += ACCEL_STEP_US;
                else
                {
                    currentDelayUs = ACCEL_START_DELAY_US;
                    rampState      = RAMP_IDLE;
                    shared_lock();
                    shared.running = false;
                    shared_unlock();
                    spindle_enable(false);
                    carriage_enable(false);
                }
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
    update_home_delay();

    shared_lock();
    localSpeed    = shared.spindleSpeed;
    localDiameter = shared.wireDiameter;
    localDir      = shared.dirForward;
    shared_unlock();

    update_bresenham(localDiameter);
    update_target_delay(localSpeed);
    currentDelayUs = ACCEL_START_DELAY_US;
    lastRampMs     = 0;
}

void motion_start()
{
    if (rampState != RAMP_IDLE) return;

    shared_lock();
    localSpeed    = shared.spindleSpeed;
    localDiameter = shared.wireDiameter;
    localDir      = shared.dirForward;
    shared_unlock();

    update_bresenham(localDiameter);
    update_target_delay(localSpeed);

    spindle_set_dir(true);   // шпиндель завжди CW
    spindle_enable(true);

    carriage_set_dir(localDir);
    carriage_enable(true);

    currentDelayUs = ACCEL_START_DELAY_US;
    lastRampMs     = (uint32_t)(xTaskGetTickCount() * portTICK_PERIOD_MS);
    rampState = RAMP_ACCELERATING;
}

void motion_stop()
{
    if (rampState == RAMP_ACCELERATING || rampState == RAMP_RUNNING)
        rampState = RAMP_DECELERATING;
}

void motion_set_speed(uint32_t rpm)
{
    localSpeed = rpm;
    update_target_delay(rpm);
}

void motion_set_wire_diameter(float mm)
{
    localDiameter = mm;
    update_bresenham(mm);
}

void motion_move_to(float mm)
{
    shared_lock();
    shared.targetPos    = mm;
    shared.moveToActive = true;
    shared_unlock();
}

void motion_update()
{
    // MoveTo — вищий пріоритет
    shared_lock();
    bool moveActive = shared.moveToActive;
    shared_unlock();

    if (moveActive)
    {
        moveto_step();
        return;
    }

    if (rampState == RAMP_IDLE)
    {
        vTaskDelay(pdMS_TO_TICKS(1));
        return;
    }

    // ── Розгін (по часу) ─────────────────────────────────────────
    ramp_update();

    // ── Читаємо актуальні параметри ──────────────────────────────
    shared_lock();
    bool     currentDir   = shared.dirForward;
    uint32_t currentSpeed = shared.spindleSpeed;
    shared_unlock();

    // ── Напрямок каретки ─────────────────────────────────────────
    if (currentDir != localDir)
    {
        localDir = currentDir;
        carriage_set_dir(localDir);
    }

    // ── Швидкість ────────────────────────────────────────────────
    if (currentSpeed != localSpeed)
        motion_set_speed(currentSpeed);

    // ── Крок шпинделя ────────────────────────────────────────────
    spindle_step();

    // ── Bresenham для каретки ────────────────────────────────────
    // Додаємо numerator до помилки кожен крок шпинделя.
    // Коли помилка >= denominator — робимо крок каретки.
    // Це рівномірно розподіляє кроки каретки між кроками шпинделя.
    breshError += breshNumerator;
    if (breshError >= breshDenominator)
    {
        breshError -= breshDenominator;
        carriage_step();

        float stepMm = T8_LEAD_MM / (float)(MOTOR_STEPS * MICROSTEPS);
        shared_lock();
        shared.carriagePos += localDir ? stepMm : -stepMm;
        shared_unlock();
    }

    // ── Лічильник витків ─────────────────────────────────────────
    static uint32_t stepCount = 0;
    stepCount++;
    uint32_t stepsPerTurn =
        (uint32_t)((float)(MOTOR_STEPS * MICROSTEPS) * SPINDLE_RATIO);
    if (stepCount >= stepsPerTurn)
    {
        stepCount = 0;
        shared_lock();
        shared.turns++;
        shared_unlock();
    }

    // ── Затримка ─────────────────────────────────────────────────
    esp_rom_delay_us(currentDelayUs);
}