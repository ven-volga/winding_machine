// motion.cpp
//
// Генерація кроків шпинделя і синхронна подача каретки.
//
// Важливо:
//   Шпиндель ЗАВЖДИ крутиться в одному напрямку (CW).
//   dirForward визначає тільки напрямок КАРЕТКИ.
//   При реверсі каретки шпиндель не змінює напрямок.

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
// Внутрішні змінні
// ─────────────────────────────────────────

static float    carriageAccumulator = 0.0f;
static float    carriageStepRatio   = 0.0f;
static uint32_t targetDelayUs       = ACCEL_START_DELAY_US;
static uint32_t currentDelayUs      = ACCEL_START_DELAY_US;
static uint32_t localSpeed          = 300;
static float    localDiameter       = 0.20f;
static bool     localDir            = true;  // напрямок ТІЛЬКИ каретки

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

static void update_carriage_ratio(float diameter)
{
    float spindleStepsPerTurn =
        (float)(MOTOR_STEPS * MICROSTEPS) * SPINDLE_RATIO;
    float carriageStepsPerTurn =
        (diameter / T8_LEAD_MM) * (float)(MOTOR_STEPS * MICROSTEPS);
    carriageStepRatio = carriageStepsPerTurn / spindleStepsPerTurn;
}

static void update_target_delay(uint32_t rpm)
{
    if (rpm == 0) { targetDelayUs = 10000; return; }
    float stepsPerTurn   = (float)(MOTOR_STEPS * MICROSTEPS) * SPINDLE_RATIO;
    float stepsPerSecond = stepsPerTurn * (float)rpm / 60.0f;
    targetDelayUs = (uint32_t)(1000000.0f / stepsPerSecond);
}

static void ramp_update()
{
    switch (rampState)
    {
        case RAMP_IDLE:
            break;

        case RAMP_ACCELERATING:
            if (currentDelayUs > targetDelayUs + ACCEL_STEP_US)
                currentDelayUs -= ACCEL_STEP_US;
            else { currentDelayUs = targetDelayUs; rampState = RAMP_RUNNING; }
            break;

        case RAMP_RUNNING:
            if      (currentDelayUs < targetDelayUs)
                currentDelayUs += ACCEL_STEP_US;
            else if (currentDelayUs > targetDelayUs + ACCEL_STEP_US)
                currentDelayUs -= ACCEL_STEP_US;
            break;

        case RAMP_DECELERATING:
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

    update_carriage_ratio(localDiameter);
    update_target_delay(localSpeed);
    currentDelayUs = ACCEL_START_DELAY_US;
}

void motion_start()
{
    if (rampState != RAMP_IDLE) return;

    shared_lock();
    localSpeed    = shared.spindleSpeed;
    localDiameter = shared.wireDiameter;
    localDir      = shared.dirForward;
    shared_unlock();

    update_carriage_ratio(localDiameter);
    update_target_delay(localSpeed);

    // Шпиндель ЗАВЖДИ CW (true) — не залежить від dirForward
    spindle_set_dir(true);
    spindle_enable(true);

    // Каретка — відповідно до dirForward
    carriage_set_dir(localDir);
    carriage_enable(true);

    currentDelayUs = ACCEL_START_DELAY_US;
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
    update_carriage_ratio(mm);
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
        shared_lock();
        bool moveActive = shared.moveToActive;
        bool serviceActive = shared.serviceCarriageActive;
        shared_unlock();

        if (!moveActive && !serviceActive)
            vTaskDelay(pdMS_TO_TICKS(10));

        return;
    }

    ramp_update();

    // Читаємо напрямок каретки і швидкість
    shared_lock();
    bool     currentDir   = shared.dirForward;
    uint32_t currentSpeed = shared.spindleSpeed;
    shared_unlock();

    // Оновлюємо напрямок ТІЛЬКИ каретки — шпиндель не чіпаємо
    if (currentDir != localDir)
    {
        localDir = currentDir;
        carriage_set_dir(localDir);
        // spindle_set_dir — не викликаємо, шпиндель завжди CW
    }

    // Оновлення швидкості
    if (currentSpeed != localSpeed)
        motion_set_speed(currentSpeed);

    // Крок шпинделя
    spindle_step();

    // Крок каретки через accumulator
    carriageAccumulator += carriageStepRatio;
    while (carriageAccumulator >= 1.0f)
    {
        carriage_step();
        carriageAccumulator -= 1.0f;

        float stepMm = T8_LEAD_MM / (float)(MOTOR_STEPS * MICROSTEPS);
        shared_lock();
        shared.carriagePos += localDir ? stepMm : -stepMm;
        shared_unlock();
    }

    // Лічильник витків
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

    esp_rom_delay_us(currentDelayUs);
}