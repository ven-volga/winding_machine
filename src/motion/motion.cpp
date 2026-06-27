// motion.cpp
//
// 1. BRESENHAM для каретки — рівномірний розподіл кроків
// 2. РОЗГІН ПО ЧАСУ — стабільний профіль незалежно від CPU
// 3. УТРИМАННЯ ШПИНДЕЛЯ — читає GPIO напряму при зупинці

#include "motion.h"
#include "stepper.h"
#include "shared_state.h"
#include "config.h"
#include "pins.h"
#include "hall.h"
#include <cmath>

extern "C" {
    #include "freertos/FreeRTOS.h"
    #include "freertos/task.h"
    #include "esp_rom_sys.h"
    #include "driver/gpio.h"
}

typedef enum
{
    RAMP_IDLE,
    RAMP_ACCELERATING,
    RAMP_RUNNING,
    RAMP_DECELERATING,
} RampState;

static RampState rampState = RAMP_IDLE;

// ─────────────────────────────────────────
// Bresenham
// ─────────────────────────────────────────

static int32_t breshError       = 0;
static int32_t breshNumerator   = 1;
static int32_t breshDenominator = 20;

#define BRESH_SCALE 100000

static void update_bresenham(float diameter)
{
    float spindleStepsPerTurn  = (float)(MOTOR_STEPS * MICROSTEPS) * SPINDLE_RATIO;
    float carriageStepsPerTurn = (diameter / T8_LEAD_MM) * (float)(MOTOR_STEPS * MICROSTEPS);
    float ratio = carriageStepsPerTurn / spindleStepsPerTurn;
    breshNumerator   = (int32_t)(ratio * BRESH_SCALE);
    breshDenominator = BRESH_SCALE;
    breshError       = 0;
}

// ─────────────────────────────────────────
// Розгін по часу
// ─────────────────────────────────────────

static uint32_t targetDelayUs  = ACCEL_START_DELAY_US;
static uint32_t currentDelayUs = ACCEL_START_DELAY_US;
static uint32_t lastRampMs     = 0;

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

                    // Читаємо тумблер напряму з GPIO —
                    // не через shared щоб завжди мати актуальний стан
                    // SW_SPINDLE_HOLD: LOW = увімкнено (PULLUP)
                    bool hold = (gpio_get_level((gpio_num_t)SW_SPINDLE_HOLD) == 0);
                    if (!hold) spindle_enable(false);
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

    spindle_set_dir(true);
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

void motion_emergency_stop()
{
    // Миттєва зупинка — без гальмування
    // Скидаємо всі стани щоб наступний старт був чистим
    rampState      = RAMP_IDLE;
    breshError     = 0;
    currentDelayUs = ACCEL_START_DELAY_US;

    shared_lock();
    shared.running      = false;
    shared.moveToActive = false;
    shared_unlock();
}

void motion_update()
{
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

    ramp_update();

    shared_lock();
    bool     currentDir   = shared.dirForward;
    uint32_t currentSpeed = shared.spindleSpeed;
    shared_unlock();

    if (currentDir != localDir)
    {
        localDir = currentDir;
        carriage_set_dir(localDir);
    }

    if (currentSpeed != localSpeed)
        motion_set_speed(currentSpeed);

    spindle_step();

    // Діагностика Холла — тимчасово
    static uint32_t lastPrint = 0;
    static int32_t totalPulses = 0;
    totalPulses += hallPulses;
    hallPulses = 0;

    uint32_t now = (uint32_t)(xTaskGetTickCount() * portTICK_PERIOD_MS);
    if (now - lastPrint > 2000) {
        lastPrint = now;
        printf("[HALL] total=%ld A=%lu B=%lu B_whenA=%d A_whenB=%d\n",
            (long)totalPulses,
            (unsigned long)hallPulsesA,
            (unsigned long)hallPulsesB,
           (    int)hallB_whenA,
            (int)hallA_whenB);
    }

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