#pragma once

// =============================================================
// config.h — всі налаштування проєкту в одному місці
// =============================================================

// ─────────────────────────────────────────
// МОТОРИ
// ─────────────────────────────────────────

#define MOTOR_STEPS     200
#define MICROSTEPS      16

// ─────────────────────────────────────────
// КІНЕМАТИКА
// ─────────────────────────────────────────

#define SPINDLE_RATIO   2.0f
#define T8_LEAD_MM      2.0f

// ─────────────────────────────────────────
// РОЗГІН
// ─────────────────────────────────────────

#define ACCEL_START_DELAY_US  8000
#define ACCEL_STEP_US         10

// ─────────────────────────────────────────
// ШВИДКОСТІ СЕРВІСНОГО РЕЖИМУ
//
// Окремі швидкості для ручного керування
// шпинделем і кареткою через кнопки.
// ─────────────────────────────────────────

// Швидкість шпинделя в сервісному режимі (об/хв)
// При утриманні кнопки CW/CCW
#define SERVICE_SPINDLE_SPEED_RPM   250

// Швидкість каретки в сервісному режимі (об/хв мотора)
// При утриманні кнопки LEFT/RIGHT
#define SERVICE_CARRIAGE_SPEED_RPM  150

// Швидкість переміщення до заданої позиції (об/хв мотора)
// Повільно для точного позиціювання
#define CARRIAGE_HOME_SPEED_RPM     60

// ─────────────────────────────────────────
// ЕНКОДЕР
// ─────────────────────────────────────────

#define ENCODER_STEP_MS   2
#define LONG_PRESS_MS     800
#define BTN_DEBOUNCE_MS   30

// ─────────────────────────────────────────
// ДІАПАЗОНИ ПАРАМЕТРІВ UI
// ─────────────────────────────────────────

#define SPINDLE_SPEED_MIN   10
#define SPINDLE_SPEED_MAX   600
#define SPINDLE_SPEED_STEP  10

#define WIRE_DIAMETER_MIN   0.05f
#define WIRE_DIAMETER_MAX   3.00f
#define WIRE_DIAMETER_STEP  0.01f

#define WINDING_WIDTH_MIN   1.0f
#define WINDING_WIDTH_MAX   200.0f
#define WINDING_WIDTH_STEP  0.5f

// Діапазон позиції каретки — від'ємні для секційної намотки
#define CARRIAGE_POS_MIN   -350.0f
#define CARRIAGE_POS_MAX    350.0f
#define CARRIAGE_POS_STEP   0.5f