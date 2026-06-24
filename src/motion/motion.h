#pragma once

#include <cstdint>

// =============================================================
// motion.h — генерація кроків і синхронізація
// =============================================================

void motion_init();
void motion_start();
void motion_stop();
void motion_set_speed(uint32_t rpm);
void motion_set_wire_diameter(float mm);

// Перемістити каретку до абсолютної позиції (мм).
// Рухається на фіксованій швидкості CARRIAGE_HOME_SPEED_RPM.
// По досягненні — автоматично зупиняється.
// Має вищий пріоритет ніж намотка.
void motion_move_to(float mm);

void motion_update();

// Миттєва аварійна зупинка — без гальмування.
// Скидає стан ramp в IDLE, скидає accumulator.
void motion_emergency_stop();