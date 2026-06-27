#pragma once

// =============================================================
// hall.h — квадратурний енкодер на датчиках Холла
// =============================================================

#include <cstdint>
#include <stdbool.h>

void hall_init();

// Основні змінні
extern volatile int32_t  hallPulses;
extern volatile bool     hallSpindleRunning;
extern volatile bool     hallDir;
extern volatile uint32_t hallLastPulseMs;

// Діагностичні змінні — окремі лічильники для A і B
extern volatile uint32_t hallPulsesA;   // скільки разів спрацював A
extern volatile uint32_t hallPulsesB;   // скільки разів спрацював B
extern volatile bool     hallB_whenA;   // стан B при спаданні A
extern volatile bool     hallA_whenB;   // стан A при спаданні B