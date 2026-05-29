#pragma once

// =============================================================
// encoder.h — квадратурний енкодер з кнопкою
//
// Polling без ISR — безпечно для I2C LCD.
// Gray-code decoder з порогом 4 переходи на клік.
// Кнопка: debounce + короткий натиск + довгий натиск.
// =============================================================

#include <cstdint>
#include <stdbool.h>

// Ініціалізація GPIO енкодера
void encoder_init();

// Опитування — викликати кожні 1мс з ui_task
void encoder_update();

// Результати після encoder_update():
// encDelta: -1, 0, +1 за цей цикл
// evClick:  true якщо був короткий натиск
// evLong:   true якщо був довгий натиск
//
// Скидаються автоматично на початку наступного encoder_update()

extern int8_t encDelta;
extern bool   evClick;
extern bool   evLong;