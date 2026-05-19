#pragma once

#include <stdint.h>

// ─────────────────────────────────────────
// LCD 2004 I2C драйвер
//
// Дисплей 4 рядки × 20 символів.
// Підключення через PCF8574 I2C експандер
// (стандартний модуль з Алі).
//
// I2C адреса за замовчуванням: 0x27
// Якщо не працює — спробуй 0x3F
// ─────────────────────────────────────────

#define LCD_I2C_ADDR   0x27
#define LCD_COLS       20
#define LCD_ROWS       4

// Ініціалізація I2C і дисплею
void lcd_init();

// Очистити екран
void lcd_clear();

// Встановити курсор (col: 0-19, row: 0-3)
void lcd_set_cursor(uint8_t col, uint8_t row);

// Вивести рядок
void lcd_print(const char* str);

// Вивести один символ
void lcd_write_char(char c);

// Увімкнути/вимкнути підсвітку
void lcd_backlight(bool on);