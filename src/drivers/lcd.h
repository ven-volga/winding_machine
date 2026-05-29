#pragma once

#include <cstdint>

void lcd_init();
void lcd_clear();

void lcd_set_cursor(uint8_t col, uint8_t row);
void lcd_print(const char* text);