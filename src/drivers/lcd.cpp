// lcd.cpp
//
// Драйвер LCD 2004 через I2C експандер PCF8574.
// Використовує нативний ESP-IDF I2C API.
//
// Підключення:
//   SDA → GPIO 21,  SCL → GPIO 22
//   VCC експандера → 3.3V,  VCC дисплею → 5V,  GND → GND
//
// Конфлікт ISR/I2C вирішений в encoder.cpp (polling замість ISR) —
// тому тут Mutex не потрібен і не використовується.

#include "lcd.h"
#include "pins.h"

#include "driver/i2c.h"
#include "esp_rom_sys.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

// ─────────────────────────────────────────
// Константи HD44780
// ─────────────────────────────────────────

#define LCD_CMD_CLEAR        0x01
#define LCD_CMD_ENTRY_MODE   0x06
#define LCD_CMD_DISPLAY_ON   0x0C
#define LCD_CMD_FUNCTION_SET 0x28
#define LCD_CMD_DDRAM        0x80

static const uint8_t ROW_OFFSETS[4] = {0x00, 0x40, 0x14, 0x54};

// ─────────────────────────────────────────
// Біти PCF8574
// ─────────────────────────────────────────

#define BIT_RS  0x01
#define BIT_RW  0x02
#define BIT_EN  0x04
#define BIT_BL  0x08

static uint8_t backlightBit = BIT_BL;

// ─────────────────────────────────────────
// I2C
// ─────────────────────────────────────────

#define I2C_PORT    I2C_NUM_0
#define I2C_SDA_PIN 21
#define I2C_SCL_PIN 22
#define I2C_FREQ_HZ 100000

static void i2c_write_byte(uint8_t data)
{
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (LCD_I2C_ADDR << 1) | I2C_MASTER_WRITE, true);
    i2c_master_write_byte(cmd, data, true);
    i2c_master_stop(cmd);
    i2c_master_cmd_begin(I2C_PORT, cmd, pdMS_TO_TICKS(10));
    i2c_cmd_link_delete(cmd);
}

static void lcd_strobe(uint8_t data)
{
    i2c_write_byte(data | BIT_EN | backlightBit);
    esp_rom_delay_us(10);
    i2c_write_byte((data & ~BIT_EN) | backlightBit);
    esp_rom_delay_us(200);
}

static void lcd_write_nibble(uint8_t nibble, uint8_t mode)
{
    lcd_strobe((nibble & 0xF0) | mode | backlightBit);
}

static void lcd_send(uint8_t value, uint8_t mode)
{
    lcd_write_nibble(value & 0xF0, mode);
    lcd_write_nibble((value << 4) & 0xF0, mode);
}

// ─────────────────────────────────────────
// Публічні функції
// ─────────────────────────────────────────

void lcd_init()
{
    i2c_config_t conf = {
        .mode          = I2C_MODE_MASTER,
        .sda_io_num    = I2C_SDA_PIN,
        .scl_io_num    = I2C_SCL_PIN,
        .sda_pullup_en = GPIO_PULLUP_ENABLE,
        .scl_pullup_en = GPIO_PULLUP_ENABLE,
        .master        = { .clk_speed = I2C_FREQ_HZ },
        .clk_flags     = 0,
    };
    i2c_param_config(I2C_PORT, &conf);
    i2c_driver_install(I2C_PORT, I2C_MODE_MASTER, 0, 0, 0);

    vTaskDelay(pdMS_TO_TICKS(100));

    // Ініціалізація HD44780 в 4-бітному режимі (datasheet послідовність)
    lcd_write_nibble(0x30, 0); vTaskDelay(pdMS_TO_TICKS(5));
    lcd_write_nibble(0x30, 0); vTaskDelay(pdMS_TO_TICKS(1));
    lcd_write_nibble(0x30, 0); vTaskDelay(pdMS_TO_TICKS(1));
    lcd_write_nibble(0x20, 0); vTaskDelay(pdMS_TO_TICKS(1));

    lcd_send(LCD_CMD_FUNCTION_SET, 0);
    lcd_send(LCD_CMD_DISPLAY_ON,   0);
    lcd_send(LCD_CMD_CLEAR,        0);
    vTaskDelay(pdMS_TO_TICKS(5));
    lcd_send(LCD_CMD_ENTRY_MODE,   0);
}

void lcd_clear()
{
    lcd_send(LCD_CMD_CLEAR, 0);
    vTaskDelay(pdMS_TO_TICKS(5));
}

void lcd_set_cursor(uint8_t col, uint8_t row)
{
    if (row >= LCD_ROWS) row = LCD_ROWS - 1;
    if (col >= LCD_COLS) col = LCD_COLS - 1;
    lcd_send(LCD_CMD_DDRAM | (col + ROW_OFFSETS[row]), 0);
}

void lcd_print(const char* str)
{
    while (*str)
        lcd_send((uint8_t)*str++, BIT_RS);
}

void lcd_write_char(char c)
{
    lcd_send((uint8_t)c, BIT_RS);
}

void lcd_backlight(bool on)
{
    backlightBit = on ? BIT_BL : 0x00;
    i2c_write_byte(backlightBit);
}