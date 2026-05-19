// lcd.cpp
//
// Драйвер LCD 2004 через I2C експандер PCF8574.
// Використовує нативний ESP-IDF I2C API (не Arduino).
//
// Підключення:
//   SDA → GPIO 21
//   SCL → GPIO 22
//   VCC → 5V (або 3.3V якщо є jumper на модулі)
//   GND → GND
//
// PCF8574 керує 8 лініями LCD через I2C:
//   P0 → RS    (Register Select)
//   P1 → RW    (завжди 0 — тільки запис)
//   P2 → EN    (Enable — строб даних)
//   P3 → BL    (підсвітка)
//   P4 → D4    (шина даних, старший нібл)
//   P5 → D5
//   P6 → D6
//   P7 → D7

#include "lcd.h"
#include "pins.h"

#include "driver/i2c.h"
#include "esp_rom_sys.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

// ─────────────────────────────────────────
// Константи LCD команд (HD44780)
// ─────────────────────────────────────────

#define LCD_CMD_CLEAR          0x01
#define LCD_CMD_HOME           0x02
#define LCD_CMD_ENTRY_MODE     0x06  // зліва направо, без зсуву
#define LCD_CMD_DISPLAY_ON     0x0C  // дисплей вкл, курсор викл, моргання викл
#define LCD_CMD_FUNCTION_SET   0x28  // 4-бітна шина, 2 рядки, 5×8 точок
#define LCD_CMD_DDRAM          0x80  // базова адреса DDRAM

// Початкові адреси рядків в DDRAM пам'яті HD44780
// (для 2004 дисплею)
static const uint8_t ROW_OFFSETS[4] = {0x00, 0x40, 0x14, 0x54};

// ─────────────────────────────────────────
// Біти PCF8574
// ─────────────────────────────────────────

#define BIT_RS  0x01
#define BIT_RW  0x02
#define BIT_EN  0x04
#define BIT_BL  0x08

// Поточний стан підсвітки (додається до кожного байту)
static uint8_t backlightBit = BIT_BL;

// ─────────────────────────────────────────
// I2C налаштування
// ─────────────────────────────────────────

#define I2C_PORT     I2C_NUM_0
#define I2C_SDA_PIN  21
#define I2C_SCL_PIN  22
#define I2C_FREQ_HZ  100000  // 100 кГц — стандарт для LCD

// ─────────────────────────────────────────
// Низькорівневі функції
// ─────────────────────────────────────────

// Надіслати один байт до PCF8574 через I2C
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

// Стробувати EN: надіслати байт з EN=1, потім EN=0
// Так LCD зчитує дані
static void lcd_strobe(uint8_t data)
{
    i2c_write_byte(data | BIT_EN | backlightBit);
    esp_rom_delay_us(1);
    i2c_write_byte((data & ~BIT_EN) | backlightBit);
    esp_rom_delay_us(50);
}

// Надіслати старший нібл (4 біти) до LCD
static void lcd_write_nibble(uint8_t nibble, uint8_t mode)
{
    // mode: 0 = команда (RS=0), BIT_RS = дані (RS=1)
    uint8_t data = (nibble & 0xF0) | mode | backlightBit;
    lcd_strobe(data);
}

// Надіслати повний байт (команда або дані) через 4-бітну шину
// Спочатку старший нібл, потім молодший
static void lcd_send(uint8_t value, uint8_t mode)
{
    lcd_write_nibble(value & 0xF0, mode);          // старший нібл
    lcd_write_nibble((value << 4) & 0xF0, mode);   // молодший нібл
}

// ─────────────────────────────────────────
// Публічні функції
// ─────────────────────────────────────────

void lcd_init()
{
    // Налаштування I2C шини
    i2c_config_t conf = {
        .mode             = I2C_MODE_MASTER,
        .sda_io_num       = I2C_SDA_PIN,
        .scl_io_num       = I2C_SCL_PIN,
        .sda_pullup_en    = GPIO_PULLUP_ENABLE,
        .scl_pullup_en    = GPIO_PULLUP_ENABLE,
        .master = {
            .clk_speed    = I2C_FREQ_HZ,
        },
        .clk_flags        = 0,
    };
    i2c_param_config(I2C_PORT, &conf);
    i2c_driver_install(I2C_PORT, I2C_MODE_MASTER, 0, 0, 0);

    // Затримка після подачі живлення
    vTaskDelay(pdMS_TO_TICKS(50));

    // Ініціалізація HD44780 в 4-бітному режимі
    // (послідовність згідно datasheet)
    lcd_write_nibble(0x30, 0); vTaskDelay(pdMS_TO_TICKS(5));
    lcd_write_nibble(0x30, 0); esp_rom_delay_us(150);
    lcd_write_nibble(0x30, 0); esp_rom_delay_us(150);
    lcd_write_nibble(0x20, 0); esp_rom_delay_us(150); // перехід в 4-біт режим

    // Основні команди ініціалізації
    lcd_send(LCD_CMD_FUNCTION_SET, 0);  // 4-біт, 2 рядки, 5×8
    lcd_send(LCD_CMD_DISPLAY_ON,   0);  // дисплей вкл
    lcd_send(LCD_CMD_CLEAR,        0);  // очистити
    vTaskDelay(pdMS_TO_TICKS(2));       // clear потребує >1.5 мс
    lcd_send(LCD_CMD_ENTRY_MODE,   0);  // напрямок вводу
}

void lcd_clear()
{
    lcd_send(LCD_CMD_CLEAR, 0);
    vTaskDelay(pdMS_TO_TICKS(2));
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
    {
        lcd_send((uint8_t)*str++, BIT_RS);
    }
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