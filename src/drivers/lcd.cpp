#include "lcd.h"

#include <cstring>

extern "C"
{
    #include "driver/i2c.h"
    #include "esp_rom_sys.h"
}

#define I2C_MASTER_SCL_IO           22
#define I2C_MASTER_SDA_IO           21
#define I2C_MASTER_NUM              I2C_NUM_0
#define I2C_MASTER_FREQ_HZ          100000

#define LCD_ADDR                    0x27

#define LCD_BACKLIGHT               0x08
#define LCD_ENABLE                  0x04
#define LCD_RS                      0x01

static void i2c_write(uint8_t data)
{
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();

    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (LCD_ADDR << 1) | I2C_MASTER_WRITE, true);
    i2c_master_write_byte(cmd, data, true);
    i2c_master_stop(cmd);

    i2c_master_cmd_begin(
        I2C_MASTER_NUM,
        cmd,
        pdMS_TO_TICKS(100)
    );

    i2c_cmd_link_delete(cmd);
}

static void lcd_strobe(uint8_t data)
{
    i2c_write(data | LCD_ENABLE | LCD_BACKLIGHT);

    esp_rom_delay_us(10);

    i2c_write((data & ~LCD_ENABLE) | LCD_BACKLIGHT);

    esp_rom_delay_us(50);
}

static void lcd_write4bits(uint8_t data)
{
    lcd_strobe(data);
}

static void lcd_send(uint8_t value, uint8_t mode)
{
    uint8_t high = value & 0xF0;
    uint8_t low  = (value << 4) & 0xF0;

    lcd_write4bits(high | mode);
    lcd_write4bits(low | mode);
}

static void lcd_command(uint8_t cmd)
{
    lcd_send(cmd, 0);

    esp_rom_delay_us(2000);
}

static void lcd_write_char(char c)
{
    lcd_send(c, LCD_RS);

    esp_rom_delay_us(100);
}

void lcd_clear()
{
    lcd_command(0x01);

    vTaskDelay(pdMS_TO_TICKS(5));
}

void lcd_set_cursor(uint8_t col, uint8_t row)
{
    static const uint8_t row_offsets[] =
    {
        0x00,
        0x40,
        0x14,
        0x54
    };

    lcd_command(0x80 | (col + row_offsets[row]));
}

void lcd_print(const char* text)
{
    while(*text)
    {
        lcd_write_char(*text++);
    }
}

void lcd_init()
{
    i2c_config_t conf = {};

    conf.mode = I2C_MODE_MASTER;
    conf.sda_io_num = (gpio_num_t)I2C_MASTER_SDA_IO;
    conf.scl_io_num = (gpio_num_t)I2C_MASTER_SCL_IO;

    conf.sda_pullup_en = GPIO_PULLUP_ENABLE;
    conf.scl_pullup_en = GPIO_PULLUP_ENABLE;

    conf.master.clk_speed = I2C_MASTER_FREQ_HZ;

    i2c_param_config(I2C_MASTER_NUM, &conf);

    i2c_driver_install(
        I2C_MASTER_NUM,
        conf.mode,
        0,
        0,
        0
    );

    vTaskDelay(pdMS_TO_TICKS(50));

    lcd_write4bits(0x30);
    esp_rom_delay_us(5000);

    lcd_write4bits(0x30);
    esp_rom_delay_us(5000);

    lcd_write4bits(0x30);
    esp_rom_delay_us(5000);

    lcd_write4bits(0x20);
    esp_rom_delay_us(5000);

    lcd_command(0x28);
    lcd_command(0x0C);
    lcd_command(0x06);

    lcd_clear();
}