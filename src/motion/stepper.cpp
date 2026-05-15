#include "stepper.h"
#include "pins.h"

#include "driver/gpio.h"
#include "esp_rom_sys.h"

void stepper_init()
{
    gpio_set_direction((gpio_num_t)SPINDLE_STEP_PIN, GPIO_MODE_OUTPUT);
    gpio_set_direction((gpio_num_t)SPINDLE_DIR_PIN, GPIO_MODE_OUTPUT);
    gpio_set_direction((gpio_num_t)SPINDLE_EN_PIN, GPIO_MODE_OUTPUT);

    gpio_set_direction((gpio_num_t)CARRIAGE_STEP_PIN, GPIO_MODE_OUTPUT);
    gpio_set_direction((gpio_num_t)CARRIAGE_DIR_PIN, GPIO_MODE_OUTPUT);
    gpio_set_direction((gpio_num_t)CARRIAGE_EN_PIN, GPIO_MODE_OUTPUT);

    gpio_set_level((gpio_num_t)SPINDLE_EN_PIN, 0);
    gpio_set_level((gpio_num_t)CARRIAGE_EN_PIN, 0);
}

void spindle_step()
{
    gpio_set_level((gpio_num_t)SPINDLE_STEP_PIN, 1);

    esp_rom_delay_us(2);

    gpio_set_level((gpio_num_t)SPINDLE_STEP_PIN, 0);
}

void carriage_step()
{
    gpio_set_level((gpio_num_t)CARRIAGE_STEP_PIN, 1);

    esp_rom_delay_us(2);

    gpio_set_level((gpio_num_t)CARRIAGE_STEP_PIN, 0);
}