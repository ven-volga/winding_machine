// hall.cpp
//
// Два датчики Холла A3144, два магніти зі зсувом 180°,
// датчики зі зсувом 90°.
//
// 4 імпульси на оберт шпинделя (2 магніти × 2 датчики).
//
// Визначення напрямку (виміряно експериментально):
//   CW обертання:
//     A спадає → B=HIGH
//     B спадає → A=HIGH
//   CCW обертання:
//     A спадає → B=LOW
//     B спадає → A=LOW

#include "hall.h"
#include "pins.h"
#include "config.h"

extern "C" {
    #include "freertos/FreeRTOS.h"
    #include "freertos/task.h"
    #include "driver/gpio.h"
    #include "esp_attr.h"
}

volatile int32_t  hallPulses         = 0;
volatile bool     hallSpindleRunning = false;
volatile bool     hallDir            = true;
volatile uint32_t hallLastPulseMs    = 0;

// Діагностичні змінні (можна прибрати після налагодження)
volatile uint32_t hallPulsesA = 0;
volatile uint32_t hallPulsesB = 0;
volatile bool     hallB_whenA = false;
volatile bool     hallA_whenB = false;

static void IRAM_ATTR hall_isr_a(void* arg)
{
    bool b = gpio_get_level((gpio_num_t)HALL_B_PIN);

    // CW: A спадає → B=HIGH
    bool cw = b;

    if (cw)
        hallPulses = hallPulses + 1;
    else
        hallPulses = hallPulses - 1;

    hallDir            = cw;
    hallPulsesA        = hallPulsesA + 1;
    hallB_whenA        = b;
    hallLastPulseMs    = (uint32_t)(xTaskGetTickCountFromISR() * portTICK_PERIOD_MS);
    hallSpindleRunning = true;
}

static void IRAM_ATTR hall_isr_b(void* arg)
{
    bool a = gpio_get_level((gpio_num_t)HALL_A_PIN);

    // CW: B спадає → A=HIGH
    bool cw = a;

    if (cw)
        hallPulses = hallPulses + 1;
    else
        hallPulses = hallPulses - 1;

    hallDir            = cw;
    hallPulsesB        = hallPulsesB + 1;
    hallA_whenB        = a;
    hallLastPulseMs    = (uint32_t)(xTaskGetTickCountFromISR() * portTICK_PERIOD_MS);
    hallSpindleRunning = true;
}

void hall_init()
{
    gpio_config_t conf = {};
    conf.pin_bit_mask  = (1ULL << HALL_A_PIN) | (1ULL << HALL_B_PIN);
    conf.mode          = GPIO_MODE_INPUT;
    conf.pull_up_en    = GPIO_PULLUP_DISABLE;
    conf.pull_down_en  = GPIO_PULLDOWN_DISABLE;
    conf.intr_type     = GPIO_INTR_DISABLE;
    gpio_config(&conf);

    gpio_set_intr_type((gpio_num_t)HALL_A_PIN, GPIO_INTR_NEGEDGE);
    gpio_set_intr_type((gpio_num_t)HALL_B_PIN, GPIO_INTR_NEGEDGE);

    gpio_install_isr_service(0);
    gpio_isr_handler_add((gpio_num_t)HALL_A_PIN, hall_isr_a, nullptr);
    gpio_isr_handler_add((gpio_num_t)HALL_B_PIN, hall_isr_b, nullptr);
}