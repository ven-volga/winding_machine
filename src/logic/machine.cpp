#include "machine.h"
#include "motion/motion.h"
#include "machine_state.h"
#include "pins.h"

#include "driver/gpio.h"

void machine_init()
{
    gpio_set_direction((gpio_num_t)PEDAL_PIN, GPIO_MODE_INPUT);
}

void machine_update()
{
    // pedal hold-to-run
    bool pedalPressed = gpio_get_level((gpio_num_t)PEDAL_PIN);

    if(pedalPressed)
    {
        motion_start();
    }
    else
    {
        motion_stop();
    }

    // future:
    // width control
    // auto reverse
    // turn counting
    // homing
}