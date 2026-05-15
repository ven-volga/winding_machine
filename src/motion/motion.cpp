#include "motion.h"
#include "stepper.h"
#include "pins.h"
#include "machine_state.h"
#include "config.h"

MachineState machineState;

// carriage accumulator
// allows fractional synchronization
static float carriageAccumulator = 0.0f;

void motion_init()
{
    stepper_init();
}

void motion_start()
{
    machineState.running = true;
}

void motion_stop()
{
    machineState.running = false;
}

void motion_set_spindle_speed(uint32_t speed)
{
    machineState.spindleSpeed = speed;
}

void motion_set_wire_diameter(float diameter)
{
    machineState.wireDiameter = diameter;
}

void motion_update()
{
    if(!machineState.running)
        return;

    // TODO:
    // spindle motion generation
    // carriage synchronization
}