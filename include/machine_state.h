#pragma once
#include <cstdint>

struct MachineState
{
    // Diameter of wire in mm
    float wireDiameter = 0.20f;

    // Width of winding area
    float windingWidth = 30.0f;

    // Current carriage position
    float carriagePosition = 0.0f;

    // Current spindle turns
    uint32_t turns = 0;

    // Winding speed
    uint32_t spindleSpeed = 300;

    // Direction
    bool directionForward = true;

    // Running state
    bool running = false;
};

extern MachineState machineState;
