#pragma once

#include <cstdint>

void motion_init();

void motion_start();

void motion_stop();

void motion_update();

void motion_set_spindle_speed(uint32_t speed);

void motion_set_wire_diameter(float diameter);