#pragma once
#include "main.h"

namespace ali::pid {

int chassis_control();

void move (float dist, float theta);

void move (float dist, float theta, bool s, float s_min, float s_time);

void move (float dist, float theta, bool s, float s_min, float s_time, float max);

} // namespace ali::pid