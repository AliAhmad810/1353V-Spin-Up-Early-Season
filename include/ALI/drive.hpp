#pragma once
#include "main.h"

namespace ali::drive {

float get_pos();

void reset_motors();

void power_drive(float left, float right);

} // namespace ali::drive