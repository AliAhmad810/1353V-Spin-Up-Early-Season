#include "ALI/drive.hpp"

namespace ali::drive {

#define WHEEL_SIZE 3.25
#define DRIVING_GEAR 36.0
#define DRIVEN_GEAR 60.0
#define MOTOR_RPM 600.0

float inches_per_tick = (WHEEL_SIZE * M_PI) / (3600.0 / MOTOR_RPM * 50) * (DRIVING_GEAR / DRIVEN_GEAR);

float avg_motors() {
    return (rF.get_position() + 
            rM.get_position() + 
            rB.get_position() + 
            lF.get_position() + 
            lM.get_position() + 
            lB.get_position()) / 6.0;
}

float get_pos() {
    return avg_motors() * inches_per_tick;
} 

void reset_motors() {
    rF.tare_position();
    rM.tare_position();
    rB.tare_position();
    lF.tare_position();
    lM.tare_position();
    lB.tare_position();
}

void power_drive(float left, float right) {
    
    rF.move_voltage(right);
    rM.move_voltage(right);
    rB.move_voltage(right);
    lF.move_voltage(left);
    lM.move_voltage(left);
    lB.move_voltage(left);
}

} // namespace ali::drive