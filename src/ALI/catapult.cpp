#include "ALI/catapult.hpp"
#include "globals.hpp"
#include "pros/llemu.hpp"

namespace ali::cata {

bool shoot = false;

void cata_control() {
    while (true) {
        if(shoot) {
            catapult.move_voltage(12000);
            pros::delay(300);
            catapult.move_voltage(0);
        }

        if (!cataLimit.get_value()) {
            catapult.move_voltage(12000); 
        } else {
            catapult.move_voltage(0);
            shoot = false;
        }

        pros::delay(5);
    }
}

void toggle_cata() {
    shoot = true;
    pros::delay(200);
}

// bool enableShootReset = false;

// int shoot_and_reset() {
//     while(true) {
//     while (enableShootReset) {
//         if (!cataLimit.get_value()) {
//             catapult.move_voltage(12000);
//         } else {
//             catapult.move_voltage(0);
//             enableShootReset = false;
//         }
//     }
//     pros::delay(5);
//     }
//     return 1;
// }

// void shoot_cata() {
//     catapult.move_voltage(12000);
//     pros::delay(500);
//     ali::cata::enableShootReset = true;
// }

} // namespace ali::catapult