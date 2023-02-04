#include "ALI/catapult.hpp"
#include "globals.hpp"
#include "pros/llemu.hpp"

namespace ali::cata {

bool enableShootReset = false;

int shoot_and_reset() {
    while(true) {
    while (enableShootReset) {
        pros::lcd::set_text(7, "aaaaaaaaaaaaaaaa");
        if (!cataLimit.get_value()) {
            catapult.move_voltage(12000);
        } else {
            catapult.move_voltage(0);
            enableShootReset = false;
        }
    }
    pros::delay(5);
    }
    return 1;
}

void shoot_cata() {
    catapult.move_voltage(12000);
    pros::delay(500);
    ali::cata::enableShootReset = true;
}

} // namespace ali::catapult