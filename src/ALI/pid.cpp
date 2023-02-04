#include "pid.hpp"
#include "globals.hpp"

namespace ali::pid {

// Linear PID constants + declarations
float lin_kP = 100;
float lin_kD = 0;
float lin_kI = 0; 

float lin_error; // sensor - desired = position
float prev_lin_error = 0; // position last loop
float lin_derivative; // error - prev_error = speed
float total_lin_error = 0; // total error accumulated

// Angular PID constants + declarations
float ang_kP = 100;
float ang_kD = 0;
float ang_kI = 0;

float ang_error; // sensor - desired = position
float prev_ang_error = 0; // position last loop
float ang_derivative; // error - prev_error = speed
float total_ang_error = 0; // total error accumulated

// Autonomous Settings
float distance = 0; // desired distance, in inches
float angle = 0; // desired angle, in degrees
bool use_slew  = false;
float slew_min = 0; // minimum velocity to start at
float slew_time = 1; // time to max velocity, in milliseconds
float max_volts = 10000; // maximum voltage to run at

// Autonomous Settings; not configured
float dist_to_stop = 0.15; // in inches
float ang_to_stop = 0.5; // in degrees

// Toggle to enable PID
bool enable_PID = false;

float calc_slew_step() { // how much to speed up every loop
    float vel_diff = max_volts - slew_min; // velocity needed to accel over distance
    int steps = (int) (slew_time / 20); // how many loop iterations to go through
    return vel_diff / steps; // how much to increase velocity each iteration
}

float calc_lin_PID () {
    // get the current position of the drive relative to start of the movement
    float curr_pos = drive::get_pos();

    // calculate error (distance to target)
    lin_error = distance - curr_pos;

    // calculate derivative
    lin_derivative = lin_error - prev_lin_error;

    // calculate output
    float output = ((lin_error * lin_kP) + (lin_derivative * lin_kD)); 
    

    if (slew_min < max_volts && use_slew) {
        output = slew_min * ((output>0) - (output<0));
        slew_min += calc_slew_step();
    } else if (output > max_volts) {
        output = max_volts;
    }

    prev_lin_error = lin_error;

    return output;
}

float calc_ang_PID() {
    // get current angle of the bot relative to start of the match
    float curr_angle = inertial.get_heading();

    // calculate error (angle to target)
    ang_error = angle - curr_angle;

    // calculate derivative
    ang_derivative = ang_error - prev_ang_error;

    // calculate output
    float output = ((ang_error * ang_kP) + (ang_derivative * ang_kD));

    if (output > max_volts) {
        output = max_volts;
    }

    prev_ang_error = ang_error;

    return output; 
}

int chassis_control () {
    while (enable_PID) {
        float lateral = calc_lin_PID();
        float angular = calc_ang_PID();
        float left_power = lateral + angular;
        float right_power = lateral - angular;

        drive::power_drive(left_power, right_power);
        /*
        if (fabs(lin_error) <= dist_to_stop && fabs(ang_error) <=ang_to_stop) {
            enable_PID = false;
        }
        */

        pros::delay(20);
        
        
    }
    return 1;
}

void move (float dist, float theta) {
    drive::reset_motors();
    
    distance = dist;
    angle = theta;

    enable_PID = true;

    chassis_control();
}

void move (float dist, float theta, bool s, float s_min, float s_time) {
    drive::reset_motors();
    distance = dist;
    angle = theta;
    use_slew = s;
    slew_min = s_min;
    slew_time = s_time;

    enable_PID = true;
}

void move (float dist, float theta, bool s, float s_min, float s_time, float max) {
    drive::reset_motors();
    distance = dist;
    angle = theta;
    use_slew = s;
    slew_min = s_min;
    slew_time = s_time;
    max_volts = max;

    enable_PID = true;
}

} // namespace ali::pid