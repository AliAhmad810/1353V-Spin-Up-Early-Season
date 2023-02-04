#pragma once
#include "main.h"

// Intake motor
inline pros::Motor intake (7);

// Catapult motor
inline pros::Motor catapult (14);

// Catapult limit switch
inline pros::ADIDigitalIn cataLimit ('A');

// Piston boost pistons
inline pros::ADIDigitalOut bandSlip ('G');

// Catapult rotation sensor
inline pros::Rotation cataRotation (4);

// Vision sensor
inline pros::Vision vision (16);

// Expansion
inline pros::ADIDigitalOut expansion ('H');

// Controller
inline pros::Controller master (pros::E_CONTROLLER_MASTER);

// Right side of drive
inline pros::Motor rF(-1);
inline pros::Motor rM(9);
inline pros::Motor rB(-15);

// Left side of drive
inline pros::Motor lF(-3);
inline pros::Motor lM(-17);
inline pros::Motor lB(18);

// Inertial
inline pros::Imu inertial(12);