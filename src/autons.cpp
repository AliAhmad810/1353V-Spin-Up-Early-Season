#include "ALI/catapult.hpp"
#include "EZ-Template/util.hpp"
#include "globals.hpp"
#include "main.h"
#include <cmath>


/////
// For instalattion, upgrading, documentations and tutorials, check out website!
// https://ez-robotics.github.io/EZ-Template/
/////


const int DRIVE_SPEED = 95; // This is 110/127 (around 87% of max speed).  We don't suggest making this 127.
                             // If this is 127 and the robot tries to heading correct, it's only correcting by
                             // making one side slower.  When this is 87%, it's correcting by making one side
                             // faster and one side slower, giving better heading correction.
const int TURN_SPEED  = 80;
const int SWING_SPEED = 80;



///
// Constants
///

// It's best practice to tune constants when the robot is empty and with heavier game objects, or with lifts up vs down.
// If the objects are light or the cog doesn't change much, then there isn't a concern here.

void default_constants() {
  chassis.set_slew_min_power(60, 60);
  chassis.set_slew_distance(12, 8);
  chassis.set_pid_constants(&chassis.headingPID, 11, 0, 20, 0);
  chassis.set_pid_constants(&chassis.forward_drivePID, 0.47, 0, 6.5, 0);
  chassis.set_pid_constants(&chassis.backward_drivePID, 0.45, 0, 6.6, 0);
  chassis.set_pid_constants(&chassis.turnPID, 5, 0.003, 35, 15);
  chassis.set_pid_constants(&chassis.swingPID, 10, 0, 50, 0);
}

void exit_condition_defaults() {
  chassis.set_exit_condition(chassis.turn_exit, 50, 3, 500, 7, 500, 500);
  chassis.set_exit_condition(chassis.swing_exit, 50, 3, 500, 7, 500, 500);
  chassis.set_exit_condition(chassis.drive_exit, 80, 50, 300, 150, 500, 500);
}

void modified_exit_condition() {
  chassis.set_exit_condition(chassis.turn_exit, 100, 3, 500, 7, 500, 500);
  chassis.set_exit_condition(chassis.swing_exit, 100, 3, 500, 7, 500, 500);
  chassis.set_exit_condition(chassis.drive_exit, 80, 50, 300, 150, 500, 500);
}



///
// Drive Example
///
void drive_example() {
  // The first parameter is target inches
  // The second parameter is max speed the robot will drive at
  // The third parameter is a boolean (true or false) for enabling/disabling a slew at the start of drive motions
  // for slew, only enable it when the drive distance is greater then the slew distance + a few inches


  chassis.set_drive_pid(150, DRIVE_SPEED, true);
  chassis.wait_drive();

  // rF.move_voltage(12000);
  // rM.move_voltage(12000);
  // rB.move_voltage(12000);
  // lF.move_voltage(12000);
  // lM.move_voltage(12000);
  // lB.move_voltage(12000);
  // pros::delay(10000);
  // rF.move_voltage(0);
  // rM.move_voltage(0);
  // rB.move_voltage(0);
  // lF.move_voltage(0);
  // lM.move_voltage(0);
  // lB.move_voltage(0);
  // pros::delay(1000);
  
  //chassis.set_drive_pid(-24, DRIVE_SPEED);
  //chassis.wait_drive();
  /*
  chassis.set_drive_pid(-12, DRIVE_SPEED);
  chassis.wait_drive();
  */
}



///
// Turn Example
///
void turn_example() {
  // The first parameter is target degrees
  // The second parameter is max speed the robot will drive at


  chassis.set_turn_pid(90, TURN_SPEED);
  chassis.wait_drive();
  chassis.set_turn_pid(90, TURN_SPEED);
  chassis.wait_drive();
  chassis.set_turn_pid(90, TURN_SPEED);
  chassis.wait_drive();
  chassis.set_turn_pid(90, TURN_SPEED);
  chassis.wait_drive();

  //chassis.set_turn_pid(0, TURN_SPEED);
  //chassis.wait_drive();
}



///
// Combining Turn + Drive
///
void drive_and_turn() {
  chassis.set_drive_pid(24, DRIVE_SPEED, true);
  chassis.wait_drive();

  chassis.set_turn_pid(45, TURN_SPEED);
  chassis.wait_drive();

  chassis.set_turn_pid(-45, TURN_SPEED);
  chassis.wait_drive();

  chassis.set_turn_pid(0, TURN_SPEED);
  chassis.wait_drive();

  chassis.set_drive_pid(-24, DRIVE_SPEED, true);
  chassis.wait_drive();
}



///
// Wait Until and Changing Max Speed
///
void wait_until_change_speed() {
  // wait_until will wait until the robot gets to a desired position


  // When the robot gets to 6 inches, the robot will travel the remaining distance at a max speed of 40
  chassis.set_drive_pid(24, DRIVE_SPEED, true);
  chassis.wait_until(6);
  chassis.set_max_speed(40); // After driving 6 inches at DRIVE_SPEED, the robot will go the remaining distance at 40 speed
  chassis.wait_drive();

  chassis.set_turn_pid(45, TURN_SPEED);
  chassis.wait_drive();

  chassis.set_turn_pid(-45, TURN_SPEED);
  chassis.wait_drive();

  chassis.set_turn_pid(0, TURN_SPEED);
  chassis.wait_drive();

  // When the robot gets to -6 inches, the robot will travel the remaining distance at a max speed of 40
  chassis.set_drive_pid(-24, DRIVE_SPEED, true);
  chassis.wait_until(-6);
  chassis.set_max_speed(40); // After driving 6 inches at DRIVE_SPEED, the robot will go the remaining distance at 40 speed
  chassis.wait_drive();
}



///
// Swing Example
///
void swing_example() {
  // The first parameter is ez::LEFT_SWING or ez::RIGHT_SWING
  // The second parameter is target degrees
  // The third parameter is speed of the moving side of the drive


  chassis.set_swing_pid(ez::LEFT_SWING, 45, SWING_SPEED);
  chassis.wait_drive();

  chassis.set_drive_pid(24, DRIVE_SPEED, true);
  chassis.wait_until(12);

  chassis.set_swing_pid(ez::RIGHT_SWING, 0, SWING_SPEED);
  chassis.wait_drive();
}



///
// Auto that tests everything
///
void combining_movements() {
  chassis.set_drive_pid(24, DRIVE_SPEED, true);
  chassis.wait_drive();

  chassis.set_turn_pid(45, TURN_SPEED);
  chassis.wait_drive();

  chassis.set_swing_pid(ez::RIGHT_SWING, -45, TURN_SPEED);
  chassis.wait_drive();

  chassis.set_turn_pid(0, TURN_SPEED);
  chassis.wait_drive();

  chassis.set_drive_pid(-24, DRIVE_SPEED, true);
  chassis.wait_drive();
}



///
// Interference example
///
void tug (int attempts) {
  for (int i=0; i<attempts-1; i++) {
    // Attempt to drive backwards
    printf("i - %i", i);
    chassis.set_drive_pid(-12, 127);
    chassis.wait_drive();

    // If failsafed...
    if (chassis.interfered) {
      chassis.reset_drive_sensor();
      chassis.set_drive_pid(-2, 20);
      pros::delay(1000);
    }
    // If robot successfully drove back, return
    else {
      return;
    }
  }
}

// If there is no interference, robot will drive forward and turn 90 degrees. 
// If interfered, robot will drive forward and then attempt to drive backwards. 
void interfered_example() {
 chassis.set_drive_pid(24, DRIVE_SPEED, true);
 chassis.wait_drive();

 if (chassis.interfered) {
   tug(3);
   return;
 }

 chassis.set_turn_pid(90, TURN_SPEED);
 chassis.wait_drive();
}



// . . .
// Make your own autonomous functions here!
// . . .

// RIGHT SIDE

void six_disc_right_side() {
  // Start intaking
  intake.move_voltage(-12000);

  // Drive forward to disc
  chassis.set_drive_pid(42, DRIVE_SPEED, true);
  chassis.wait_drive();

  // Turn back to face goal
  chassis.set_turn_pid(45, TURN_SPEED);
  chassis.wait_drive();
  chassis.set_turn_pid(45, 0);

  // Shoot
  ali::cata::toggle_cata();

  // Drive to barrier discs
  chassis.set_drive_pid(30, DRIVE_SPEED-20, true);
  chassis.wait_until(25);
  chassis.set_swing_pid(RIGHT_SWING, 0, SWING_SPEED);
  chassis.wait_drive();

  // Drive forward to intake discs
  chassis.set_drive_pid(28, DRIVE_SPEED-20, true);
  chassis.wait_drive();

  // Drive back to shooting pos
  chassis.set_drive_pid(-28, DRIVE_SPEED, true);
  chassis.wait_drive();

  chassis.set_swing_pid(RIGHT_SWING, 45, SWING_SPEED);
  chassis.wait_drive();

  chassis.set_drive_pid(-30, DRIVE_SPEED, true);
}

void eight_disc_right_side() {

}

// LEFT SIDE

void five_disc_left_side() {

}

void six_disc_left_side() {

}

// AWP

void five_disc_solo_AWP() {

}

void eight_disc_solo_AWP() {

}

// SKILLS

void twenty_four_disc_skills_auto() {

}

void thirty_disc_skills_auto() {
  // Move forward into roller
  chassis.set_drive_pid(2,70, false);
  chassis.wait_drive();

  // Turn roller 
  // 10 pts
  intake.move_voltage(12000);
  pros::delay(300);
  intake.move_voltage(0);
  
  ///////////////////////////////////////////////////////////////

  // Move backward
  chassis.set_drive_pid(-4, DRIVE_SPEED-20, true);
  chassis.wait_drive(); 

  // Turn to roller 2
  chassis.set_turn_pid(125, TURN_SPEED);
  chassis.wait_drive();
  chassis.set_turn_pid(125, 0);
  

  // Start intaking
  intake.move_voltage(-12000);

  // Move forward into disc, then into roller
  chassis.set_drive_pid(35, DRIVE_SPEED-25, true);
  chassis.wait_until(28);

  // Stop intaking 
  intake.move_voltage(0);

  chassis.wait_drive();

  pros::delay(100);

  // Turn roller
  // 20 pts
  intake.move_voltage(12000);
  pros::delay(300);
  intake.move_voltage(0);

  ////////////////////////////////////////////////////////////////////

  // Move backward to line up to red goal
  chassis.set_drive_pid(-2, DRIVE_SPEED-20, false);
  chassis.wait_drive();

  // Intake more just in case
  intake.move_voltage(-12000);

  // Turn so cata faces red goal
  chassis.set_turn_pid(2, TURN_SPEED);
  chassis.wait_drive();
  chassis.set_turn_pid(0,0);

  // Drive to red goal
  chassis.set_drive_pid(-57.5, DRIVE_SPEED, true);
  chassis.wait_drive();

  // Stop intake
  intake.move_voltage(0);

  // Turn slightly
  chassis.set_turn_pid(6.7, 50);

  // Shoot cata and reload 
  // 35 pts
  ali::cata::toggle_cata();
  pros::delay(100);
  ali::cata::shoot = false;

  ///////////////////////////////////////////////////////////////////

  // Turn to barrier discs
  chassis.set_turn_pid(-90, TURN_SPEED);
  chassis.wait_drive();

  // Start intaking
  intake.move_voltage(-12000);

  // Drive forward to intake discs
  chassis.set_drive_pid(30, DRIVE_SPEED-40, true);
  chassis.wait_drive();

  // Drive back to pos
  chassis.set_drive_pid(-32, DRIVE_SPEED, true);
  chassis.wait_drive();

  // Turn to shoot
  chassis.set_turn_pid(5, TURN_SPEED);
  chassis.wait_drive();

  // Shoot cata and reload 
  // 50 pts
  ali::cata::toggle_cata();
  pros::delay(100);
  ali::cata::shoot = false;

  ///////////////////////////////////////////////////////////////////
  
  // Turn to discs in the middle
  chassis.set_turn_pid(-35, TURN_SPEED);
  chassis.wait_drive();
  chassis.set_turn_pid(-35, 0);

  // Start intaking
  intake.move_voltage(-12000);

  // Drive to first disc in line of 3
  chassis.set_drive_pid(32, DRIVE_SPEED-25, true);
  chassis.wait_drive();

  // Turn to face the other 2 discs
  chassis.set_turn_pid(-135, TURN_SPEED);
  chassis.wait_drive();
  chassis.set_turn_pid(-135, 0);

  // Drive forward and intake the 2 discs
  chassis.set_drive_pid(50, DRIVE_SPEED-30, true);
  chassis.wait_drive();

  // Turn to drive to red goal
  chassis.set_turn_pid(-20, TURN_SPEED-10);
  chassis.wait_drive();
  chassis.set_turn_pid(-20, 0);

  // Drive to red goal
  chassis.set_drive_pid(-32, DRIVE_SPEED, true);
  chassis.wait_drive();

  // Turn to align cata to goal
  chassis.set_turn_pid(-90, TURN_SPEED-10);
  chassis.wait_drive();
  chassis.set_turn_pid(-90, 0);

  // Drive a tiny bit to the goal
  chassis.set_drive_pid(-5, DRIVE_SPEED, true);
  chassis.wait_drive();

  // Stop intaking 
  intake.move_voltage(0);

  // Shoot 
  // 75 pts
  ali::cata::toggle_cata();
  pros::delay(100);
  ali::cata::shoot = false;
}


// OLD

void solo_AWP_old() {

  // Drive into first roller
  chassis.set_drive_pid(7, DRIVE_SPEED, true);
  chassis.wait_drive();

  // Spin first roller
  intake.move_voltage(7500);
  pros::delay(500);
  intake.move_voltage(0);

  // Move back a tiny bit
  chassis.set_drive_pid(-7, DRIVE_SPEED, true);
  chassis.wait_drive();
  pros::delay(400);

  // Turn back to face barrier
  chassis.set_turn_pid(92, SWING_SPEED);
  chassis.wait_drive();
  chassis.set_turn_pid(90, 0);
  pros::delay(10);

  // Drive towards barrier
  chassis.set_drive_pid(-44, DRIVE_SPEED, true);
  chassis.wait_drive();

  // Turn towards middle
  chassis.set_turn_pid(0, TURN_SPEED);
  chassis.wait_drive();

  // Drive to middle
  chassis.set_drive_pid(-48, DRIVE_SPEED, true);
  chassis.wait_drive();

  // Turn towards goal
  chassis.set_turn_pid(-41, TURN_SPEED);
  chassis.wait_drive();
  chassis.set_turn_pid(-41, 0);
  pros::delay(10);

  // Shoot discs at goal
  catapult.move_voltage(11000);
  pros::delay(1500);
  catapult.move_voltage(0);
  //pistonBoost.set_value(false);

  chassis.set_drive_pid(14,DRIVE_SPEED, true);
  chassis.wait_drive();

  // Turn towards second roller
  pros::delay(1000);
  chassis.set_turn_pid(120, TURN_SPEED);
  chassis.wait_drive();
  chassis.set_turn_pid(120, 0);

  // Outtake
  intake.move_voltage(11000);

  // Drive towards roller
  chassis.set_drive_pid(60, DRIVE_SPEED, true);
  chassis.wait_drive();

  // Swing a tiny bit both ways
  chassis.set_turn_pid(180, SWING_SPEED);
  chassis.wait_drive();
  chassis.set_turn_pid(0, 0);
  chassis.set_drive_pid(12, DRIVE_SPEED, true);
  chassis.wait_drive();
  chassis.set_turn_pid(90, TURN_SPEED);
  chassis.wait_drive();
  //chassis.set_swing_pid(LEFT_SWING, 80, TURN_SPEED);
  //chassis.wait_drive();
  //chassis.set_swing_pid(LEFT_SWING, 80, 0);

  // Spin second roller
  intake.move_voltage(7000);
  pros::delay(500);
  intake.move_voltage(0);
}

void two_disc_solo_AWP() {
  // Move forward into roller and spin roller, move back 
  chassis.set_drive_pid(4,DRIVE_SPEED, true);
  chassis.wait_drive();

  intake.move_voltage(-8000);
  pros::delay(200);
  intake.move_voltage(0);

  chassis.set_drive_pid(-14, DRIVE_SPEED, true);
  chassis.wait_drive(); 

  // Turn towards middle of the field, then drive to it
  chassis.set_turn_pid(44, TURN_SPEED);
  chassis.wait_drive();
  chassis.set_turn_pid(44,0);

  chassis.set_drive_pid(-62, DRIVE_SPEED, true);
  chassis.wait_drive();
  
  // Turn to aim, shoot, load, shoot
  chassis.set_turn_pid(-46, TURN_SPEED);
  chassis.wait_drive();
  chassis.set_turn_pid(-46, 0);
  chassis.wait_drive();

  pros::delay(300);

  catapult.move_voltage(12000);
  pros::delay(500);
  catapult.move_voltage(0);

  // Turn towards roller, drive, turn
  chassis.set_turn_pid(-135, TURN_SPEED);
  chassis.wait_drive();
  chassis.set_turn_pid(-135, 0);

  chassis.set_drive_pid(62, DRIVE_SPEED, true);
  chassis.wait_drive();

  chassis.set_turn_pid(-90, TURN_SPEED);
  chassis.wait_drive();
  chassis.set_turn_pid(-90, 0);
  
  // Mofe forward 
  chassis.set_drive_pid(14, 80);
  chassis.wait_drive();

  // Turn roller
  intake.move_voltage(-7500);
  pros::delay(500);
  intake.move_voltage(0);

}

void two_disc_right_side() {
  chassis.set_drive_pid(-40, DRIVE_SPEED, true);
  chassis.wait_drive();

  chassis.set_turn_pid(36, TURN_SPEED-30);
  chassis.wait_drive();
  chassis.set_turn_pid(36, 0);

  catapult.move_voltage(12000);
  pros::delay(500);
  while(!cataLimit.get_value()) {
    catapult.move_voltage(12000);
  }
  catapult.move_voltage(0);

  intake.move_voltage(-12000);

  chassis.set_turn_pid(-45, TURN_SPEED-30);
  chassis.wait_drive();
  chassis.set_turn_pid(-45, 0);

  chassis.set_drive_pid(40, DRIVE_SPEED-10, true);
  chassis.wait_drive();

  chassis.set_turn_pid(0, TURN_SPEED-30);
  chassis.wait_drive();
  chassis.set_turn_pid(0, 0);

  intake.move_voltage(0);

  chassis.set_drive_pid(17.5, DRIVE_SPEED-45, false);

  chassis.wait_drive();

  intake.move_voltage(0);

  intake.move_voltage(11000);
  pros::delay(250);
  intake.move_voltage(0);

  //bandSlip.set_value(true);
  pros::delay(1000);
  bandSlip.set_value(false);
  
}

void twelve_disc_skills_auto() {
  // Move forward into roller
  chassis.set_drive_pid(2,70, false);
  chassis.wait_drive();

  // Turn roller 
  // 10 pts
  intake.move_voltage(12000);
  pros::delay(400);
  intake.move_voltage(0);
  
  ///////////////////////////////////////////////////////////////

  // Move backward
  chassis.set_drive_pid(-6, DRIVE_SPEED-25, true);
  chassis.wait_drive(); 

  // Turn to roller 2
  chassis.set_turn_pid(125, TURN_SPEED);
  chassis.wait_drive();
  chassis.set_turn_pid(125, 0);
  

  // Start intaking
  intake.move_voltage(-12000);

  // Move forward into disc, then into roller
  chassis.set_drive_pid(34, DRIVE_SPEED-30, true);
  chassis.wait_drive();

  pros::delay(100);

  // Stop intake
  // 20 pts
  intake.move_voltage(0);

  ////////////////////////////////////////////////////////////////////

  // Move backward to line up to red goal
  chassis.set_drive_pid(-5, DRIVE_SPEED-30, false);
  chassis.wait_drive();

  // Intake more just in case
  intake.move_voltage(-12000);

  // Turn so cata faces red goal
  chassis.set_turn_pid(2, TURN_SPEED);
  chassis.wait_drive();
  chassis.set_turn_pid(0,0);

  // Drive to red goal
  chassis.set_drive_pid(-59, DRIVE_SPEED, true);
  chassis.wait_drive();

  // Stop intake
  intake.move_voltage(0);

  // Shoot cata and reload 
  // 35 pts
  catapult.move_voltage(12000);
  pros::delay(500);
  while(!cataLimit.get_value()) {
    catapult.move_voltage(12000);
  }
  catapult.move_voltage(0);

  ///////////////////////////////////////////////////////////////////

  // Turn to discs in the middle
  chassis.set_turn_pid(-35, TURN_SPEED);
  chassis.wait_drive();
  chassis.set_turn_pid(-35, 0);

  // Start intaking
  intake.move_voltage(-12000);

  // Drive to first disc in line of 3
  chassis.set_drive_pid(32, DRIVE_SPEED-25, true);
  chassis.wait_drive();

  // Turn to face the other 2 discs
  chassis.set_turn_pid(-135, TURN_SPEED);
  chassis.wait_drive();
  chassis.set_turn_pid(-135, 0);

  // Drive forward and intake the 2 discs
  chassis.set_drive_pid(50, DRIVE_SPEED-30, true);
  chassis.wait_drive();

  // Turn to drive to red goal
  chassis.set_turn_pid(-20, TURN_SPEED-10);
  chassis.wait_drive();
  chassis.set_turn_pid(-20, 0);

  // Drive to red goal
  chassis.set_drive_pid(-32, DRIVE_SPEED, true);
  chassis.wait_drive();

  // Turn to align cata to goal
  chassis.set_turn_pid(-90, TURN_SPEED-10);
  chassis.wait_drive();
  chassis.set_turn_pid(-90, 0);

  // Drive a tiny bit to the goal
  chassis.set_drive_pid(-5, DRIVE_SPEED, true);
  chassis.wait_drive();

  // Stop intaking 
  intake.move_voltage(0);

  // Shoot 
  // 50 pts
  catapult.move_voltage(12000);
  pros::delay(500);
  while(!cataLimit.get_value()) {
    catapult.move_voltage(12000);
  }
  catapult.move_voltage(0);

  ///////////////////////////////////////////////////////////////////

  // Turn to middle disc
  chassis.set_turn_pid(-32, TURN_SPEED-10);
  chassis.wait_drive();
  chassis.set_turn_pid(-32, 0);

  // Start intaking
  intake.move_voltage(-12000);

  // Move forward
  chassis.set_drive_pid(42, DRIVE_SPEED-10);
  chassis.wait_drive();

  // Turn to other discs
  chassis.set_turn_pid(-135, TURN_SPEED-5);
  chassis.wait_drive();
  chassis.set_turn_pid(-135, 0);

  // Drive forward to intake 1 disc
  chassis.set_drive_pid(16, DRIVE_SPEED-15, true);
  chassis.wait_drive();

  // Turn to face roller
  chassis.set_turn_pid(-169, TURN_SPEED);
  chassis.wait_drive();
  chassis.set_turn_pid(-169, 0);

  // Stop intaking
  intake.move_voltage(0);

  // Drive towards roller
  chassis.set_drive_pid(38, DRIVE_SPEED-20, true);
  chassis.wait_drive();

  // Swing
  chassis.set_swing_pid(RIGHT_SWING, -181, 70);
  chassis.wait_drive();

  pros::delay(200);

  // Turn roller 
  // 60 pts
  intake.move_voltage(-12000);
  pros::delay(400);
  intake.move_voltage(0);

  pros::delay(200);

  // Swing back
  chassis.set_swing_pid(RIGHT_SWING, -170, 70);
  chassis.wait_drive();

  ////////////////////////////////////////////////////////////////

  // Move backward
  chassis.set_drive_pid(-6, DRIVE_SPEED-20, true);
  chassis.wait_drive(); 

  // Turn to roller 2
  chassis.set_turn_pid(-60, TURN_SPEED);
  chassis.wait_drive();
  chassis.set_turn_pid(-60, 0);
  

  // Start intaking
  intake.move_voltage(-12000);

  // Move forward into disc, then into roller
  chassis.set_drive_pid(36, DRIVE_SPEED-40, true);
  chassis.wait_drive();

  // Stop intake
  // 70 pts
  intake.move_voltage(0);

  ////////////////////////////////////////////////////////////////////

  // Move backward to line up to blue goal
  chassis.set_drive_pid(-5, DRIVE_SPEED-30, false);
  chassis.wait_drive();

  // Start intake again
  intake.move_voltage(-12000);

  // Turn so cata faces blue goal
  chassis.set_turn_pid(182, TURN_SPEED);
  chassis.wait_drive();
  chassis.set_turn_pid(182,0);

  // Drive to blue goal
  chassis.set_drive_pid(-62, DRIVE_SPEED, true);
  chassis.wait_drive();

  // Shoot cata and reload 
  // 85 pts
  catapult.move_voltage(12000);
  pros::delay(500);
  while(!cataLimit.get_value()) {
    catapult.move_voltage(12000);
  }
  catapult.move_voltage(0);

  // Stop intake
  intake.move_voltage(0);
  
  ////////////////////////////////////////////////////////////////////

    // Turn to discs in the middle
  chassis.set_turn_pid(145, TURN_SPEED);
  chassis.wait_drive();
  chassis.set_turn_pid(145, 0);

  // Start intaking
  intake.move_voltage(-12000);

  // Drive to first disc in line of 3
  chassis.set_drive_pid(32, DRIVE_SPEED-25, true);
  chassis.wait_drive();

  // Turn to face the other 2 discs
  chassis.set_turn_pid(41, TURN_SPEED);
  chassis.wait_drive();
  chassis.set_turn_pid(41, 0);

  // Drive forward and intake the 2 discs
  chassis.set_drive_pid(50, DRIVE_SPEED-40, true);
  chassis.wait_drive();

  // Turn to drive to blue goal
  chassis.set_turn_pid(160, TURN_SPEED-10);
  chassis.wait_drive();
  chassis.set_turn_pid(160, 0);

  // Drive to blue goal
  chassis.set_drive_pid(-32, DRIVE_SPEED, true);
  chassis.wait_drive();

  // Turn to align cata to goal
  chassis.set_turn_pid(90, TURN_SPEED-10);
  chassis.wait_drive();
  chassis.set_turn_pid(90, 0);

  // Drive a tiny bit to the goal
  chassis.set_drive_pid(-5, DRIVE_SPEED, true);
  chassis.wait_drive();

  // Stop intaking 
  intake.move_voltage(0);

  // Shoot 
  // 100 pts
  catapult.move_voltage(12000);
  pros::delay(500);
  while(!cataLimit.get_value()) {
    catapult.move_voltage(12000);
  }
  catapult.move_voltage(0);

  //////////////////////////////////////////////////////////////

  // Drive to corner
  chassis.set_drive_pid(66, DRIVE_SPEED-10, true);
  chassis.wait_drive();

  // Turn to expand
  chassis.set_turn_pid(45, TURN_SPEED-20);
  chassis.wait_drive();
  chassis.set_turn_pid(45, 0);

  // expand
  expansion.set_value(true);\

}

void do_nothing() { 
  
} 