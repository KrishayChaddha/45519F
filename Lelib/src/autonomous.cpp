#include "main.h"
#include "autonomous.hpp"
#include "lemlib/api.hpp"

extern lemlib::Chassis chassis;

// LEFT AUTONS
void left_rush() {

}

void left_seven() {
    chassis.moveToPose(12, -47.19, 90, 2000); // Move to the 7-ring position
    
}

void left_nine() {
}
   

// RIGHT AUTONS
void right_rush() {

}

void right_seven() {

}

void right_nine() {

}


// SOLO AUTON
void solo_autonomous() {

}


// SKILLS
void skills() {

}