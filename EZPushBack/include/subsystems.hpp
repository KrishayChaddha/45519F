#pragma once

#include "EZ-Template/api.hpp"
#include "api.h"

extern Drive chassis;

// Your motors, sensors, etc. should go here.  Below are examples

 inline pros::Motor intake(-14, pros::MotorGears::blue);
 inline pros::Motor outtake(16, pros::MotorGears::green, pros::MotorUnits::degrees);
 inline ez::Piston funnel('H');
 inline ez::Piston descore('G');
 inline ez::Piston matchload('F');
 inline pros::Rotation outtake_rotation(13);
 inline ez::Piston hood('E');

 
// inline pros::adi::DigitalIn limit_switch('A');