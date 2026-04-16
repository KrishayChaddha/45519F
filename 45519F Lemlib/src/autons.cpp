#include "autons.hpp"
#include "robotConfigs.h"
#include "distancereset.hpp"
#include "lemlib/chassis/chassis.hpp"
#include "pros/rtos.hpp"

void auto_right_7() {

    // Start state
    descore.set_value(true);
    chassis.setPose(47.953, 17.619, 0);

    chassis.moveToPoint(47.953, 47.19, 12000);
    chassis.waitUntilDone();

    matchload.set_value(true);
    pros::delay(150);

    // matchloading
    chassis.turnToHeading(90, 1000);
    chassis.waitUntilDone();

    chassis.setPose(47.953, 47.19, 90);
    chassis.moveToPoint(61.00, 47.19, 10000);
    chassis.waitUntilDone();

    // intake.tare_position();
    intake.move(-127);

    pros::delay(750);
    intake.move(0);

    matchload.set_value(false);
    hood.set_value(true);

    // 
    chassis.moveToPoint(32.028, 47.19, 1100, {.forwards = false, .minSpeed = 100});
    chassis.waitUntilDone();

    // lever cycle (4 block)
    intake.move(-127);
    lever.move(107);
    pros::delay(700);
    lever.move(0);
    intake.move(0);

    lever.move(-127);
    pros::delay(1600);
    lever.move(0);

    
    intake.move(-127);

    // pickup path for blocks
    chassis.moveToPoint(40, 47.19, 1100);
    //chassis.moveToPose(22, 22, 225, 1300);
    chassis.waitUntilDone();

    //chassis.turnToHeading(225, 1000);
   // chassis.waitUntilDone();

    chassis.moveToPose(14, 14, 230, 10000, {.maxSpeed = 60});
    chassis.waitUntilDone();
    intake.move(0);

    // outtake on bottom middle
    intake.move(60);
    pros::delay(500);
    intake.move(0);

    // descore 
    chassis.moveToPoint(32.669, 33.403, 1300, {.forwards = false});
    chassis.waitUntilDone();
    chassis.turnToHeading(-90, 1000);
    chassis.waitUntilDone();

    descore.set_value(false);

    chassis.moveToPoint(11.0, 33.403, 1200);
    chassis.waitUntilDone();

    chassis.turnToHeading(245, 1000);
    chassis.waitUntilDone();

    intake.move(0);
    lever.move(0);

    left_motor_group.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
    right_motor_group.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
}

void auto_left_7() {
    // matchload rush 
    descore.set_value(true);
    chassis.setPose(48.212, -17.72, 0);


    chassis.moveToPoint(48.212, -47.19, 1200);
    chassis.waitUntilDone();

    //matchloading
    matchload.set_value(true);
    pros::delay(150);

    chassis.moveToPose(56.346, -47.19, -90, 1200);
    chassis.waitUntilDone();

    intake.tare_position();
    intake.move(-127);
    pros::delay(600);
    intake.move(0);
    matchload.set_value(false);

    //back up for scoring long goal
    chassis.moveToPoint(32.034, -47.19, 1100);
    chassis.waitUntilDone();

    // lever cycle (4 block)
    lever.move(127);
    pros::delay(450);
    lever.move(0);
    lever.move(-127);
    pros::delay(450);
    lever.move(0);

    intake.move(127);
    // pickup path for blocks
    chassis.moveToPose(22.419, -23.25, -225, 1300);
    chassis.waitUntilDone();

    intake.move(0);
    chassis.moveToPose(13.765, -14.184, 45, 1200);
    chassis.waitUntilDone();

    // outtake top middle
    lever.move(60);
    pros::delay(500);
    lever.move(0);

    lever.move(-127);
    pros::delay(450);
    lever.move(0);

    //descore
    chassis.moveToPose(36.017, -36.711, 90, 1300);
    chassis.waitUntilDone();

    descore.set_value(false);
    chassis.moveToPoint(10.439, -36.711, 1200);
    chassis.waitUntilDone();

}