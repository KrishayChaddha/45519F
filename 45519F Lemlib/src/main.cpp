#include "robotConfigs.h"
#include "autons.hpp"
#include "distancereset.hpp"
#include "pros/llemu.hpp"
#include "pros/misc.h"
#include "main.h"

/**
 * A callback function for LLEMU's center button.
 *
 * When this callback is fired, it will toggle line 2 of the LCD text between
 * "I was pressed!" and nothing.
 */

 bool isRedAlliance = true; // default to red alliance, but this can be changed in the controller selector
 int selected_side = 0;
 int selected_mode = 0;

void displayRoutineOnBrain() {
	//nothing here for now, but you can put code in here to display stuff on the brain screen without using a separate task if you want
}

void controllerSelector() {
    pros::Controller master(pros::E_CONTROLLER_MASTER);
    const char* side_names[] = {"LEFT ", "RIGHT"};
    const char* mode_names[] = {"3+4"};
    int step = 0; // 0: Color, 1: Side, 2: Mode
    bool confirmed = false;

    while (!confirmed) {
        if (step == 0) { // Color Select
            master.print(0, 0, "ALLIANCE: %s", isRedAlliance ? "RED " : "BLUE");
            if (master.get_digital_new_press(DIGITAL_UP) || master.get_digital_new_press(DIGITAL_DOWN)) isRedAlliance = !isRedAlliance;
            if (master.get_digital_new_press(DIGITAL_A)) { step = 1; master.rumble("."); pros::delay(150); }
        }
        else if (step == 1) { // Side Select
            master.print(0, 0, "SIDE: %s", side_names[selected_side]);
            if (master.get_digital_new_press(DIGITAL_UP) || master.get_digital_new_press(DIGITAL_DOWN)) selected_side = !selected_side;
            if (master.get_digital_new_press(DIGITAL_A)) { step = 2; master.rumble("."); pros::delay(150); }
        }
        else if (step == 2) { // Mode Select
            master.print(0, 0, "MODE: %s", mode_names[selected_mode]);
            if (master.get_digital_new_press(DIGITAL_UP)) selected_mode = (selected_mode + 1) % 4;
            if (master.get_digital_new_press(DIGITAL_DOWN)) selected_mode = (selected_mode - 1 + 4) % 4;
            if (master.get_digital_new_press(DIGITAL_A)) { confirmed = true; master.rumble("-"); }
        }
        pros::delay(50);
    }
}

void on_center_button() {
	static bool pressed = false;
	pressed = !pressed;
	if (pressed) {
		pros::lcd::set_text(2, "I was pressed!");
	} else {
		pros::lcd::clear_line(2);
	}
}

/**
 * Runs initialization code. This occurs as soon as the program is started.
 *
 * All other competition modes are blocked by initialize; it is recommended
 * to keep execution time for this mode under a few seconds.
 */

pros::Controller master(pros::E_CONTROLLER_MASTER);

void printPose() {
    while (true) {
        auto pose = chassis.getPose();

        master.print(1, 0, "X:%.1f", pose.x);
        master.print(0, 0, "Y:%.1f", pose.y);
        master.print(2, 0, "H:%.1f", pose.theta);

        pros::delay(100); // don't spam too fast
    }
}
void initialize() {
    pros::lcd::initialize(); // initialize brain screen
    chassis.calibrate(); // calibrate sensors
    pros::Task poseTask(printPose);

    // the default rate is 50. however, if you need to change the rate, you
    // can do the following.
    // lemlib::bufferedStdout().setRate(...);
    // If you use bluetooth or a wired connection, you will want to have a rate of 10ms

    // for more information on how the formatting for the loggers
    // works, refer to the fmtlib docs

    // thread to for brain screen and position logging
    pros::Task screenTask([&]() {
        while (true) {
            // print robot location to the brain screen
            pros::lcd::print(0, "X: %f", chassis.getPose().x); // x
            pros::lcd::print(1, "Y: %f", chassis.getPose().y); // y
            pros::lcd::print(2, "Theta: %f", chassis.getPose().theta); // heading
            // log position telemetry
            lemlib::telemetrySink()->info("Chassis pose: {}", chassis.getPose());
            // delay to save resources
            pros::delay(50);
        }
    });
}

/**
 * Runs while the robot is disabled
 */
void disabled() {}

/**
 * runs after initialize if the robot is connected to field control
 */
void competition_initialize() {}

// get a path used for pure pursuit
// this needs to be put outside a function

ASSET(example_txt); 

/**
 * Runs during auto
 *
 * This is an example autonomous routine which demonstrates a lot of the features LemLib has to offer
 */
void autonomous() {
    auto_right_7(); }



pros::Controller controller(pros::E_CONTROLLER_MASTER);

void opcontrol() {
    // loop forever
    while (true) {
        // get left y and right x positions
        int leftY = controller.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y);
        int rightX = controller.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_X);

        // move the robot
        chassis.arcade(leftY, rightX);

        // delay to save resources
        pros::delay(25);
    }
}