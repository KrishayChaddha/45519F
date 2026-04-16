#include "main.h"
#include "lemlib/api.hpp"
#include <cmath>
#include "autonomous.hpp"

// --- Hardware & Globals ---
int selected_side = 0; // 0 = Left, 1 = Right
int selected_mode = 0; // 0 = Rush, 1 = 7-Ring, 2 = 9-Ring
bool isRedAlliance = true;

pros::Controller controller(pros::E_CONTROLLER_MASTER);

// Motors
pros::MotorGroup leftMotors({-1, 2, 3}, pros::MotorGearset::blue); 
pros::MotorGroup rightMotors({8, -9, 7}, pros::MotorGearset::blue);
pros::Motor intake(-14, pros::MotorGearset::blue);
pros::Motor outtake(16, pros::MotorGearset::green);

// Pneumatics
pros::adi::DigitalOut funnel('H');
pros::adi::DigitalOut descore('G');
pros::adi::DigitalOut hood('E');
pros::adi::DigitalOut matchload('F');

// Sensors
pros::Imu imu(20);
pros::Rotation outtake_rot(15); 

// --- LemLib v0.5.x Config ---
lemlib::Drivetrain drivetrain(&leftMotors, &rightMotors, 10.5, lemlib::Omniwheel::NEW_325, 450, 2);

lemlib::ControllerSettings lateral_controller(10, 0, 3, 3, 1, 100, 3, 500, 20);
lemlib::ControllerSettings angular_controller(2, 0, 10, 3, 1, 100, 3, 500, 0);

lemlib::OdomSensors sensors(nullptr, nullptr, nullptr, nullptr, &imu);

lemlib::ExpoDriveCurve throttleCurve(3, 10, 1.019); 
lemlib::ExpoDriveCurve steerCurve(3, 10, 1.019);

lemlib::Chassis chassis(drivetrain, lateral_controller, angular_controller, sensors, &throttleCurve, &steerCurve);

// --- Auton Selector ---
void controllerSelector() {
    // Added Skills and Solo to the list
    const char* side_names[] = {"Left ", "Right", "Solo ", "Skill"}; 
    const char* mode_names[] = {"Rush", "7-Ring", "9-Ring", "None"};
    int step = 0;
    
    while(true) {
        controller.clear();
        pros::delay(50); 
        
        if(step == 0) controller.print(0, 0, "Side: %s", side_names[selected_side]);
        else if(step == 1) controller.print(0, 0, "Mode: %s", mode_names[selected_mode]);

        if(controller.get_digital_new_press(DIGITAL_UP)) {
            if(step == 0) selected_side = (selected_side + 1) % 4; // Cycles through Left, Right, Solo, Skill
            else if(step == 1) selected_mode = (selected_mode + 1) % 3;
        }
        
        if(controller.get_digital_new_press(DIGITAL_A)) {
            step++;
            controller.rumble(".");
            if(step > 1) break; // Finished after choosing Side and Mode
        }
        pros::delay(150);
    }
    controller.print(0, 0, "Locked & Loaded");
}

// --- The Match Brain ---
void autonomous() {
    
    if (selected_side == 0) { // LEFT
        if (selected_mode == 0) left_rush();
        else if (selected_mode == 1) left_seven();
        else if (selected_mode == 2) left_nine();
    } 
    else if (selected_side == 1) { // RIGHT
        if (selected_mode == 0) right_rush();
        else if (selected_mode == 1) right_seven();
        else if (selected_mode == 2) right_nine();
    }
    else if (selected_side == 2) { // SOLO
        solo_autonomous();
    }
    else if (selected_side == 3) { // SKILLS
        skills();
    }
}

void initialize() {
    chassis.calibrate(); 
    controllerSelector();
}

void opcontrol() {
    enum OuttakeState { DOWN, MOVINGUP, UP, MOVINGDOWN };
    OuttakeState outtakeState = DOWN;
    bool descoreState = false;
    bool matchloadState = false;

    while (true) {
        chassis.arcade(controller.get_analog(ANALOG_LEFT_Y), controller.get_analog(ANALOG_RIGHT_X));

        // Outtake State Machine
        double currentPos = outtake_rot.get_position();
        switch (outtakeState) {
            case DOWN:
                outtake.move_voltage(0);
                if (controller.get_digital_new_press(DIGITAL_X)) outtakeState = MOVINGUP;
                break;
            case MOVINGUP:
                outtake.move_voltage(12000);
                if (currentPos >= 24000) outtakeState = UP;
                break;
            case UP:
                outtake.move_voltage(1800); 
                if (controller.get_digital_new_press(DIGITAL_B)) outtakeState = MOVINGDOWN;
                break;
            case MOVINGDOWN:
                outtake.move_voltage(-8000);
                if (currentPos <= 500) outtakeState = DOWN;
                break;
        }

        // Intake (Runs only when outtake is down)
        if (outtakeState == DOWN) {
            if (controller.get_digital(DIGITAL_R1)) intake.move_voltage(12000);
            else if (controller.get_digital(DIGITAL_R2)) intake.move_voltage(-12000);
            else intake.move_voltage(0);
        }

        // Piston Toggles
        if (controller.get_digital_new_press(DIGITAL_UP)) descore.set_value(descoreState = !descoreState);
        if (controller.get_digital_new_press(DIGITAL_LEFT)) matchload.set_value(matchloadState = !matchloadState);

        pros::delay(20);
    }
}