#include "vex.h"

using namespace vex;

//brain and controller
brain Brain;
controller Controller1 = controller(primary);

competition Competition;

// preassigning Variables

// drivetrain Motors
motor LeftFront = motor(PORT1, ratio18_1, false);
motor LeftMiddle = motor(PORT2, ratio18_1, false);
motor LeftBack = motor(PORT3, ratio18_1, false);

motor RightFront = motor(PORT8, ratio18_1, true);
motor RightMiddle = motor(PORT9, ratio18_1, true);
motor RightBack = motor(PORT10, ratio18_1, true);

// Intake Motor
motor Intake = motor(PORT4, ratio18_1, false);

// Pneumatic piston
digital_out Piston = digital_out(Brain.ThreeWirePort.A);

// Inertial sensor
inertial IneritalSensor = inertial(PORT6);

encoder VerticalOdom = encoder(Brain.ThreeWirePort.B);   // vertical odom
encoder HorizontalOdom = encoder(Brain.ThreeWirePort.D); // horizontal odom

// PID Constraints
double kP = 0.45;
double kI = 0.0;
double kD = 0.25;

double error;
double prevError;
double de;
double integral;

// pre-autonomous Function

void pre_auton(void) {
    // calibrate the inertial sensor
    IneritalSensor.calibrate();
    while(IneritalSensor.isCalibrating()) {
        wait(100, msec);
    }

    // reset encoders
    VerticalOdom.setPosition(0, degrees);
    HorizontalOdom.setPosition(0, degrees);

    // set piston initial state (retracted)
    Piston.set(false);
}


// Autonomous Task

void autonomous(void) {
    // Example code
    LeftFront.spin(forward, 50, pct);
    LeftMiddle.spin(forward, 50, pct);
    LeftBack.spin(forward, 50, pct);

    RightFront.spin(forward, 50, pct);
    RightMiddle.spin(forward, 50, pct);
    RightBack.spin(forward, 50, pct);

    wait(2, sec);

    // Stop drivetrain
    LeftFront.stop();
    LeftMiddle.stop();
    LeftBack.stop();
    RightFront.stop();
    RightMiddle.stop();
    RightBack.stop();
}

// User Control Task
void usercontrol(void) {
    while (1) {
        // Tank drive
        LeftFront.spin(forward, Controller1.Axis3.position(), pct);
        LeftMiddle.spin(forward, Controller1.Axis3.position(), pct);
        LeftBack.spin(forward, Controller1.Axis3.position(), pct);

        RightFront.spin(forward, Controller1.Axis2.position(), pct);
        RightMiddle.spin(forward, Controller1.Axis2.position(), pct);
        RightBack.spin(forward, Controller1.Axis2.position(), pct);

        // Intake control
        if (Controller1.ButtonR1.pressing()) {
            Intake.spin(forward, 100, pct);
        } else if (Controller1.ButtonR2.pressing()) {
            Intake.spin(reverse, 100, pct);
        } else {
            Intake.stop();
        }

        // Piston control
        if (Controller1.ButtonL1.pressing()) {
            Piston.set(true);  // Extend piston
        } else if (Controller1.ButtonL2.pressing()) {
            Piston.set(false); // Retract piston
        }

        wait(20, msec);
    }
}

// Main Function
int main() {
    Competition.autonomous(autonomous);
    Competition.drivercontrol(usercontrol);

    pre_auton();

    while(1) {
        wait(100, msec);
    }
}