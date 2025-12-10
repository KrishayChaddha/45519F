#include "vex.h"

using namespace vex;
using signature = vision::signature;
using code = vision::code;

// A global instance of brain used for printing to the V5 Brain screen.
brain  Brain;

//Add your devices below, and don't forget to do the same in robot-config.h:

controller Controller1(primary);

// Drivetrain Motors
motor LeftFront(PORT1, ratio6_1, false);
motor LeftMiddle(PORT2, ratio6_1, true);
motor LeftBack(PORT3, ratio6_1, false);

motor RightFront(PORT8, ratio6_1, true);
motor RightMiddle(PORT9, ratio6_1, false);
motor RightBack(PORT10, ratio6_1, true);

/*motor_group LeftDrive(LeftFront, LeftMiddle, LeftBack);
motor_group RightDrive(RightFront, RightMiddle, RightBack);
drivetrain DriveTrain(LeftDrive, RightDrive, 3.25, 285, 253, mm, 0.6);*/

// Intake Motor
motor Intake(PORT15, ratio18_1, false);

// Outtake Motors
motor outtake(PORT13, ratio18_1, true);

// Pneumatic pistons (both on same port)
digital_out Piston(Brain.ThreeWirePort.H);
digital_out Piston2(Brain.ThreeWirePort.G);
digital_out Piston3(Brain.ThreeWirePort.F);

// Inertial sensor
inertial InertialSensor(PORT20);

// Vertical odometer only
rotation verticalOdom(PORT19);
rotation armSensor(PORT14);

void vexcodeInit( void ) {
  // nothing to initialize
}