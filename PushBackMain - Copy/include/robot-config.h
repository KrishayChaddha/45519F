using namespace vex;

extern brain Brain;

//To set up a motor called LeftFront here, you'd use
//extern motor LeftFront;

//Add your devices below, and don't forget to do the same in robot-config.cpp:
extern vex::controller Controller1;

//Drivetrain motors
extern vex::motor LeftFront;
extern vex::motor LeftMiddle;
extern vex::motor LeftBack;
extern vex::motor RightFront;
extern vex::motor RightMiddle;
extern vex::motor RightBack;

//Non-drivetrain motors
extern vex::motor Intake;
extern vex::motor outtake;

//Pneumatic Pistons
extern vex::digital_out Piston;
extern vex::digital_out Piston2;
extern vex::digital_out Piston3;

//Sensors
extern vex::inertial InertialSensor;
extern vex::rotation verticalOdom;
extern vex::rotation armSensor;

void  vexcodeInit( void );