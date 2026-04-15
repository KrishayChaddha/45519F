#include "main.h"
#include "robotConfigs.h"

// left motor group (put in ports for new robot)
pros::MotorGroup left_motor_group({1, -2, -3}, pros::MotorGears::blue);
// right motor group (put in ports for new robot)
pros::MotorGroup right_motor_group({-8, 9, 7}, pros::MotorGears::blue);

pros::Motor intake(-14, pros::MotorGears::blue); // put in port for new robot's intake motor
pros::Motor lever(16, pros::MotorGears::red); // put in port for new robot's lever motor

pros::adi::Pneumatics funnel('H', false); // put in port for new robot's funnel pneumatic, starts retracted
pros::adi::Pneumatics matchload('F', false); // put in port
pros::adi::Pneumatics descore('G', false); // put in port for new robot's expansion pneumatic, starts retracted
pros::adi::Pneumatics hood('E', false); // put in port for new robot's expansion pneumatic, starts retracted

// drivetrain settings
lemlib::Drivetrain drivetrain(&left_motor_group, // left motor group
                              &right_motor_group, // right motor group
                              10, // 10 inch track width (this is the distance between the left and right wheels, not the distance between the centers of the wheels)
                              lemlib::Omniwheel::NEW_325, // using new 3.25" omnis
                              450, // drivetrain rpm is 450
                              2 // horizontal drift is 2 (for now)
);

// imu
pros::Imu imu(20); // put in the port for the new robot's imu
// horizontal tracking wheel encoder
pros::Rotation horizontal_encoder(11); // put the rotation sensor in the port for the new robot's horizontal tracking wheel
// vertical tracking wheel encoder
pros::Rotation vertical_encoder(-15); // put the rotation sensor in the port for the new robot's vertical tracking wheel
// horizontal tracking wheel

lemlib::TrackingWheel horizontal_tracking_wheel(&horizontal_encoder, lemlib::Omniwheel::NEW_2, 1.00);
// vertical tracking wheel
lemlib::TrackingWheel vertical_tracking_wheel(&vertical_encoder, lemlib::Omniwheel::NEW_2, -4.5);

// odometry settings
lemlib::OdomSensors sensors(nullptr,  //&vertical_tracking_wheel, // vertical tracking wheel 1, set to null
                            nullptr, // vertical tracking wheel 2, set to nullptr as we are using IMEs
                            nullptr, // &horizontal_tracking_wheel, // horizontal tracking wheel 1
                            nullptr, // horizontal tracking wheel 2, set to nullptr as we don't have a second one
                            &imu // inertial sensor
);

// lateral PID controller
lemlib::ControllerSettings lateral_controller(10, // proportional gain (kP)
                                              0, // integral gain (kI)
                                              47, // derivative gain (kD)
                                              3, // anti windup
                                              1, // small error range, in inches
                                              100, // small error range timeout, in milliseconds
                                              7, // large error range, in inches
                                              500, // large error range timeout, in milliseconds
                                              2                                     // maximum acceleration (slew)
);

// angular PID controller
lemlib::ControllerSettings angular_controller(2, // proportional gain (kP) 
                                              0.01, // integral gain (kI)
                                              17, // derivative gain (kD)
                                              3, // anti windup
                                              1, // small error range, in degrees
                                              100, // small error range timeout, in milliseconds
                                              3, // large error range, in degrees
                                              500, // large error range timeout, in milliseconds
                                              0 // maximum acceleration (slew)
);

// create the chassis
lemlib::Chassis chassis(drivetrain, // drivetrain settings
                        lateral_controller, // lateral PID settings
                        angular_controller, // angular PID settings
                        sensors // odometry sensors
);

