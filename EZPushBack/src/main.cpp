#include "main.h"

/////
// For installation, upgrading, documentations, and tutorials, check out our website!
// https://ez-robotics.github.io/EZ-Template/
/////

// Chassis constructor
ez::Drive chassis(
    // These are your drive motors, the first motor is used for sensing!
    {-1, 2, -3},     // Left Chassis Ports (negative port will reverse it!)
    {8, -9, 10},  // Right Chassis Ports (negative port will reverse it!)

    20,      // IMU Port
    3.25,  // Wheel Diameter (Remember, 4" wheels without screw holes are actually 4.125!)
    450);   // Wheel RPM = cartridge * (motor gear / wheel gear)

// Uncomment the trackers you're using here!
// - `8` and `9` are smart ports (making these negative will reverse the sensor)
// - you should get positive values on the encoders going FORWARD and RIGHT
// - `2.75` is the wheel diameter
// - `4.0` is the distance from the center of the wheel to the center of the robot
// ez::tracking_wheel horiz_tracker(8, 2.75, 4.0);  // This tracking wheel is perpendicular to the drive wheels
ez::tracking_wheel vert_tracker(-19, 3.25, 0);   // This tracking wheel is parallel to the drive wheels

/**
 * Runs initialization code. This occurs as soon as the program is started.
 *
 * All other competition modes are blocked by initialize; it is recommended
 * to keep execution time for this mode under a few seconds.
 */
void initialize() {
    ez::ez_template_print();

    pros::delay(500);

    chassis.odom_tracker_right_set(&vert_tracker);

    chassis.opcontrol_curve_buttons_toggle(true);
    chassis.opcontrol_drive_activebrake_set(2);
    chassis.opcontrol_curve_default_set(0.0, 0.0);

    default_constants();

    // Autonomous Selector
    ez::as::auton_selector.autons_add({
        {"Solo Auton\n\nThis will run solo_auton", solo_auton},
        {"Right Side Auto\n\nRight side auton with matchload", right_side},
        {"Left Side Auto\n\nLeft side auton with matchload", left_side},
        {"Solo Auton Fail\n\n This one fails a lot", sawp_autonfail},
        {"Drive\n\nDrive forward and come back", drive_example},
        {"Turn\n\nTurn 3 times.", turn_example},
        {"Drive and Turn\n\nDrive forward, turn, come back", drive_and_turn},
        {"Swing Turn\n\nSwing in an 'S' curve", swing_example},
        {"Motion Chaining\n\nDrive forward, turn, and come back, but blend everything together :D", motion_chaining},
        {"Combine all 3 movements", combining_movements},
        {"Interference\n\nAfter driving forward, robot performs differently if interfered or not", interfered_example},
        {"Simple Odom\n\nThis is the same as the drive example, but it uses odom instead!", odom_drive_example},
        {"Pure Pursuit\n\nGo to (0, 30) and pass through (6, 10) on the way.  Come back to (0, 0)", odom_pure_pursuit_example},
        {"Pure Pursuit Wait Until\n\nGo to (24, 24) but start running an intake once the robot passes (12, 24)", odom_pure_pursuit_wait_until_example},
        {"Boomerang\n\nGo to (0, 24, 45) then come back to (0, 0, 0)", odom_boomerang_example},
        {"Boomerang Pure Pursuit\n\nGo to (0, 24, 45) on the way to (24, 24) then come back to (0, 0, 0)", odom_boomerang_injected_pure_pursuit_example},
        {"Measure Offsets\n\nThis will turn the robot a bunch of times and calculate your offsets for your tracking wheels.", measure_offsets}
    });

    // Initialize chassis and auton selector
    chassis.initialize();
    ez::as::initialize();

    // Rumble feedback (make sure this is inside the function)
    master.rumble(chassis.drive_imu_calibrated() ? "." : "---");
}


/**
 * Runs while the robot is in the disabled state of Field Management System or
 * the VEX Competition Switch, following either autonomous or opcontrol. When
 * the robot is enabled, this task will exit.
 */
void disabled() {
  // . . .
}

/**
 * Runs after initialize(), and before autonomous when connected to the Field
 * Management System or the VEX Competition Switch. This is intended for
 * competition-specific initialization routines, such as an autonomous selector
 * on the LCD.
 *
 * This task will exit when the robot is enabled and autonomous or opcontrol
 * starts.
 */
void competition_initialize() {
  // . . .
}

/**
 * Runs the user autonomous code. This function will be started in its own task
 * with the default priority and stack size whenever the robot is enabled via
 * the Field Management System or the VEX Competition Switch in the autonomous
 * mode. Alternatively, this function may be called in initialize or opcontrol
 * for non-competition testing purposes.
 *
 * If the robot is disabled or communications is lost, the autonomous task
 * will be stopped. Re-enabling the robot will restart the task, not re-start it
 * from where it left off.
 */
void autonomous() {
  chassis.pid_targets_reset();                // Resets PID targets to 0
  chassis.drive_imu_reset();                  // Reset gyro position to 0
  chassis.drive_sensor_reset();               // Reset drive sensors to 0
  chassis.odom_xyt_set(0_in, 0_in, 0_deg);    // Set the current position, you can start at a specific position with this
  chassis.drive_brake_set(MOTOR_BRAKE_HOLD);  // Set motors to hold.  This helps autonomous consistency

  /*
  Odometry and Pure Pursuit are not magic

  It is possible to get perfectly consistent results without tracking wheels,
  but it is also possible to have extremely inconsistent results without tracking wheels.
  When you don't use tracking wheels, you need to:
   - avoid wheel slip
   - avoid wheelies
   - avoid throwing momentum around (super harsh turns, like in the example below)
  You can do cool curved motions, but you have to give your robot the best chance
  to be consistent
  */

  ez::as::auton_selector.selected_auton_call();  // Calls selected auton from autonomous selector
}

/**
 * Simplifies printing tracker values to the brain screen
 */
void screen_print_tracker(ez::tracking_wheel *tracker, std::string name, int line) {
  std::string tracker_value = "", tracker_width = "";
  // Check if the tracker exists
  if (tracker != nullptr) {
    tracker_value = name + " tracker: " + util::to_string_with_precision(tracker->get());             // Make text for the tracker value
    tracker_width = "  width: " + util::to_string_with_precision(tracker->distance_to_center_get());  // Make text for the distance to center
  }
  ez::screen_print(tracker_value + tracker_width, line);  // Print final tracker text
}

/**
 * Ez screen task
 * Adding new pages here will let you view them during user control or autonomous
 * and will help you debug problems you're having
 */
void ez_screen_task() {
  while (true) {
    // Only run this when not connected to a competition switch
    if (!pros::competition::is_connected()) {
      // Blank page for odom debugging
      if (chassis.odom_enabled() && !chassis.pid_tuner_enabled()) {
        // If we're on the first blank page...
        if (ez::as::page_blank_is_on(0)) {
          // Display X, Y, and Theta
          ez::screen_print("x: " + util::to_string_with_precision(chassis.odom_x_get()) +
                               "\ny: " + util::to_string_with_precision(chassis.odom_y_get()) +
                               "\na: " + util::to_string_with_precision(chassis.odom_theta_get()),
                           1);  // Don't override the top Page line

          // Display all trackers that are being used
          screen_print_tracker(chassis.odom_tracker_left, "l", 4);
          screen_print_tracker(chassis.odom_tracker_right, "r", 5);
          screen_print_tracker(chassis.odom_tracker_back, "b", 6);
          screen_print_tracker(chassis.odom_tracker_front, "f", 7);
        }
      }
    }

    // Remove all blank pages when connected to a comp switch
    else {
      if (ez::as::page_blank_amount() > 0)
        ez::as::page_blank_remove_all();
    }

    pros::delay(ez::util::DELAY_TIME);
  }
}
pros::Task ezScreenTask(ez_screen_task);

/**
 * Gives you some extras to run in your opcontrol:
 * - run your autonomous routine in opcontrol by pressing DOWN and B
 *   - to prevent this from accidentally happening at a competition, this
 *     is only enabled when you're not connected to competition control.
 * - gives you a GUI to change your PID values live by pressing X
 */
void ez_template_extras() {
  // Only run this when not connected to a competition switch
  if (!pros::competition::is_connected()) {
    // PID Tuner
    // - after you find values that you're happy with, you'll have to set them in auton.cpp

    // Enable / Disable PID Tuner
    //  When enabled:
    //  * use A and Y to increment / decrement the constants
    //  * use the arrow keys to navigate the constants
    if (master.get_digital_new_press(DIGITAL_X))
      chassis.pid_tuner_toggle();

    // Trigger the selected autonomous routine
    if (master.get_digital(DIGITAL_B) && master.get_digital(DIGITAL_DOWN)) {
      pros::motor_brake_mode_e_t preference = chassis.drive_brake_get();
      autonomous();
      chassis.drive_brake_set(preference);
    }

    // Allow PID Tuner to iterate
    chassis.pid_tuner_iterate();
  }

  // Disable PID Tuner when connected to a comp switch
  else {
    if (chassis.pid_tuner_enabled())
      chassis.pid_tuner_disable();
  }
}

/**
 * Runs the operator control code. This function will be started in its own task
 * with the default priority and stack size whenever the robot is enabled via
 * the Field Management System or the VEX Competition Switch in the operator
 * control mode.
 *
 * If no competition control is connected, this function will run immediately
 * following initialize().
 *
 * If the robot is disabled or communications is lost, the
 * operator control task will be stopped. Re-enabling the robot will restart the
 * task, not resume it from where it left off.
 */
void opcontrol() {
  // This is preference to what you like to drive on
  chassis.drive_brake_set(MOTOR_BRAKE_COAST);
  enum outtakeStates {DOWN, UP, MOVING};
  enum pistonStates {RETRACTED, EXTENDED};
  pistonStates pistonState = RETRACTED;
  outtakeStates outtakeState = DOWN; // starts fully down
  chassis.opcontrol_drive_reverse_set(true); // Set to true if you want to reverse the drive controls
  outtake.tare_position();

  while (true) {
    // Gives you some extras to make EZ-Template ezier
    ez_template_extras();

    //chassis.opcontrol_tank();  // Tank control
    chassis.opcontrol_arcade_standard(ez::SPLIT);   // Standard split arcade
    // chassis.opcontrol_arcade_standard(ez::SINGLE);  // Standard single arcade
    //chassis.opcontrol_arcade_flipped(ez::SPLIT);    // Flipped split arcade
    // chassis.opcontrol_arcade_flipped(ez::SINGLE);   // Flipped single arcade

    // This is the main execution loop for the user control program.
    // Each time through the loop your program should update motor + servo
    // values based on feedback from the joysticks.

    // ........................................................................
    // Insert user code here. This is where you use the joystick values to
    // update your motors, etc.
    // ........................................................................

    // Reduce outtake motor speed when funnel is lowered
    if(pistonState == RETRACTED) {
      outtake.set_voltage_limit(12700); //mV = 100
    } else {
      outtake.set_voltage_limit(5715); //5715 mV = 45%
    }

    // Only allow intake when outtake is not moving and not fully up
    if(outtakeState != MOVING && outtakeState != UP) {
        if(master.get_digital(DIGITAL_R2)) {
            intake.move(127);
        } else if(master.get_digital(DIGITAL_R1)) {
            intake.move(-70);
        } else {
            intake.move(0);
        }
        // Piston control
        if(master.get_digital(DIGITAL_L1)) {
            funnel.set(true);
            pistonState = EXTENDED;
        } else if(master.get_digital(DIGITAL_L2)) {
            funnel.set(false);
            pistonState = RETRACTED;
        }
    } else {
        intake.move(0);
    }

    if(master.get_digital(DIGITAL_UP)) {
        descore.set(true);
    } else if(master.get_digital(DIGITAL_DOWN)) {
        descore.set(false);
    }

    if(master.get_digital(DIGITAL_LEFT)) {
        matchload.set(true);
    } else if(master.get_digital(DIGITAL_RIGHT)) {
        matchload.set(false);
    }

    // Outtake macro without sensor
    // Raise outtake only if not up
    if(master.get_digital(DIGITAL_X) && outtakeState != UP && outtakeState != MOVING) {
        outtakeState = MOVING;
        if (pistonState == RETRACTED) {
            outtake.move_absolute(385, 127); //up
        } else {
            outtake.move_absolute(410, 67); //up
        }
        outtakeState = UP;
    }

    // Lower B only if not down
    if(master.get_digital(DIGITAL_B) && outtakeState != DOWN && outtakeState != MOVING) {
        outtakeState = MOVING;
        if (pistonState == RETRACTED) {
            outtake.move_absolute(-385, 127); //down
        } else {
            outtake.move_absolute(-410, 67); //down
        }
        outtakeState = DOWN;
        outtake.tare_position();
    }

    pros::delay(ez::util::DELAY_TIME);  // This is used for timer calculations!  Keep this ez::util::DELAY_TIME
  }
}
