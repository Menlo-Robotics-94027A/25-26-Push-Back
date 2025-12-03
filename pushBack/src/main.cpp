#include "main.h"

// ========================================
// MOTOR AND SENSOR SETUP
// ========================================

// Drive motors
pros::MotorGroup left_motors({LEFT_FRONT_MOTOR_PORT, LEFT_MIDDLE_MOTOR_PORT, LEFT_BACK_MOTOR_PORT},
                              pros::MotorGearset::blue); // or green/red depending on your cartridge
pros::MotorGroup right_motors({RIGHT_FRONT_MOTOR_PORT, RIGHT_MIDDLE_MOTOR_PORT, RIGHT_BACK_MOTOR_PORT},
                               pros::MotorGearset::blue);

// IMU sensor
pros::Imu imu(IMU_PORT);

// Tracking wheels (if using external tracking wheels)
// Comment these out if not using tracking wheels
// pros::ADIEncoder left_tracking_wheel(LEFT_TRACKING_WHEEL_PORT, LEFT_TRACKING_WHEEL_PORT + 1, false);
// pros::ADIEncoder right_tracking_wheel(RIGHT_TRACKING_WHEEL_PORT, RIGHT_TRACKING_WHEEL_PORT + 1, false);
// pros::ADIEncoder back_tracking_wheel(BACK_TRACKING_WHEEL_PORT, BACK_TRACKING_WHEEL_PORT + 1, false);

// ========================================
// LEMLIB DRIVETRAIN SETUP
// ========================================

// Drivetrain settings
lemlib::Drivetrain drivetrain(&left_motors,
                               &right_motors,
                               TRACK_WIDTH,
                               WHEEL_DIAMETER,
                               DRIVE_GEAR_RATIO,
                               MAX_LINEAR_SPEED);

// Lateral PID controller
lemlib::ControllerSettings lateral_controller(LATERAL_KP,
                                               LATERAL_KI,
                                               LATERAL_KD,
                                               LATERAL_SMALL_ERROR,
                                               LATERAL_SMALL_ERROR_TIMEOUT,
                                               LATERAL_LARGE_ERROR,
                                               LATERAL_LARGE_ERROR_TIMEOUT);

// Angular PID controller
lemlib::ControllerSettings angular_controller(ANGULAR_KP,
                                               ANGULAR_KI,
                                               ANGULAR_KD,
                                               ANGULAR_SMALL_ERROR,
                                               ANGULAR_SMALL_ERROR_TIMEOUT,
                                               ANGULAR_LARGE_ERROR,
                                               ANGULAR_LARGE_ERROR_TIMEOUT);

// Odometry configuration using drive encoders (integrated in motors)
lemlib::OdomSensors sensors(nullptr, // left tracking wheel (nullptr = use left motor encoder)
                             nullptr, // right tracking wheel (nullptr = use right motor encoder)
                             nullptr, // back tracking wheel (nullptr = no horizontal tracking)
                             nullptr, // left rotation sensor (nullptr = not used)
                             nullptr, // right rotation sensor (nullptr = not used)
                             &imu);   // IMU sensor

// If using external tracking wheels, replace the above with:
// lemlib::OdomSensors sensors(&left_tracking_wheel,
//                              &right_tracking_wheel,
//                              &back_tracking_wheel,
//                              nullptr,
//                              nullptr,
//                              &imu);

// Create the chassis
lemlib::Chassis chassis(drivetrain,
                        lateral_controller,
                        angular_controller,
                        sensors);

/**
 * A callback function for LLEMU's center button.
 *
 * When this callback is fired, it will toggle line 2 of the LCD text between
 * "I was pressed!" and nothing.
 */
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
void initialize() {
	pros::lcd::initialize();
	pros::lcd::set_text(1, "Initializing...");

	// Calibrate the IMU (this takes about 2 seconds)
	pros::lcd::set_text(2, "Calibrating IMU...");
	imu.reset();
	while (imu.is_calibrating()) {
		pros::delay(10);
	}
	pros::lcd::set_text(2, "IMU Calibrated!");

	// Calibrate the chassis (start odometry)
	chassis.calibrate();
	pros::lcd::set_text(3, "Chassis Calibrated!");

	// Set the pose (starting position) to (0, 0, 0)
	// Format: chassis.setPose(x, y, heading)
	// x and y are in inches, heading is in degrees
	chassis.setPose(0, 0, 0);

	pros::lcd::set_text(1, "Ready!");
	pros::lcd::register_btn1_cb(on_center_button);
}

/**
 * Runs while the robot is in the disabled state of Field Management System or
 * the VEX Competition Switch, following either autonomous or opcontrol. When
 * the robot is enabled, this task will exit.
 */
void disabled() {}

/**
 * Runs after initialize(), and before autonomous when connected to the Field
 * Management System or the VEX Competition Switch. This is intended for
 * competition-specific initialization routines, such as an autonomous selector
 * on the LCD.
 *
 * This task will exit when the robot is enabled and autonomous or opcontrol
 * starts.
 */
void competition_initialize() {}

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
	// Example autonomous movements
	// Adjust these values based on your robot and field requirements

	// Move forward 24 inches
	chassis.moveToPoint(0, 24, DEFAULT_TIMEOUT);

	// Turn to face 90 degrees (right turn)
	chassis.turnToHeading(90, DEFAULT_TIMEOUT);

	// Move forward another 24 inches
	chassis.moveToPoint(24, 24, DEFAULT_TIMEOUT);

	// Turn to face 180 degrees
	chassis.turnToHeading(180, DEFAULT_TIMEOUT);

	// Move backward to starting position
	chassis.moveToPoint(0, 0, DEFAULT_TIMEOUT);

	// Turn back to 0 degrees
	chassis.turnToHeading(0, DEFAULT_TIMEOUT);

	/* Other useful LemLib functions:
	 *
	 * chassis.moveToPose(x, y, heading, timeout) - Move to a pose (position + heading)
	 * chassis.follow(path, timeout, lookahead) - Follow a path (requires path generation)
	 * chassis.turnToPoint(x, y, timeout) - Turn to face a point
	 * chassis.getPose() - Get current position and heading
	 *
	 * You can also use these for more control:
	 * chassis.tank(left, right) - Manual tank drive control
	 * chassis.arcade(forward, turn) - Manual arcade drive control
	 */
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
	pros::Controller master(pros::E_CONTROLLER_MASTER);

	while (true) {
		// Get joystick values
		int left_y = master.get_analog(ANALOG_LEFT_Y);
		int right_x = master.get_analog(ANALOG_RIGHT_X);

		// Apply deadzone
		if (abs(left_y) < JOYSTICK_DEADZONE) left_y = 0;
		if (abs(right_x) < JOYSTICK_DEADZONE) right_x = 0;

		// Use LemLib's arcade control with drive curve
		chassis.arcade(left_y, right_x);

		// Display robot position on screen
		lemlib::Pose pose = chassis.getPose();
		pros::lcd::print(0, "X: %.2f  Y: %.2f", pose.x, pose.y);
		pros::lcd::print(1, "Heading: %.2f", pose.theta);

		// Optional: Press button A to run autonomous during driver control (for testing)
		if (master.get_digital(DIGITAL_A)) {
			autonomous();
		}

		pros::delay(10);
	}
}