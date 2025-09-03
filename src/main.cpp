#include "main.h"
#include "drive.hpp"
#include "pros/rtos.hpp"

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
	pros::lcd::set_text(1, "Hello PROS User!");

	pros::lcd::register_btn1_cb(on_center_button);

	// Start odometry update loop
	pros::Task odometry([=](){
		while (true) {
			odom_state.odom_update();
			pros::delay(20);
		}
	});

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

	odom_state.odom_init(0, 0, 0);

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

	pros::Motor lower(7);
	pros::Motor upper(10);

	pros::ADIDigitalOut level('A');
	pros::ADIDigitalOut matchload('C');
	pros::ADIDigitalOut descore('B');
	pros::ADIDigitalOut odom('D');


	bool descore_state = false;
	bool level_state = false;
	bool odom_state = false;
	bool matchload_state = false;

	left_mg.set_brake_mode(pros::E_MOTOR_BRAKE_BRAKE);
	right_mg.set_brake_mode(pros::E_MOTOR_BRAKE_BRAKE);

	while (true) {
		pros::lcd::print(0, "%d %d %d", (pros::lcd::read_buttons() & LCD_BTN_LEFT) >> 2,
		                 (pros::lcd::read_buttons() & LCD_BTN_CENTER) >> 1,
		                 (pros::lcd::read_buttons() & LCD_BTN_RIGHT) >> 0);  // Prints status of the emulated screen LCDs

		// Arcade control scheme
		int dir = master.get_analog(ANALOG_LEFT_Y);    // Gets amount forward/backward from left joystick
		int turn = master.get_analog(ANALOG_RIGHT_X);  // Gets the turn left/right from right joystick
		
		if (abs(dir) + abs(turn) > 5) {
			left_mg.move(dir + turn);                      // Sets left motor voltage
			right_mg.move(dir - turn);                     // Sets right motor voltage
		}
		else {
			left_mg.brake();
			right_mg.brake();
		}

		if (master.get_digital(pros::E_CONTROLLER_DIGITAL_R1)) {
			lower.move(127);
		}
		else if (master.get_digital(pros::E_CONTROLLER_DIGITAL_R2)) {
			lower.move(-127);
			upper.move(-127);
		}

		else if (master.get_digital(pros::E_CONTROLLER_DIGITAL_L1)) {
			lower.move(127);
			upper.move(127);
		}

		else {
			lower.move(0);
			upper.move(0);
		}


		// Pneumatics
		if (master.get_digital(pros::E_CONTROLLER_DIGITAL_X) && odom_state == false) {
			odom.set_value(true);
			odom_state = true;
			pros::delay(150);
		}
		else if (master.get_digital(pros::E_CONTROLLER_DIGITAL_X) && odom_state == true) {
			odom.set_value(false);
			odom_state = false;
			pros::delay(150);
		}

		if (master.get_digital(pros::E_CONTROLLER_DIGITAL_DOWN) && level_state == false) {
			level.set_value(true);
			level_state = true;
			pros::delay(150);

		}
		else if (master.get_digital(pros::E_CONTROLLER_DIGITAL_DOWN) && level_state == true){
			level.set_value(false);
			level_state = false;
			pros::delay(150);
		}
		if (master.get_digital(pros::E_CONTROLLER_DIGITAL_RIGHT) && matchload_state == false) {
			matchload.set_value(true);
			matchload_state = true;
			pros::delay(150);
		}
		else if (master.get_digital(pros::E_CONTROLLER_DIGITAL_RIGHT) && matchload_state == true){
			matchload.set_value(false);
			matchload_state = false;
			pros::delay(150);
		}
		if (master.get_digital(pros::E_CONTROLLER_DIGITAL_Y) && descore_state == false) {
			descore.set_value(true);
			descore_state = true;
			pros::delay(150);
		}
		
		else if (master.get_digital(pros::E_CONTROLLER_DIGITAL_Y) && descore_state == true){
			descore.set_value(false);
			descore_state = false;
			pros::delay(150);
		}

		pros::delay(20);                               // Run for 20 ms then update
	}
}