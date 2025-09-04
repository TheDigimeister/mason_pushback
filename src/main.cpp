#include "main.h"
#include "drive.hpp"
#include "lemlib/api.hpp"
#include "pros/distance.hpp"
#include "pros/rtos.hpp"

pros::Controller master(pros::E_CONTROLLER_MASTER);
// pros::MotorGroup left_mg({1, -5, -2});    // Creates a motor group with forwards ports 1 & 3 and reversed port 2
// pros::MotorGroup right_mg({-3, 6, 4});  // Creates a motor group with forwards port 5 and reversed ports 4 & 6


pros::Motor lower(7);
pros::Motor upper(10);

pros::ADIDigitalOut level('A');
pros::ADIDigitalOut matchload('C');
pros::ADIDigitalOut descore('B');
pros::ADIDigitalOut odom('D');

pros::Distance back_dist(18);

bool descore_state = false;
bool level_state = false;
bool odom_state = false;
bool matchload_state = false;
int auton_num = 5;

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
	chassis.calibrate();

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

	left_mg.set_brake_mode(pros::E_MOTOR_BRAKE_BRAKE);
	right_mg.set_brake_mode(pros::E_MOTOR_BRAKE_BRAKE);

	if (auton_num == 0) {

		odom.set_value(true);
		chassis.calibrate();
		chassis.setPose(0,0,0);

		chassis.moveToPoint(0,24,10000);
		chassis.moveToPoint(0, 0, 10000, {.forwards=false});

		chassis.moveToPoint(0,24,10000);
		chassis.moveToPoint(0, 0, 10000, {.forwards=false});

		chassis.turnToHeading(180, 10000);
		chassis.turnToHeading(0,10000);

		chassis.turnToHeading(180, 10000);
		chassis.turnToHeading(0,10000);

		chassis.turnToHeading(180, 10000);
		chassis.turnToHeading(0,10000);

		chassis.turnToHeading(180, 10000);
		chassis.turnToHeading(0,10000);

	}

	else if (auton_num == 1) {
		odom.set_value(true);
		chassis.setPose(-46.847,-14.278,90);

		// Path

		lower.move(127);

		// pick up trio
		chassis.moveToPose(-22.603, -24,135, 2000, {.maxSpeed=127}, false);
		// chassis.waitUntil(float dist)
		// matchload.set_value(true);


		// pick up long goal balls
		chassis.moveToPoint(-10, -40.327, 2000, {.maxSpeed=80});
		chassis.waitUntil(5);
		matchload.set_value(false);
		chassis.waitUntil(20);
		matchload.set_value(true);
		pros::delay(200);

		// // score low goal
		chassis.moveToPoint(-22.345, -23.821, 2000, {.forwards=false, .maxSpeed=127});
		chassis.waitUntil(20);
		matchload.set_value(false);
		lower.move(0);
		// chassis.turnToPoint(-8.676, -13.504, 5000, {}, false);
		// chassis.moveToPoint(-8.676, -13.504, 5000, {.forwards=true, .maxSpeed=50}, false);
		// chassis.turnToHeading(45, 5000, {}, false);
		// lower.move(-127);
		// pros::delay(2000);
		level.set_value(false);

		// score into long goal
		chassis.turnToPoint(-40, -51, 2000);
		lower.move(127);
		chassis.moveToPoint(-40, -51, 2000, {.forwards=true, .maxSpeed=127}, false);
		chassis.turnToHeading(-90, 2000, {}, false);
		chassis.moveToPoint(-21, -51, 2000, {.forwards=false, .maxSpeed=80});
		
		pros::Task align_score1{[=]{
			while(back_dist.get_distance() > 75) { pros::delay(50);}
			upper.move(127);
		}};

		// upper.move(127);
		matchload.set_value(true);
		pros::delay(3000);
		upper.move(0);

		// get matchload then score into long goal
		chassis.moveToPoint(-58, -51, 2000, {.forwards=true, .maxSpeed=70});
		chassis.moveToPoint(-63.869, -51, 2000, {.forwards=true, .maxSpeed=40});
		chassis.moveToPoint(-21, -51, 2000, {.forwards=false, .maxSpeed=80});
		pros::Task align_score2{[=]{
			while(back_dist.get_distance() > 75) { pros::delay(50);}
			upper.move(127);
		}};
		// upper.move(127);
		odom.set_value(false);
		pros::delay(3000);

	}

	else if (auton_num == 2){
		odom.set_value(true);
		chassis.setPose(-46.847,14.278,90);

		// Path

		lower.move(127);

		// pick up trio
		chassis.moveToPose(-22.603, 24,45, 2000, {.maxSpeed=127}, false);
		// chassis.waitUntil(float dist)
		// matchload.set_value(true);


		// pick up long goal balls
		chassis.moveToPoint(-10, 40.327, 2000, {.maxSpeed=80});
		chassis.waitUntil(5);
		matchload.set_value(false);
		chassis.waitUntil(20);
		matchload.set_value(true);
		pros::delay(200);

		// // score low goal
		chassis.moveToPoint(-22.345, 23.821, 2000, {.forwards=false, .maxSpeed=127});
		chassis.waitUntil(20);
		matchload.set_value(false);
		lower.move(0);
		// chassis.turnToPoint(-8.676, -13.504, 5000, {}, false);
		// chassis.moveToPoint(-8.676, -13.504, 5000, {.forwards=true, .maxSpeed=50}, false);
		// chassis.turnToHeading(45, 5000, {}, false);
		// lower.move(-127);
		// pros::delay(2000);
		level.set_value(false);

		// score into long goal
		chassis.turnToPoint(-40, 51, 2000);
		lower.move(127);
		chassis.moveToPoint(-40, 51, 2000, {.forwards=true, .maxSpeed=127}, false);
		chassis.turnToHeading(-90, 2000, {}, false);
		chassis.moveToPoint(-21, 51, 2000, {.forwards=false, .maxSpeed=80});
		
		pros::Task align_score1{[=]{
			while(back_dist.get_distance() > 75) { pros::delay(50);}
			upper.move(127);
		}};

		// upper.move(127);
		matchload.set_value(true);
		pros::delay(3000);
		upper.move(0);

		// get matchload then score into long goal
		chassis.moveToPoint(-58, 51, 2000, {.forwards=true, .maxSpeed=70});
		chassis.moveToPoint(-63.869, 51, 2000, {.forwards=true, .maxSpeed=40});
		chassis.moveToPoint(-21, 51, 2000, {.forwards=false, .maxSpeed=80});
		pros::Task align_score2{[=]{
			while(back_dist.get_distance() > 75) { pros::delay(50);}
			upper.move(127);
		}};
		// upper.move(127);
		odom.set_value(false);
		pros::delay(3000);

	}

	else if (auton_num == 3) {
		odom.set_value(true);
		chassis.setPose(-46.847,14.278,90);

		// Path

		lower.move(127);

		// pick up trio
		chassis.moveToPose(-22.603, 24,45, 2000, {.maxSpeed=127}, false);
		// chassis.waitUntil(float dist)
		// matchload.set_value(true);


		// pick up long goal balls
		chassis.moveToPoint(-10, 40.327, 2000, {.maxSpeed=80});
		chassis.waitUntil(5);
		matchload.set_value(false);
		chassis.waitUntil(20);
		matchload.set_value(true);
		pros::delay(200);

		// // score middle goal
		chassis.moveToPoint(-22.345, 23.821, 2000, {.forwards=false, .maxSpeed=127});
		chassis.waitUntil(20);
		matchload.set_value(false);
		// lower.move(0);
		chassis.turnToPoint(-11.513, 11.771, 2000, {.forwards=false}, false);
		chassis.moveToPoint(-11.513, 11.771, 2000, {.forwards=false, .maxSpeed=50}, false);
		level.set_value(true);
		chassis.turnToHeading(-45, 500, {}, false);
		lower.move(127);
		upper.move(127);
		pros::delay(200);
		upper.move(0);
		chassis.waitUntil(20);
		level.set_value(false);

		// score into long goal
		chassis.turnToPoint(-40, 51, 2000);
		lower.move(127);
		chassis.moveToPoint(-40, 51, 2000, {.forwards=true, .maxSpeed=127}, false);
		chassis.turnToHeading(-90, 2000, {}, false);
		chassis.moveToPoint(-21, 51, 2000, {.forwards=false, .maxSpeed=80});
		
		pros::Task align_score1{[=]{
			while(back_dist.get_distance() > 75) { pros::delay(50);}
			upper.move(127);
		}};

		// upper.move(127);
		matchload.set_value(true);
		pros::delay(3000);
		upper.move(0);

		// get matchload then score into long goal
		chassis.moveToPoint(-58, 51, 2000, {.forwards=true, .maxSpeed=70});
		chassis.moveToPoint(-63.869, 51, 2000, {.forwards=true, .maxSpeed=40});
		chassis.moveToPoint(-21, 51, 2000, {.forwards=false, .maxSpeed=80});
		pros::Task align_score2{[=]{
			while(back_dist.get_distance() > 75) { pros::delay(50);}
			upper.move(127);
		}};
		// upper.move(127);
		odom.set_value(false);
		pros::delay(3000);

	}
	else if (auton_num == 4) {
		odom.set_value(true);
		chassis.setPose(-46.847,-14.278,90);

		// Path

		lower.move(127);

		// pick up trio
		chassis.moveToPose(-22.603, -24,135, 2000, {.maxSpeed=127}, false);
		// chassis.waitUntil(float dist)
		// matchload.set_value(true);


		// pick up long goal balls
		chassis.moveToPoint(-10, -40.327, 2000, {.maxSpeed=80});
		chassis.waitUntil(5);
		matchload.set_value(false);
		chassis.waitUntil(20);
		matchload.set_value(true);
		pros::delay(200);

		// // score middle goal
		chassis.moveToPoint(-22.345, -23.821, 2000, {.forwards=false, .maxSpeed=127});
		chassis.waitUntil(20);
		matchload.set_value(false);
		// lower.move(0);
		chassis.turnToPoint(-12.029, -11.957, 2000, {.forwards=true}, false);
		chassis.moveToPoint(-12.029, -11.957, 2000, {.forwards=true, .maxSpeed=50}, false);
		// level.set_value(true);
		chassis.turnToHeading(45, 500, {}, false);
		lower.move(-127);
		// upper.move(127);
		pros::delay(1000);
		upper.move(0);
		chassis.waitUntil(20);
		level.set_value(false);

		// score into long goal
		chassis.turnToPoint(-40, -51, 2000);
		lower.move(127);
		chassis.moveToPoint(-40, -51, 2000, {.forwards=true, .maxSpeed=127}, false);
		chassis.turnToHeading(-90, 2000, {}, false);
		chassis.moveToPoint(-21, -51, 2000, {.forwards=false, .maxSpeed=80});
		
		pros::Task align_score1{[=]{
			while(back_dist.get_distance() > 75) { pros::delay(50);}
			upper.move(127);
		}};

		// upper.move(127);
		matchload.set_value(true);
		pros::delay(3000);
		upper.move(0);

		// get matchload then score into long goal
		chassis.moveToPoint(-58, -51, 2000, {.forwards=true, .maxSpeed=70});
		chassis.moveToPoint(-63.869, -51, 2000, {.forwards=true, .maxSpeed=40});
		chassis.moveToPoint(-21, -51, 2000, {.forwards=false, .maxSpeed=80});
		pros::Task align_score2{[=]{
			while(back_dist.get_distance() > 75) { pros::delay(50);}
			upper.move(127);
		}};
		// upper.move(127);
		odom.set_value(false);
		pros::delay(3000);
	}

	else if (auton_num == 5) {

		// Skills
		odom.set_value(false);
		chassis.setPose(-47.363,-16.654,180);

		// Path

		lower.move(127);
		level.set_value(false);
		matchload.set_value(true);

		// Path

		// Get matchloader
		chassis.moveToPoint(-47.878, -50, 2000, {.forwards = true, .maxSpeed = 127});
		chassis.turnToHeading(-90, 2000);
		chassis.moveToPoint(-64.385, -50, 2000, {.forwards = true, .maxSpeed = 80});
		pros::delay(3000);
		
		// Score long goal
		chassis.moveToPoint(-26.73, -51, 2000, {.forwards = false, .maxSpeed = 50}, false);
		chassis.turnToHeading(-90, 2000);
		upper.move(127);
		pros::delay(3000);
		upper.move(0);
		matchload.set_value(false);
		
		// Get first trio
		chassis.moveToPoint(-23.893, -23.618, 2000, {.forwards = true, .maxSpeed = 127});
		chassis.waitUntil(10);
		matchload.set_value(true);
		pros::delay(500);
		
		// // Get second trio
		// chassis.moveToPoint(-4.807, -43.737, 2000, {.forwards = true, .maxSpeed = 50});
		// chassis.waitUntil(10);
		// matchload.set_value(false);
		// chassis.waitUntil(20);
		// matchload.set_value(true);
		// pros::delay(1000);
		
		// Get third trio
		chassis.moveToPoint(-14.608, -32.129, 2000, {.forwards = true, .maxSpeed = 127});
		chassis.turnToHeading(135, 2000);
		chassis.waitUntil(10);
		matchload.set_value(false);

		chassis.moveToPoint(40.327, -23.62, 5000, {.forwards = true, .maxSpeed = 127});
		chassis.moveToPoint(43.938, -51, 5000, {.forwards = true, .maxSpeed = 127});

	
		// Score second long goal
		chassis.turnToHeading(90, 2000);
		chassis.moveToPoint(26.4, -52.5, 2000, {.forwards = false, .maxSpeed = 65}, false);
		chassis.turnToHeading(90, 2000);
		upper.move(127);
		pros::delay(3000);
		upper.move(0);
		
		// Get second matchloader
		chassis.turnToHeading(90, 2000);
		matchload.set_value(true);
		chassis.moveToPoint(61.733, -50, 2000, {.forwards = true, .maxSpeed = 80});
		pros::delay(3000);

		// Go to matchloader 3
		chassis.moveToPoint(29.753, -0.365, 2000, {.forwards = false, .maxSpeed = 100});
		chassis.moveToPoint(46.259, 47, 2000, {.forwards = true, .maxSpeed = 100});

		// Score long goal 3
		chassis.turnToHeading(90, 2000);
		chassis.moveToPoint(23.821, 45.5, 2000, {.forwards = false, .maxSpeed = 50}, false);
		chassis.turnToHeading(90, 2000);
		upper.move(127);
		pros::delay(3000);
		upper.move(0);



		// Get matchloader
		chassis.moveToPoint(63.023, 47, 2000, {.forwards = true, .maxSpeed = 80}, false);
		pros::delay(3000);


		// Score long goal 3
		chassis.moveToPoint(23.821, 45.5, 2000, {.forwards = false, .maxSpeed = 50}, false);
		chassis.turnToHeading(90, 2000);
		upper.move(127);
		matchload.set_value(false);
		pros::delay(3000);
		upper.move(0);



		// Get 4th trio
		chassis.moveToPoint(35.942, 47.305, 2000, {.maxSpeed=127});
		chassis.turnToHeading( 180, 2000);
		chassis.moveToPose(24.078, 23.835, 225, 2000, {.maxSpeed = 127});
		chassis.waitUntil(15);
		matchload.set_value(true);
		pros::delay(1000);

		// Get 5th trio
		chassis.turnToPoint(-27, 20, 2000);
		chassis.moveToPoint(-27, 20, 2000, {.forwards = true, .maxSpeed = 127});
		chassis.waitUntil(10);
		matchload.set_value(false);
		// chassis.waitUntil(20);
		// matchload.set_value(true);

		// Score middle goal
		chassis.turnToHeading(-45, 2000);
		chassis.moveToPoint(-12, 7, 2000, {.forwards = false, .maxSpeed = 50}, false);
		chassis.turnToHeading(-45, 2000);
		level.set_value(true);
		pros::delay(200);
		upper.move(127);
		pros::delay(3000);
		upper.move(0);


		// Go to matchloader 4
		chassis.moveToPoint(-47.105, 38, 2000, {.forwards = true, .maxSpeed = 127});
		level.set_value(false);
		chassis.turnToHeading(-90, 2000);
		matchload.set_value(true);

		// Get matchloader 4
		chassis.moveToPoint(-63.353, 38, 2000, {.forwards = true, .maxSpeed = 80}, false);
		pros::delay(3000);

		// Score long goal 4
		chassis.moveToPoint(-23.635, 40, 2000, {.forwards = false, .maxSpeed = 50}, false);
		chassis.turnToHeading(-90, 2000);
		upper.move(127);
		pros::delay(3000);
		upper.move(0);

		// Prep for parking
		chassis.moveToPoint(-47.363, 23.835, 2000, {.forwards = true, .maxSpeed = 127});

		
		// // Score second long goal
		// chassis.moveToPoint(26.4, -47.345, 2000, {.forwards = false, .maxSpeed = 50});
		// upper.move(127);
		// pros::delay(2000);
		// upper.move(0);



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

	left_mg.set_brake_mode(pros::E_MOTOR_BRAKE_BRAKE);
	right_mg.set_brake_mode(pros::E_MOTOR_BRAKE_BRAKE);


	while (true) {
		pros::lcd::print(0, "%d %d %d", (pros::lcd::read_buttons() & LCD_BTN_LEFT) >> 2,
		                 (pros::lcd::read_buttons() & LCD_BTN_CENTER) >> 1,
		                 (pros::lcd::read_buttons() & LCD_BTN_RIGHT) >> 0);  // Prints status of the emulated screen LCDs

		// Arcade control scheme
		int dir = master.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y);    // Gets amount forward/backward from left joystick
		int turn = master.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_X);  // Gets the turn left/right from right joystick
		
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