#include "main.h"
#include "autons.hpp"
#include "lemlib/api.hpp"
#include "robot.hpp"
#include "utils.hpp"


bool descore_state = false;
bool level_state = false;
bool odom_state = false;
bool matchload_state = false;

const int AUTON_NUM = MIDDLEGOAL35;
const float RAYCAST_RESET_ANGLE_RANGE = 10.0; // ± degrees from 0°/360° or 90°/270° 
const float RAYCAST_RESET_MIN_ERROR = 0.3; // minimum error required before applying correction
const float RAYCAST_RESET_MAX_ERROR = 3.0; // maximum error to restrict correction (e.g. matchloader depth)

/**
 * A callback function for LLEMU's center button.
 *
 * When this callback is fired, it will toggle line 2 of the LCD text between
 * "I was pressed!" and nothing.
 */
void on_center_button() {

}

/**
 * Runs initialization code. This occurs as soon as the program is started.
 *
 * All other competition modes are blocked by initialize; it is recommended
 * to keep execution time for this mode under a few seconds.
 */
void initialize() {
	pros::lcd::initialize();
	pros::lcd::register_btn1_cb(on_center_button);

	chassis.calibrate();

    chassis.setPose(positionFromRaycast(back_dist.get() * MM_TO_IN, BACK_DIST_OFFSET, WEST), positionFromRaycast(right_dist.get() * MM_TO_IN, RIGHT_DIST_OFFSET, SOUTH),90);
	// chassis.setPose(0,0,0);

	left_mg.set_brake_mode(pros::E_MOTOR_BRAKE_BRAKE);
	right_mg.set_brake_mode(pros::E_MOTOR_BRAKE_BRAKE);

	pros::Task distance_resets([&] {

		while(true) {

			float frontReading = front_dist.get() * MM_TO_IN;
			float leftReading = left_dist.get() * MM_TO_IN;
			float rightReading = right_dist.get() * MM_TO_IN;
			float backReading = back_dist.get() * MM_TO_IN;

			float frontConfidence = front_dist.get_confidence();
			float leftConfidence = left_dist.get_confidence();
			float rightConfidence = right_dist.get_confidence();
			float backConfidence = back_dist.get_confidence();

			lemlib::Pose currentPose = chassis.getPose();
					
			float estimated_x = currentPose.x;
			float estimated_y = currentPose.y;

			float normalizedTheta = normalizeAngle(currentPose.theta);
			std::printf("Normalized Theta: %.3f", normalizedTheta);

			float estimatedRightPos = 0;
			float estimatedFrontPos = 0;
			float estimatedLeftPos = 0;
			float estimatedBackPos = 0;

			WALL wallDirection = NORTH;
			bool parallel = false;

			float error_x = 144.0;
			float error_y = 144.0;

			if (fabs(normalizedTheta) < RAYCAST_RESET_ANGLE_RANGE/2.0) {
				parallel = true;
				wallDirection = NORTH;
			}
			else if (fabs(normalizedTheta - 180) < RAYCAST_RESET_ANGLE_RANGE/2.0) {
				parallel = true;
				wallDirection = SOUTH;
			}
			else if (fabs(normalizedTheta - 90) < RAYCAST_RESET_ANGLE_RANGE/2.0) {
				parallel = true;
				wallDirection = EAST;
			}
			else if (fabs(normalizedTheta - 270) < RAYCAST_RESET_ANGLE_RANGE/2.0) {
				parallel = true;
				wallDirection = WEST;
			}
			else {
				parallel = false;
			}
			
			if (parallel) {
				switch (wallDirection) {
					case NORTH:
						estimatedRightPos = positionFromRaycast(rightReading, RIGHT_DIST_OFFSET, EAST);
						estimatedFrontPos = positionFromRaycast(frontReading, FRONT_DIST_OFFSET, NORTH);
						estimatedLeftPos = positionFromRaycast(leftReading, LEFT_DIST_OFFSET, WEST);
						estimatedBackPos = positionFromRaycast(backReading, BACK_DIST_OFFSET, SOUTH);
						estimated_x = (leftConfidence * estimatedLeftPos + rightConfidence * estimatedRightPos) / (leftConfidence + rightConfidence);
						estimated_y = (frontConfidence * estimatedFrontPos + backConfidence * estimatedBackPos) / (frontConfidence + backConfidence);
						break;
					case SOUTH:
						estimatedRightPos = positionFromRaycast(rightReading, RIGHT_DIST_OFFSET, WEST);
						estimatedFrontPos = positionFromRaycast(frontReading, FRONT_DIST_OFFSET, SOUTH);
						estimatedLeftPos = positionFromRaycast(leftReading, LEFT_DIST_OFFSET, EAST);
						estimatedBackPos = positionFromRaycast(backReading, BACK_DIST_OFFSET, NORTH);
						estimated_x = (leftConfidence * estimatedLeftPos + rightConfidence * estimatedRightPos) / (leftConfidence + rightConfidence);
						estimated_y = (frontConfidence * estimatedFrontPos + backConfidence * estimatedBackPos) / (frontConfidence + backConfidence);
						break;
					case EAST:
						estimatedRightPos = positionFromRaycast(rightReading, RIGHT_DIST_OFFSET, SOUTH);
						estimatedFrontPos = positionFromRaycast(frontReading, FRONT_DIST_OFFSET, EAST);
						estimatedLeftPos = positionFromRaycast(leftReading, LEFT_DIST_OFFSET, NORTH);
						estimatedBackPos = positionFromRaycast(backReading, BACK_DIST_OFFSET, WEST);
						estimated_y = (leftConfidence * estimatedLeftPos + rightConfidence * estimatedRightPos) / (leftConfidence + rightConfidence);
						estimated_x = (frontConfidence * estimatedFrontPos + backConfidence * estimatedBackPos) / (frontConfidence + backConfidence);
						break;
					case WEST:
						estimatedRightPos = positionFromRaycast(rightReading, RIGHT_DIST_OFFSET, NORTH);
						estimatedFrontPos = positionFromRaycast(frontReading, FRONT_DIST_OFFSET, WEST);
						estimatedLeftPos = positionFromRaycast(leftReading, LEFT_DIST_OFFSET, SOUTH);
						estimatedBackPos = positionFromRaycast(backReading, BACK_DIST_OFFSET, EAST);
						estimated_y = (leftConfidence * estimatedLeftPos + rightConfidence * estimatedRightPos) / (leftConfidence + rightConfidence);
						estimated_x = (frontConfidence * estimatedFrontPos + backConfidence * estimatedBackPos) / (frontConfidence + backConfidence);
						break;
					default:
						std::printf("Invalid wall direction");
						break;
				}

				error_x = fabs(estimated_x - currentPose.x);
				error_y = fabs(estimated_y - currentPose.y);

				std::printf("X Error: %.3f, Y Error: %.3f\n", error_x, error_y);

				if (error_x > RAYCAST_RESET_MIN_ERROR && error_x < RAYCAST_RESET_MAX_ERROR) {
					chassis.setPose(estimated_x, chassis.getPose().y, chassis.getPose().theta);
					std::printf("X pos reset!\n");
				}

				if (error_y > RAYCAST_RESET_MIN_ERROR && error_y < RAYCAST_RESET_MAX_ERROR) {
					chassis.setPose(chassis.getPose().x, estimated_y, chassis.getPose().theta);
					std::printf("Y pos reset!\n");
				}

				if (odom_state == true && frontReading < 300) {
					chassis.setPose(estimated_x, estimated_y, chassis.getPose().theta);
				}
			}

		pros::delay(500);
	}
	});

	pros::Task anti_jam([=]{
		while(true){
			if(lower.get_current_draw() > 2400) {
				lower.move(-127);
				pros::delay(100);
				lower.move(127);
				pros::delay(1000);
			} else {pros::delay(500);}
		}
	});

	pros::Task print_coordinates([=](){
		while (true) {
			// std::cout << "Estimated pose: x=" << chassis.getPose().x << ", y=" << chassis.getPose().y << ", theta=" << chassis.getPose().theta;
			if (true) {
				std::cout << std::endl;
				// std::printf("Estimated pose: x=%.3f, y=%.3f, theta=%.3f", chassis.getPose().x, chassis.getPose().y, chassis.getPose().theta);
				// pros::lcd::print(0, "X:%.2f, Y:%.2f, Theta:%.2f", chassis.getPose().x, chassis.getPose().y, chassis.getPose().theta);
				master.print(0, 0, "X:%.2fY:%.2fT:%.2f", chassis.getPose().x, chassis.getPose().y, chassis.getPose().theta);
				// master.print(0,0,"Dist Theta: %.3f", calculateHeading(chassis.getPose().x, chassis.getPose().y, chassis.getPose().theta, -5.289,5.503,-5.63,-2.028,front_dist.get() * MM_TO_IN,left_dist.get() * MM_TO_IN, NORTH, WEST));
				pros::delay(100);
			}
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

	switch (AUTON_NUM) {
		case 1:
			nineBallLowGoal();
			break;
		case 2:
			nineBallMiddleGoal();
			break;
		case 3:
			soloAWPLowGoal();
			break;
		case 4:
			soloAWPMiddleGoal();
			break;
		case 5:
			skills();
			break;
		case 6:
			skills2();
			break;
		case 7:
			middleGoal31();
			break;
		case 8:
			middleGoal35();
			break;
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

 #pragma region opcontrol
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