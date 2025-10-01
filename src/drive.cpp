#include "drive.hpp"
#include "lemlib/chassis/chassis.hpp"
#include "lemlib/chassis/trackingWheel.hpp"
#include "main.h"


pros::ADIAnalogIn P_pot('E');
pros::ADIAnalogIn D_pot('F');

pros::MotorGroup left_mg({1, -5, -2});    // Creates a motor group with forwards ports 1 & 3 and reversed port 2
pros::MotorGroup right_mg({-3, 6, 4});  // Creates a motor group with forwards port 5 and reversed ports 4 & 6

lemlib::Drivetrain drivetrain(&left_mg, // left motor group
                              &right_mg, // right motor group
                              11.5, // 10 inch track width
                              lemlib::Omniwheel::NEW_325, // using new 4" omnis
                              450, // drivetrain rpm is 360
                              3 // horizontal drift is 2 (for now)
);

pros::Rotation vert_odom(-12);
pros::Rotation hor_odom(20);
pros::Imu inertial(11);

lemlib::TrackingWheel vertical_tracking_wheel(&vert_odom, lemlib::Omniwheel::NEW_275, 0.125);
lemlib::TrackingWheel hor_tracking_wheel(&hor_odom, lemlib::Omniwheel::NEW_2 * (120.0/125.0), -3.75); // lemlib::Omniwheel::NEW_2 * (117.25/126.25)

lemlib::OdomSensors sensors(&vertical_tracking_wheel, // vertical tracking wheel 1, set to null
                            nullptr, // vertical tracking wheel 2, set to nullptr as we are using IMEs
                            &hor_tracking_wheel, // horizontal tracking wheel 1
                            nullptr, // horizontal tracking wheel 2, set to nullptr as we don't have a second one
                            &inertial // inertial sensor
);

// PERFECT lateral PID controller
lemlib::ControllerSettings lateral_controller(6, // proportional gain (kP)
                                              0.5, // integral gain (kI) // 2
                                              45, // derivative gain (kD)
                                              2, // anti windup
                                              1, // small error range, in inches
                                              300, // small error range timeout, in milliseconds
                                              3, // large error range, in inches
                                              700, // large error range timeout, in milliseconds
                                              0 // maximum acceleration (slew)
);

// PERFECT angular PID controller
lemlib::ControllerSettings angular_controller(5.5, // proportional gain (kP)
                                              0.04, // integral gain (kI) // 0.04
                                              57, // derivative gain (kD)
                                              21, // anti windup
                                              1, // small error range, in degrees
                                              150, // small error range timeout, in milliseconds
                                              3, // large error range, in degrees
                                              500, // large error range timeout, in milliseconds
                                              0 // maximum acceleration (slew)
);

// // angular PID controller
// lemlib::ControllerSettings angular_controller(3, // proportional gain (kP)
//                                               0.01, // integral gain (kI)
//                                               135, // derivative gain (kD)
//                                               24.75, // anti windup
//                                               1, // small error range, in degrees
//                                               100, // small error range timeout, in milliseconds
//                                               3, // large error range, in degrees
//                                               500, // large error range timeout, in milliseconds
//                                               0 // maximum acceleration (slew)
// );



lemlib::Chassis chassis(drivetrain, // drivetrain settings
                        lateral_controller, // lateral PID settings
                        angular_controller, // angular PID settings
                        sensors // odometry sensors
);