#pragma once

#include "lemlib/chassis/chassis.hpp"
#include "pros/adi.hpp"
#include "pros/imu.hpp"

extern pros::MotorGroup left_mg;
extern pros::MotorGroup right_mg;

extern pros::Rotation odom_sensor;
extern pros::IMU inertial;

extern lemlib::Chassis chassis;

extern lemlib::ControllerSettings lateral_controller;
extern lemlib::ControllerSettings angular_controller;

extern float p_value;
extern float d_value;

extern pros::ADIAnalogIn P_pot;
extern pros::ADIAnalogIn D_pot;