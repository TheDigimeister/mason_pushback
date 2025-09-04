#ifndef _DRIVE_HPP_
#define _DRIVE_HPP_

#include "odom.hpp"
#include "control.hpp"
#include "pros/motor_group.hpp"

extern pros::MotorGroup left_mg;    // Creates a motor group with forwards ports 1 & 3 and reversed port 2
extern pros::MotorGroup right_mg;  // Creates a motor group with forwards port 5 and reversed ports 4 & 6


extern OdometryState odom_state;
extern MCL mcl;
extern MCLSensors mcl_sensors;

#endif