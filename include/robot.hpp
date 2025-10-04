#pragma once

#include "pros/distance.hpp"
#include "pros/motors.hpp"
#include "pros/adi.hpp"

extern pros::Distance front_dist;
extern pros::Distance back_dist;
extern pros::Distance left_dist;
extern pros::Distance right_dist;

extern pros::Motor lower;
extern pros::Motor upper;

extern pros::ADIDigitalOut level;
extern pros::ADIDigitalOut matchload;
extern pros::ADIDigitalOut descore;
extern pros::ADIDigitalOut odom;

extern bool odom_state;