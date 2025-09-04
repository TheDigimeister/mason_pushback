#include "control.hpp"
#include "main.h"
#include "pros/distance.hpp"

pros::MotorGroup left_mg({1, -5, -2});    // Creates a motor group with forwards ports 1 & 3 and reversed port 2
pros::MotorGroup right_mg({-3, 6, 4});  // Creates a motor group with forwards port 5 and reversed ports 4 & 6

pros::Distance front_dist(11);
pros::Distance back_dist(12);
pros::Distance left_dist(13);
pros::Distance right_dist(14);


OdometryState odom_state(Pose {0, 0, 0}, left_mg, right_mg);

MCL mcl(odom_state);

MCLSensors mcl_sensors(front_dist, back_dist, left_dist, right_dist);