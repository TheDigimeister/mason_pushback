#include "main.h"

pros::MotorGroup left_mg({1, -5, -2});    // Creates a motor group with forwards ports 1 & 3 and reversed port 2
pros::MotorGroup right_mg({-3, 6, 4});  // Creates a motor group with forwards port 5 and reversed ports 4 & 6

OdometryState odom_state(Pose {0, 0, 0}, left_mg, right_mg);
