#include "main.h"

pros::Distance front_dist(16);
pros::Distance back_dist(18);
pros::Distance left_dist(13);
pros::Distance right_dist(17);

pros::Motor lower(7);
pros::Motor upper(10);

pros::ADIDigitalOut level('A');
pros::ADIDigitalOut matchload('C');
pros::ADIDigitalOut descore('B');
pros::ADIDigitalOut odom('D');