#include "main.h"
#include "utils.hpp"

void nineBallLowGoal() {
    odom.set_value(false);
    // chassis.setPose(-46.847,-14.278,90);
    chassis.setPose(-46.847, positionFromRaycast(right_dist.get() * MM_TO_IN, RIGHT_DIST_OFFSET, SOUTH),90);


    // 9-ball low goal side

    lower.move(127);

    // pick up trio
    chassis.moveToPoint(-27, -22, 1500, {.maxSpeed=127});
    // chassis.waitUntil(float dist)
    // matchload.set_value(true);


    // pick up long goal balls
    chassis.moveToPoint(-8.25, -43, 2000, {.maxSpeed=80});
    chassis.waitUntil(5);
    matchload.set_value(false);
    chassis.waitUntil(30);
    matchload.set_value(true);
    pros::delay(200);

    // // score low goal
    chassis.moveToPoint(-22.345, -23.821, 2000, {.forwards=false, .maxSpeed=127});

    lower.move(0);
    // chassis.turnToPoint(-8.676, -13.504, 5000, {}, false);
    // chassis.moveToPoint(-8.676, -13.504, 5000, {.forwards=true, .maxSpeed=50}, false);
    // chassis.turnToHeading(45, 5000, {}, false);
    // lower.move(-127);
    // pros::delay(2000);
    level.set_value(false);

    // score into long goal
    chassis.turnToPoint(-40, -49.5, 2000);
    chassis.waitUntil(20);
    matchload.set_value(false);
    lower.move(127);
    chassis.moveToPoint(-40, -49.5, 2000, {.forwards=true, .maxSpeed=127}, false);
    chassis.turnToHeading(-90, 2000, {}, false);
    chassis.moveToPoint(-21, -49.5, 2000, {.forwards=false, .maxSpeed=80});
    
    pros::Task align_score1{[=]{
        while(back_dist.get() > 85) { pros::delay(50);}
        upper.move(127);
    }};

    // upper.move(127);
    matchload.set_value(true);
    pros::delay(3000);
    upper.move(0);

    // get matchload then score into long goal
    chassis.moveToPoint(-58, -49.5, 2000, {.forwards=true, .maxSpeed=70});
    pros::Task matchload_stop([=](){
        while(front_dist.get() > 204) {pros::delay(50);}
        chassis.cancelMotion();
        pros::delay(500);
    });
    chassis.moveToPoint(-62, -49.5, 750, {.forwards=true, .maxSpeed=60}, false);

    left_mg.move(-65);
    right_mg.move(-65);
    pros::delay(250);
    left_mg.move(65);
    right_mg.move(65);
    pros::delay(500);
    left_mg.move(0);
    right_mg.move(0);
    pros::delay(1500);

    chassis.moveToPoint(-21, -49.5, 2000, {.forwards=false, .maxSpeed=80});
    pros::Task align_score2{[=]{
        while(back_dist.get() > 85) { pros::delay(50);}
        upper.move(127);
    }};
    // upper.move(127);
    odom.set_value(false);
    pros::delay(3000);

}