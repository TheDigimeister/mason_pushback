#include "main.h"

void nineBallMiddleGoal() {
    odom.set_value(false);
    chassis.setPose(-46.847,14.278,90);

    // 9-ball middle goal side

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
        while(back_dist.get() > 85) { pros::delay(50);}
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
        while(back_dist.get() > 85) { pros::delay(50);}
        upper.move(127);
    }};
    // upper.move(127);
    odom.set_value(false);
    pros::delay(3000);

}