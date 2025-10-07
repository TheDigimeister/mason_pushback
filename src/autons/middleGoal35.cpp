#include "main.h"
#include "robot.hpp"
#include "utils.hpp"

void middleGoal35() {
    odom.set_value(false);
    // chassis.setPose(-46.847,-14.278,90);
    chassis.setPose(positionFromRaycast(back_dist.get() * MM_TO_IN, BACK_DIST_OFFSET, WEST), positionFromRaycast(left_dist.get() * MM_TO_IN, LEFT_DIST_OFFSET, NORTH),90);
    // chassis.setPose()


    // 9-ball low goal side (37)

    lower.move(127);

    // pick up trio
    // chassis.moveToPoint(-36, -14.5, 1500, {.maxSpeed=127, .minSpeed = 50, .earlyExitRange = 2});
    // chassis.turnToHeading(125, 2000, {.minSpeed = 5, .earlyExitRange = 2});
    // chassis.turnToPoint(-22, -25, 1000, {.minSpeed=5, .earlyExitRange=2});
    chassis.moveToPoint(-24, 25, 250, {.maxSpeed=127, .minSpeed = 60, .earlyExitRange = 36});
    chassis.moveToPoint(-24, 25, 2000, {.maxSpeed = 60, .minSpeed = 60, .earlyExitRange = 3});
    // chassis.waitUntil(float dist)
    // matchload.set_value(true);


    // pick up long goal balls
    chassis.moveToPoint(-8, 44.5, 2000, {.maxSpeed=80, .minSpeed = 5, .earlyExitRange = 3}, false);
    // matchload.set_value(true);
    pros::delay(100);

    // chassis.moveToPoint(-22.345, -23.821, 2000, {.forwards=false, .maxSpeed=127});

    // back up
    chassis.moveToPoint(-24, 36, 2000, {.forwards = false, .maxSpeed = 127, .minSpeed = 100, .earlyExitRange = 5}, false);
    lower.move(0);
    // chassis.turnToPoint(-8.676, -13.504, 5000, {}, false);
    // chassis.moveToPoint(-8.676, -13.504, 5000, {.forwards=true, .maxSpeed=50}, false);
    // chassis.turnToHeading(45, 5000, {}, false);
    // lower.move(-127);
    // pros::delay(2000);
    level.set_value(false);


    // move to goals
    // chassis.turnToHeading(-135, 2000);   
    lower.move(127);
    chassis.moveToPoint(-42, 46, 2000, {.forwards = false, .maxSpeed=127, .minSpeed = 50, .earlyExitRange = 3});
    chassis.turnToHeading(-90, 2000, {.maxSpeed = 127, .minSpeed = 5, .earlyExitRange = 1});

    // score into long goal
    chassis.moveToPoint(-28, 49, 4000, {.forwards=false, .maxSpeed=127});
    matchload.set_value(true);
    pros::Task align_score1{[=]{
        while(back_dist.get() > 120) { pros::delay(50);}
        upper.move(127);
    }};
    pros::delay(2000);
    upper.move(0);

    // get matchload
    pros::Task matchload_stop([=](){
        while(front_dist.get() > 220) {pros::delay(50);}
        chassis.cancelMotion();
    });
    chassis.moveToPoint(-61, 47.5, 2000, {.forwards=true, .maxSpeed=80, .minSpeed = 80}, false);

    pros::delay(700);

    // left_mg.move(-65);
    // right_mg.move(-65);
    // pros::delay(250);
    // left_mg.move(65);
    // right_mg.move(65);
    // pros::delay(500);
    // left_mg.move(0);
    // right_mg.move(0);
    // pros::delay(1500);

    // score into middle goal
    chassis.moveToPoint(-13, 13, 2000, {.forwards=false, .maxSpeed=127}, false); // 11.75, 8.5
    chassis.turnToHeading(-45, 2000);
    level.set_value(true);
    pros::delay(100);
    upper.move(127);
    // pros::Task align_score2{[=]{
    //     while(back_dist.get() > 120) { pros::delay(50);}
    //     upper.move(127);
    // }};
    pros::delay(2500);
    upper.move(0);

    // descore
    chassis.moveToPoint(-46, 35.75, 2000, {.minSpeed=5, .earlyExitRange=2});
    descore.set_value(true);
    chassis.turnToHeading(90, 2000, {.minSpeed=5, .earlyExitRange=2});
    // chassis.moveToPoint(-26.80, -38, 2000, {.forwards=false, .minSpeed=30, .earlyExitRange=3}, false);
    // chassis.turnToHeading(-90, 2000, {.minSpeed=5, .earlyExitRange=3});
    chassis.moveToPoint(-11, 35.75, 2000, {.forwards=true, .minSpeed = 30, .earlyExitRange = 3});
    chassis.waitUntil(12);
    descore.set_value(false);
    chassis.turnToHeading(90, 2000);


    // upper.move(127);
    odom.set_value(false);
    matchload.set_value(false);



}