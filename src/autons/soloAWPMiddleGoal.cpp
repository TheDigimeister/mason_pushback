#include "main.h"

void soloAWPMiddleGoal() {
    odom.set_value(false);
    chassis.setPose(-46.847,14.278,90);

    // solo AWP middle goal

    lower.move(127);

    // pick up trio
    chassis.moveToPose(-22, 24,35, 2000, {.maxSpeed=127}, false);
    // chassis.waitUntil(float dist)
    // matchload.set_value(true);


    // pick up long goal balls
    chassis.moveToPoint(-7.75, 42, 2000, {.maxSpeed=80});
    chassis.waitUntil(5);
    matchload.set_value(false);
    chassis.waitUntil(20);
    matchload.set_value(true);
    pros::delay(200);

    // // score middle goal
    chassis.moveToPoint(-22.345, 23.821, 2000, {.forwards=false, .maxSpeed=127});
    chassis.waitUntil(20);
    matchload.set_value(false);
    // lower.move(0);
    chassis.turnToHeading(-45, 2000, {}, false);
    chassis.moveToPoint(-14, 12, 2000, {.forwards=false, .maxSpeed=50}, false);
    level.set_value(true);
    chassis.turnToHeading(-45, 2000, {}, false);
    lower.move(127);
    upper.move(127);
    pros::delay(300);
    upper.move(0);
    chassis.waitUntil(20);

    // score into long goal
    chassis.turnToPoint(-40, 49, 2000);
    lower.move(127);
    chassis.moveToPoint(-40, 49, 2000, {.forwards=true, .maxSpeed=127}, false);
    level.set_value(false);
    chassis.turnToHeading(-90, 2000, {}, false);
    // pros::Task align_score1{[=]{
    // 	while(back_dist.get() > 85) { pros::delay(50);}
    // 	upper.move(127);
    // }};
    // chassis.moveToPoint(-21, 49, 2000, {.forwards=false, .maxSpeed=80});
    

    // upper.move(127);
    matchload.set_value(true);
    // pros::delay(1500);
    upper.move(0);

    // get matchload then score into long goal
    chassis.moveToPoint(-58, 50, 2000, {.forwards=true, .maxSpeed=70});
    chassis.moveToPoint(-63.869, 50, 750, {.forwards=true, .maxSpeed=40});
    pros::Task align_score2{[=]{
        while(back_dist.get() > 85) { pros::delay(50);}
        upper.move(127);
    }};
    chassis.moveToPoint(-21, 49, 2000, {.forwards=false, .maxSpeed=80});

    // upper.move(127);
    odom.set_value(true);
    pros::delay(3000);

}