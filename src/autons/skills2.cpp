#include "drive.hpp"
#include "main.h"
#include "robot.hpp"
#include "utils.hpp"

void skills2() {

    odom.set_value(false);
    odom_state = false;
    chassis.setPose(positionFromRaycast(right_dist.get() * MM_TO_IN, RIGHT_DIST_OFFSET, WEST), positionFromRaycast(front_dist.get() * MM_TO_IN, FRONT_DIST_OFFSET, SOUTH),180);

    // Path

    lower.move(127);
    level.set_value(false);
    matchload.set_value(true);

    // Path

    // Get matchloader
    chassis.moveToPoint(-48, -48, 2000, {.forwards = true, .maxSpeed = 127, .minSpeed = 5, .earlyExitRange = 1});
    chassis.turnToHeading(-90, 2000, {.minSpeed = 5, .earlyExitRange = 3});
    chassis.moveToPoint(-62, -47.5, 1000, {.forwards = true, .maxSpeed = 127}, false);
    left_mg.move(-50);
    right_mg.move(-50);
    pros::delay(250);
    left_mg.move(50);
    right_mg.move(50);
    pros::delay(500);
    left_mg.move(0);
    right_mg.move(0);
    pros::delay(1500);
    
    // Score long goal
    chassis.moveToPoint(-28, -48, 2000, {.forwards = false, .maxSpeed = 100});
    pros::Task align_score1{[=]{
        while(back_dist.get() > 85) { pros::delay(50);}
        upper.move(127);
    }};
    chassis.turnToHeading(-90, 2000, {}, false);
    pros::delay(1500);
    // chassis.setPose(chassis.getPose().x, -72 + LEFT_DIST_OFFSET + left_dist.get() * MM_TO_IN, chassis.getPose().theta); 
    upper.move(0);
    matchload.set_value(false);
    
    // Get first trio
    chassis.moveToPoint(-45, -47, 2000, {.minSpeed = 5, .earlyExitRange = 3});
    chassis.turnToPoint(-22, -22, 2000);
    chassis.moveToPoint(-22, -22, 2000, {.forwards = true, .maxSpeed = 50, .minSpeed = 5, .earlyExitRange = 3});
    chassis.waitUntil(10);
    // matchload.set_value(true);
    // pros::delay(500);
    
    // // Get second trio
    // chassis.moveToPoint(-4.807, -43.737, 2000, {.forwards = true, .maxSpeed = 50});
    // chassis.waitUntil(10);
    // matchload.set_value(false);
    // chassis.waitUntil(20);
    // matchload.set_value(true);
    // pros::delay(1000);
    
    // // Get third trio
    // chassis.moveToPoint(-14.608, -32.129, 2000, {.forwards = true, .maxSpeed = 127});
    // chassis.turnToHeading(135, 2000);
    // chassis.waitUntil(10);
    matchload.set_value(false);

    chassis.turnToHeading(90, 2000);
    chassis.moveToPoint(41.40, -22, 5000, {.forwards = true, .maxSpeed = 100});
    // chassis.turnToHeading(90, 500);
    // chassis.setPose(72 - FRONT_DIST_OFFSET - front_dist.get() * MM_TO_IN, -72 + RIGHT_DIST_OFFSET + right_dist.get() * MM_TO_IN, chassis.getPose().theta); 
    chassis.turnToHeading(180, 2000, {.minSpeed = 5, .earlyExitRange = 3});
    chassis.moveToPoint(47, -49, 5000, {.forwards = true, .maxSpeed = 100});


    // Score second long goal
    chassis.turnToHeading(90, 2000);
    chassis.moveToPoint(28, -48.5, 2000, {.forwards = false, .maxSpeed = 100});
    pros::Task align_score2{[=]{
        while(back_dist.get() > 85) { pros::delay(50);}
        upper.move(127);
    }};
    chassis.turnToHeading(90, 2000, {});
    pros::delay(1500);
    // chassis.setPose(chassis.getPose().x, -72 + RIGHT_DIST_OFFSET + right_dist.get() * MM_TO_IN, chassis.getPose().theta); 
    upper.move(0);
    
    // Get second matchloader
    chassis.turnToHeading(90, 2000);
    matchload.set_value(true);
    chassis.moveToPoint(62, -47, 1000, {.forwards = true, .maxSpeed = 127}, false);
    left_mg.move(-50);
    right_mg.move(-50);
    pros::delay(250);
    left_mg.move(50);
    right_mg.move(50);
    pros::delay(500);
    left_mg.move(0);
    right_mg.move(0);
    pros::delay(1500);

    // Score 2nd long goal
    chassis.moveToPoint(28, -48.5, 2000, {.forwards = false, .maxSpeed = 100});
    pros::Task align_score3{[=]{
        while(back_dist.get() > 85) { pros::delay(50);}
        upper.move(127);
    }};
    chassis.turnToHeading(90, 2000, {});
    pros::delay(1500);
    // chassis.setPose(chassis.getPose().x, -72 + RIGHT_DIST_OFFSET + right_dist.get() * MM_TO_IN, chassis.getPose().theta); 
    upper.move(0);

    // Go to matchloader 3
    chassis.moveToPoint(51, -48, 2000, {.forwards = true, .minSpeed = 5, .earlyExitRange = 3});
    chassis.turnToHeading(20, 2000, {}, false);
    odom.set_value(true);
    odom_state = true;
    matchload.set_value(false);
    pros::delay(200);

    left_mg.move(110);
    right_mg.move(110);
    pros::delay(1400);
    left_mg.move(0);
    right_mg.move(0);
    pros::delay(300);

    chassis.turnToHeading(90, 2000, {}, false);
    left_mg.move(-50);
    right_mg.move(-50);
    pros::delay(200);
    left_mg.move(0);
    right_mg.move(0);

    pros::delay(300);
    odom.set_value(false);
    odom_state = false;
    pros::delay(300);
    chassis.turnToPoint(43, 48, 2000);
    chassis.moveToPoint(43, 48, 2000, {.forwards = false, .minSpeed = 5, .earlyExitRange = 3});
    chassis.turnToHeading(90, 2000);
    matchload.set_value(true);
    pros::delay(200);


    // // Score long goal 3
    // chassis.moveToPoint(28, 48.5, 2000, {.forwards = false, .maxSpeed = 100});
    // pros::Task align_score3{[=]{
    //     while(back_dist.get() > 85) { pros::delay(50);}
    //     upper.move(127);
    // }};
    // chassis.turnToHeading(90, 2000, {}, false);
    // matchload.set_value(true);
    // pros::delay(1500);
    // // chassis.setPose(chassis.getPose().x, 72 - LEFT_DIST_OFFSET - left_dist.get() * MM_TO_IN, chassis.getPose().theta); 
    // upper.move(0);

    // Get matchloader
    chassis.moveToPoint(62, 49, 1000, {.forwards = true, .maxSpeed = 127}, false);
    left_mg.move(-50);
    right_mg.move(-50);
    pros::delay(250);
    left_mg.move(50);
    right_mg.move(50);
    pros::delay(500);
    left_mg.move(0);
    right_mg.move(0);
    pros::delay(1500);


    // Score long goal 3
    chassis.moveToPoint(28, 48.5, 2000, {.forwards = false, .maxSpeed = 100});
    pros::Task align_score4{[=]{
        while(back_dist.get() > 85) { pros::delay(50);}
        upper.move(127);
    }};
    chassis.turnToHeading(90, 2000, {}, false);
    pros::delay(1500);
    // chassis.setPose(chassis.getPose().x, 72 - LEFT_DIST_OFFSET - left_dist.get() * MM_TO_IN, chassis.getPose().theta); 
    upper.move(0);



    // Get 4th trio
    chassis.moveToPoint(47, 47, 2000, {.maxSpeed=127, .minSpeed=5, .earlyExitRange=3});
    matchload.set_value(false);
    chassis.turnToHeading( -135, 2000, {.minSpeed = 5, .earlyExitRange = 3});
    // chassis.moveToPoint(35, 35, 2000);
    chassis.moveToPoint(20, 20, 2000, {.maxSpeed = 127, .minSpeed = 5, .earlyExitRange = 3});
    // chassis.waitUntil(15);
    // matchload.set_value(true);
    // pros::delay(1000);

    // Get 5th trio
    chassis.turnToPoint(-47, 22, 2000);
    chassis.moveToPoint(-47, 22, 2000, {.forwards = true, .maxSpeed = 127, .minSpeed = 5, .earlyExitRange = 3});

    chassis.turnToHeading(-90, 1000, {});
    // chassis.setPose(-72 + FRONT_DIST_OFFSET + front_dist.get() * MM_TO_IN, chassis.getPose().y, chassis.getPose().theta); 

    // chassis.waitUntil(10);
    // matchload.set_value(false);
    // chassis.waitUntil(20);
    // matchload.set_value(true);

    // Score middle goal
    // chassis.turnToHeading(-45, 2000);
    // chassis.moveToPoint(-10, 10, 2000, {.forwards = false, .maxSpeed = 50}, false);
    // chassis.turnToHeading(-45, 2000);
    // level.set_value(true);
    // pros::delay(200);
    // upper.move(127);
    // pros::delay(3000);
    // upper.move(0);


    // Go to matchloader 4
    chassis.moveToPoint(-47, 47, 2000, {.forwards = true, .maxSpeed = 127});
    level.set_value(false);
    chassis.turnToHeading(-90, 2000, {}, false);
    // chassis.setPose(chassis.getPose().x, 72 - RIGHT_DIST_OFFSET - right_dist.get() * MM_TO_IN, chassis.getPose().theta); 
    // pros::delay(100);
    matchload.set_value(true);

    // Get matchloader 4
    chassis.moveToPoint(-62, 46.5, 1000, {.forwards = true, .maxSpeed = 127}, false);
    left_mg.move(-50);
    right_mg.move(-50);
    pros::delay(250);
    left_mg.move(50);
    right_mg.move(50);
    pros::delay(500);
    left_mg.move(0);
    right_mg.move(0);
    pros::delay(1500);

    // Score long goal 4
    chassis.moveToPoint(-28, 48.5, 2000, {.forwards = false, .maxSpeed = 100});
    pros::Task align_score5{[=]{
        while(back_dist.get() > 85) { pros::delay(50);}
        upper.move(127);
    }};
    chassis.turnToHeading(-90, 2000, {}, false);
    pros::delay(1500);
    // chassis.setPose(chassis.getPose().x, 72 - RIGHT_DIST_OFFSET - right_dist.get() * MM_TO_IN, chassis.getPose().theta); 
    upper.move(0);

    // Prep for parking
    chassis.moveToPoint(-51, 47, 2000, {.forwards = true, .maxSpeed = 50});
    matchload.set_value(false);
    chassis.turnToHeading(-150, 2000, {}, false);
    odom.set_value(true);
    odom_state = true;
    // chassis.moveToPose(-63, 16, 180, 2000, {.maxSpeed=50},false);
    left_mg.move(90);
    right_mg.move(90);
    pros::delay(1300);
    // matchload.set_value(true);
    // pros::delay(1000);
    // left_mg.move(127);
    // right_mg.move(127);
    // pros::delay(5000);
    left_mg.move(0);
    right_mg.move(0);
    left_mg.brake();
    right_mg.brake();


    
    // // Score second long goal
    // chassis.moveToPoint(26.4, -47.345, 2000, {.forwards = false, .maxSpeed = 50});
    // upper.move(127);
    // pros::delay(2000);
    // upper.move(0);

}