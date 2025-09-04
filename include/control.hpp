#ifndef _CONTROL_HPP_
#define _CONTROL_HPP_

#include "odom.hpp"
#include "pros/distance.hpp"
#include <vector>

struct Particle {
    float x;
    float y;
    float theta;
    float weight;
};

struct MCLSensors {
    pros::Distance &front_dist;
    pros::Distance &back_dist;
    pros::Distance &left_dist;
    pros::Distance &right_dist;
};

class MCL {
    public:
    MCL(
        OdometryState &odom_state
    );
    Particle particles[20];
    void init();
    void predict();
    void update();
    void normalize();
    void resample();
    void estimate();
    Pose estimated_pose;

    private:
    OdometryState odom_state;
    Particle motion_model(Particle p);
    float sensor_model(Particle p, MCLSensors mcl_sensors, float sigma);
    Particle weighted_random_sample(Particle particles[]);
};

#endif