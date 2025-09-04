#ifndef _CONTROL_HPP_
#define _CONTROL_HPP_

#include "odom.hpp"
#include "pros/distance.hpp"
#include "pros/imu.hpp"

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
    pros::Imu &imu;
};

class MCL {
    public:
    MCL(
        OdometryState &odom_state
    );
    Particle particles[20];
    void init();
    void predict();
    void update(MCLSensors &sensors);
    void normalize();
    void resample();
    void estimate();
    Pose estimated_pose;

    private:
    OdometryState odom_state;
    Particle motion_model(Particle p);
    float sensor_model(Particle p, MCLSensors &mcl_sensors, float sigma);
    Particle weighted_random_sample(Particle particles[]);
    float raycast_to_wall(float start_x, float start_y, float dir_x, float dir_y);
};

class PID {
    public:
        PID(
            float kP,
            float kI,
            float kD
        );

    void init();

    float update(float target, float current);
    
    private:
        float previous_error;
        float integral;
        float kP;
        float kI;
        float kD;
};

#endif