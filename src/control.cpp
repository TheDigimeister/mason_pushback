#include "control.hpp"
#include "drive.hpp"
#include "main.h"
#include "odom.hpp"
#include <cmath>
#include <cstdlib>
#include <numeric>
#include <random>

const int NUM_PARTICLES = 20;

MCL::MCL(OdometryState &odom_state) : odom_state(odom_state){}

void MCL::init() {
    std::random_device rd;
    std::mt19937 gen(rd());
    std::uniform_real_distribution<float> x_dist(odom_state.get_x_position() - 12.0, odom_state.get_x_position() + 12.0);
    std::uniform_real_distribution<float> y_dist(odom_state.get_y_position() - 12.0, odom_state.get_y_position() + 12.0);
    std::uniform_real_distribution<float> theta_dist(odom_state.get_heading() - 12.0, odom_state.get_heading() + 12.0);

    for (auto& p: this->particles) {
        p.x = x_dist(gen);
        p.y = y_dist(gen);
        p.theta = theta_dist(gen);
    }

}

void MCL::predict() {

    for (auto& p: this->particles) {
        p = motion_model(p);
    }

}

void MCL::update() {
    for (auto& p: this->particles) {
        p.weight = sensor_model(p, mcl_sensors, 1.0);
    }

}

void MCL::normalize() {

    float total_weight = 0;

    for (auto& p: this->particles) {
        total_weight += p.weight;
    }

    for(auto& p: this-> particles) {
        p.weight /= total_weight;
    }

}

void MCL::resample() {

    Particle new_particles[NUM_PARTICLES];

    for (int i=0; i < NUM_PARTICLES; i++) {
        new_particles[i] = this->weighted_random_sample(this->particles);
    }

    for (int i = 0; i < NUM_PARTICLES; i++) {
        this->particles[i] = new_particles[i];
    }

}

void MCL::estimate() {

    float x_est = 0;
    float y_est = 0;
    float theta_est = 0;

    for (auto& p: this->particles) {
        x_est += p.weight * p.x;
        y_est += p.weight * p.y;
        theta_est += p.weight * p.theta;
    }

    Pose estimated_pose = {x_est, y_est, theta_est};

}

Particle MCL::motion_model(Particle p) {
    float x_pred = p.x + odom_state.get_global_delta_x();
    float y_pred = p.y + odom_state.get_global_delta_y();
    float theta_pred = p.theta + odom_state.get_global_delta_theta();
    return Particle {x_pred, y_pred, theta_pred, p.weight};
}

float MCL::sensor_model(Particle p, MCLSensors mcl_sensors, float sigma) {

    float error = 0;
    float n_pred = 144.0 - p.y;
    float e_pred = 144.0 - p.x;
    float s_pred = p.y;
    float w_pred = p.x;

    float n_error = fabs(n_pred - mcl_sensors.front_dist.get_distance() * MM_TO_INCHES);
    float e_error = fabs(e_pred - mcl_sensors.back_dist.get_distance() * MM_TO_INCHES);
    float s_error = fabs(s_pred - mcl_sensors.right_dist.get_distance() * MM_TO_INCHES);
    float w_error = fabs(w_pred - mcl_sensors.left_dist.get_distance() * MM_TO_INCHES);

    float n_weight = expf(-(n_error*n_error)/(2*sigma*sigma));
    float e_weight = expf(-(e_error*e_error)/(2*sigma*sigma));
    float s_weight = expf(-(s_error*s_error)/(2*sigma*sigma));
    float w_weight = expf(-(w_error*w_error)/(2*sigma*sigma));

    return n_weight * e_weight * s_weight * w_weight;

}

Particle MCL::weighted_random_sample(Particle particles[]) {

    std::random_device rd;
    std::mt19937 gen(rd());
    std::uniform_real_distribution<float> random_value(0, 1);

    float cumulative_weight = 0;

    for (int i=0; i < NUM_PARTICLES; i++) {

        cumulative_weight += particles[i].weight;

        if (cumulative_weight >= random_value(gen)) {
            return particles[i];
        }

    }

    return particles[NUM_PARTICLES-1];
}