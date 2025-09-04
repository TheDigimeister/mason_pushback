#include "control.hpp"
#include "odom.hpp"
#include <cmath>
#include <cstdlib>
#include <random>
#include "main.h"

#define M_PI 3.14159265358979323846

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

void MCL::update(MCLSensors &sensors) {
    for (auto& p: this->particles) {
        p.weight = sensor_model(p, sensors, 1.0);
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

    this->estimated_pose = {x_est, y_est, theta_est};  // Store the result

}

Particle MCL::motion_model(Particle p) {
    std::random_device rd;
    std::mt19937 gen(rd());
    std::normal_distribution<float> xy_noise(0, 0.5);  // 0.5 inch noise
    std::normal_distribution<float> theta_noise(0, 0.05);  // ~3 degree noise
    
    float x_pred = p.x + odom_state.get_global_delta_x() + xy_noise(gen);
    float y_pred = p.y + odom_state.get_global_delta_y() + xy_noise(gen);
    float theta_pred = p.theta + odom_state.get_global_delta_theta() + theta_noise(gen);
    return Particle {x_pred, y_pred, theta_pred, p.weight};
}

float MCL::sensor_model(Particle p, MCLSensors &mcl_sensors, float sigma) {
    
    // Get current robot heading from IMU (in radians)
    float robot_heading = mcl_sensors.imu.get_heading() * M_PI / 180.0;
    
    // Distance sensor readings in mm, convert to inches
    float front_reading = mcl_sensors.front_dist.get_distance() * MM_TO_INCHES;
    float back_reading = mcl_sensors.back_dist.get_distance() * MM_TO_INCHES;
    float left_reading = mcl_sensors.left_dist.get_distance() * MM_TO_INCHES;
    float right_reading = mcl_sensors.right_dist.get_distance() * MM_TO_INCHES;
    
    // Calculate sensor direction vectors in global frame
    // Assuming sensors are mounted: front=+x, right=+y, back=-x, left=-y in robot frame
    float cos_heading = cosf(robot_heading);
    float sin_heading = sinf(robot_heading);
    
    // Sensor direction unit vectors in global frame
    float front_dir_x = cos_heading;
    float front_dir_y = sin_heading;
    
    float back_dir_x = -cos_heading;
    float back_dir_y = -sin_heading;
    
    float left_dir_x = -sin_heading;
    float left_dir_y = cos_heading;
    
    float right_dir_x = sin_heading;
    float right_dir_y = -cos_heading;
    
    // Raycast from particle position to find expected distances to walls
    float expected_front_dist = raycast_to_wall(p.x, p.y, front_dir_x, front_dir_y);
    float expected_back_dist = raycast_to_wall(p.x, p.y, back_dir_x, back_dir_y);
    float expected_left_dist = raycast_to_wall(p.x, p.y, left_dir_x, left_dir_y);
    float expected_right_dist = raycast_to_wall(p.x, p.y, right_dir_x, right_dir_y);
    
    // Calculate errors between actual and expected sensor readings
    float front_error = fabs(front_reading - expected_front_dist);
    float back_error = fabs(back_reading - expected_back_dist);
    float left_error = fabs(left_reading - expected_left_dist);
    float right_error = fabs(right_reading - expected_right_dist);
    
    // Calculate Gaussian weights for each sensor
    float front_weight = expf(-(front_error * front_error) / (2 * sigma * sigma));
    float back_weight = expf(-(back_error * back_error) / (2 * sigma * sigma));
    float left_weight = expf(-(left_error * left_error) / (2 * sigma * sigma));
    float right_weight = expf(-(right_error * right_error) / (2 * sigma * sigma));

    return front_weight * back_weight * left_weight * right_weight;
}

Particle MCL::weighted_random_sample(Particle particles[]) {

    std::random_device rd;
    std::mt19937 gen(rd());
    std::uniform_real_distribution<float> dist(0, 1);

    float random_value = dist(gen);  // Generate random value ONCE
    float cumulative_weight = 0;

    for (int i=0; i < NUM_PARTICLES; i++) {

        cumulative_weight += particles[i].weight;

        if (cumulative_weight >= random_value) {
            return particles[i];
        }

    }

    return particles[NUM_PARTICLES-1];
}

float MCL::raycast_to_wall(float start_x, float start_y, float dir_x, float dir_y) {
    // Field boundaries: x=[0, 144], y=[0, 144] inches
    const float FIELD_MIN = 0.0;
    const float FIELD_MAX = 144.0;
    
    // Calculate distances to each wall along the ray direction
    float distances[4];
    int valid_intersections = 0;
    
    // Check intersection with west wall (x = 0)
    if (dir_x < 0) {  // Ray pointing left
        float t = (FIELD_MIN - start_x) / dir_x;
        float intersect_y = start_y + t * dir_y;
        if (t > 0 && intersect_y >= FIELD_MIN && intersect_y <= FIELD_MAX) {
            distances[valid_intersections++] = t;
        }
    }
    
    // Check intersection with east wall (x = 144)
    if (dir_x > 0) {  // Ray pointing right
        float t = (FIELD_MAX - start_x) / dir_x;
        float intersect_y = start_y + t * dir_y;
        if (t > 0 && intersect_y >= FIELD_MIN && intersect_y <= FIELD_MAX) {
            distances[valid_intersections++] = t;
        }
    }
    
    // Check intersection with south wall (y = 0)
    if (dir_y < 0) {  // Ray pointing down
        float t = (FIELD_MIN - start_y) / dir_y;
        float intersect_x = start_x + t * dir_x;
        if (t > 0 && intersect_x >= FIELD_MIN && intersect_x <= FIELD_MAX) {
            distances[valid_intersections++] = t;
        }
    }
    
    // Check intersection with north wall (y = 144)
    if (dir_y > 0) {  // Ray pointing up
        float t = (FIELD_MAX - start_y) / dir_y;
        float intersect_x = start_x + t * dir_x;
        if (t > 0 && intersect_x >= FIELD_MIN && intersect_x <= FIELD_MAX) {
            distances[valid_intersections++] = t;
        }
    }
    
    // Return the minimum distance (closest wall)
    if (valid_intersections > 0) {
        float min_distance = distances[0];
        for (int i = 1; i < valid_intersections; i++) {
            if (distances[i] < min_distance) {
                min_distance = distances[i];
            }
        }
        return min_distance;
    }
    
    // If no valid intersection found, return a large value
    return 1000.0;
}

PID::PID(float kP, float kI, float kD) : kP(kP), kI(kI), kD(kD) {};

float PID::update(float target, float current) {
    float error = target - current;
    this->integral += error;
    float derivative = error - this->previous_error;
    return this->kD * error + this->kI * integral + this->kD * derivative;
};