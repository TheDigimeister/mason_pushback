#include "main.h"
// #include "robot.hpp"
// #include "lemlib/api.hpp"
// #include "pros/distance.hpp"
// #include "pros/rtos.hpp"
// #include <vector>
// #include <random>
// #include <cmath>
// #include <algorithm>

// MCL (Monte Carlo Localization) Implementation
// Uses distance sensors on all four sides of the robot for pose estimation
// 
// VRC/LemLib Coordinate System:
// - X+ = Right (East direction)
// - Y+ = Forward/North (towards opponent side) 
// - Angle 0° = North (Y+ direction)
// - Positive angles = Clockwise rotation
// - Field center at origin (0, 0)
// - Field bounds: X ∈ [-72, +72], Y ∈ [-72, +72] inches
//
// Sensor Layout (relative to robot):
// - Front sensor: points forward (Y+ direction when robot heading = 0°)
// - Back sensor: points backward (Y- direction when robot heading = 0°)  
// - Left sensor: points left (X- direction when robot heading = 0°)
// - Right sensor: points right (X+ direction when robot heading = 0°)

// MCL Configuration
const int NUM_PARTICLES = 500;
const double FIELD_WIDTH = 144.0;  // VRC field width in inches
const double FIELD_LENGTH = 144.0; // VRC field length in inches
const double SENSOR_NOISE_STD = 2.0; // Standard deviation for sensor noise
const double MOTION_NOISE_STD = 0.5; // Standard deviation for motion noise
const double ANGLE_NOISE_STD = 0.1; // Standard deviation for angular noise

// Field boundaries (VRC field centered at origin)
const double FIELD_X_MIN = -FIELD_WIDTH / 2.0;   // -72 inches
const double FIELD_X_MAX = FIELD_WIDTH / 2.0;    // +72 inches  
const double FIELD_Y_MIN = -FIELD_LENGTH / 2.0;  // -72 inches
const double FIELD_Y_MAX = FIELD_LENGTH / 2.0;   // +72 inches

// Robot dimensions (adjust based on your robot)
const double ROBOT_WIDTH = 18.0;  // Robot width in inches
const double ROBOT_LENGTH = 18.0; // Robot length in inches

// Distance from robot center to each sensor (adjust based on your robot)
const double FRONT_SENSOR_OFFSET = ROBOT_LENGTH / 2.0;
const double BACK_SENSOR_OFFSET = ROBOT_LENGTH / 2.0;
const double LEFT_SENSOR_OFFSET = ROBOT_WIDTH / 2.0;
const double RIGHT_SENSOR_OFFSET = ROBOT_WIDTH / 2.0;

// Particle structure
struct Particle {
    double x;      // X position in inches
    double y;      // Y position in inches
    double theta;  // Heading in radians
    double weight; // Particle weight
};

// MCL Class
class MCLController {
private:
    std::vector<Particle> particles;
    std::random_device rd;
    std::mt19937 gen;
    std::normal_distribution<double> motion_noise;
    std::normal_distribution<double> angle_noise;
    std::normal_distribution<double> sensor_noise;
    
    lemlib::Pose last_pose;
    bool initialized;

public:
    MCLController() : gen(rd()), 
                      motion_noise(0.0, MOTION_NOISE_STD),
                      angle_noise(0.0, ANGLE_NOISE_STD),
                      sensor_noise(0.0, SENSOR_NOISE_STD),
                      initialized(false),
                      last_pose(lemlib::Pose(0,0,0)) {
        particles.resize(NUM_PARTICLES);
        initializeParticles();
    }
    
    // Initialize particles randomly across the field
    void initializeParticles() {
        std::uniform_real_distribution<double> x_dist(FIELD_X_MIN + ROBOT_WIDTH/2, FIELD_X_MAX - ROBOT_WIDTH/2);
        std::uniform_real_distribution<double> y_dist(FIELD_Y_MIN + ROBOT_LENGTH/2, FIELD_Y_MAX - ROBOT_LENGTH/2);
        std::uniform_real_distribution<double> theta_dist(-M_PI, M_PI);
        
        for (auto& particle : particles) {
            particle.x = x_dist(gen);
            particle.y = y_dist(gen);
            particle.theta = theta_dist(gen);
            particle.weight = 1.0 / NUM_PARTICLES;
        }
    }
    
    // Prediction step: move particles based on odometry
    void predict(const lemlib::Pose& current_pose) {
        if (!initialized) {
            last_pose = current_pose;
            initialized = true;
            return;
        }
        
        // Calculate motion delta
        double dx = current_pose.x - last_pose.x;
        double dy = current_pose.y - last_pose.y;
        double dtheta_degrees = current_pose.theta - last_pose.theta;
        
        // Convert angle difference to radians for particle calculations
        double dtheta = dtheta_degrees * M_PI / 180.0;
        
        // Normalize angle difference (in radians)
        while (dtheta > M_PI) dtheta -= 2 * M_PI;
        while (dtheta < -M_PI) dtheta += 2 * M_PI;
        
        // Move each particle
        for (auto& particle : particles) {
            // Add motion noise
            double noisy_dx = dx + motion_noise(gen);
            double noisy_dy = dy + motion_noise(gen);
            double noisy_dtheta = dtheta + angle_noise(gen);
            
            // Update particle pose
            particle.x += noisy_dx;
            particle.y += noisy_dy;
            particle.theta += noisy_dtheta;
            
            // Normalize angle (keep in radians)
            while (particle.theta > M_PI) particle.theta -= 2 * M_PI;
            while (particle.theta < -M_PI) particle.theta += 2 * M_PI;
            
            // Keep particles within field bounds
            particle.x = std::max(FIELD_X_MIN + ROBOT_WIDTH/2, std::min(particle.x, FIELD_X_MAX - ROBOT_WIDTH/2));
            particle.y = std::max(FIELD_Y_MIN + ROBOT_LENGTH/2, std::min(particle.y, FIELD_Y_MAX - ROBOT_LENGTH/2));
        }
        
        last_pose = current_pose;
    }
    
    // Get expected distance from particle to wall in global coordinates
    // VRC Coordinate System: X+ = Right, Y+ = North, 0° = North, Clockwise positive
    double getExpectedDistance(const Particle& particle, int sensor_id) {
        double sensor_x, sensor_y;
        double expected_dist = 0.0;
        
        // Convert particle theta from radians to LemLib convention
        // LemLib: 0° = North (Y+), clockwise positive
        // Our particles use standard math convention, so we need to convert
        double robot_heading = particle.theta;
        
        // Calculate sensor position in global coordinates relative to robot center
        switch (sensor_id) {
            case 0: // Front sensor (points forward relative to robot)
                sensor_x = particle.x + FRONT_SENSOR_OFFSET * sin(robot_heading);
                sensor_y = particle.y + FRONT_SENSOR_OFFSET * cos(robot_heading);
                // Distance to north wall (Y = FIELD_Y_MAX)
                expected_dist = FIELD_Y_MAX - sensor_y;
                break;
                
            case 1: // Back sensor (points backward relative to robot)
                sensor_x = particle.x - BACK_SENSOR_OFFSET * sin(robot_heading);
                sensor_y = particle.y - BACK_SENSOR_OFFSET * cos(robot_heading);
                // Distance to south wall (Y = FIELD_Y_MIN)
                expected_dist = sensor_y - FIELD_Y_MIN;
                break;
                
            case 2: // Left sensor (points left relative to robot)
                sensor_x = particle.x - LEFT_SENSOR_OFFSET * cos(robot_heading);
                sensor_y = particle.y + LEFT_SENSOR_OFFSET * sin(robot_heading);
                // Distance to west wall (X = FIELD_X_MIN)
                expected_dist = sensor_x - FIELD_X_MIN;
                break;
                
            case 3: // Right sensor (points right relative to robot)
                sensor_x = particle.x + RIGHT_SENSOR_OFFSET * cos(robot_heading);
                sensor_y = particle.y - RIGHT_SENSOR_OFFSET * sin(robot_heading);
                // Distance to east wall (X = FIELD_X_MAX)
                expected_dist = FIELD_X_MAX - sensor_x;
                break;
                
            default:
                expected_dist = 0;
        }
        
        // Ensure distance is positive (sensors can't see through walls)
        return std::max(0.0, expected_dist);
    }
    
    // Update step: weight particles based on sensor measurements
    void update() {
        // Get sensor readings (convert from mm to inches)
        double front_reading = front_dist.get() / 25.4;
        double back_reading = back_dist.get() / 25.4;
        double left_reading = left_dist.get() / 25.4;
        double right_reading = right_dist.get() / 25.4;
        
        // Check which sensors have valid readings
        std::vector<double> readings = {front_reading, back_reading, left_reading, right_reading};
        std::vector<bool> sensor_valid = {
            front_reading > 0 && front_reading < 200,  // Valid range: 0-200 inches
            back_reading > 0 && back_reading < 200,
            left_reading > 0 && left_reading < 200,
            right_reading > 0 && right_reading < 200
        };
        
        // Count valid sensors
        int valid_sensor_count = 0;
        for (bool valid : sensor_valid) {
            if (valid) valid_sensor_count++;
        }
        
        // Skip update only if NO sensors are valid
        if (valid_sensor_count == 0) {
            return;
        }
        
        double total_weight = 0.0;
        
        // Update particle weights based on sensor likelihood
        for (auto& particle : particles) {
            double weight = 1.0;
            
            // Calculate likelihood for each sensor
            for (int i = 0; i < 4; i++) {
                if (sensor_valid[i]) {
                    // Use actual sensor reading for valid sensors
                    double expected = getExpectedDistance(particle, i);
                    double error = readings[i] - expected;
                    
                    // Gaussian likelihood function
                    double likelihood = exp(-0.5 * (error * error) / (SENSOR_NOISE_STD * SENSOR_NOISE_STD));
                    weight *= likelihood;
                } else {
                    // For invalid sensors, apply neutral weight (no information gain/loss)
                    // This is equivalent to saying "we don't know what this sensor should read"
                    // Alternative: weight *= 1.0; (no change)
                    // Alternative: weight *= 0.5; (slight penalty for uncertainty)
                    weight *= 1.0; // Neutral - no information from this sensor
                }
            }
            
            particle.weight = weight;
            total_weight += weight;
        }
        
        // Normalize weights
        if (total_weight > 0) {
            for (auto& particle : particles) {
                particle.weight /= total_weight;
            }
        }
    }
    
    // Resample particles based on weights (systematic resampling)
    void resample() {
        std::vector<Particle> new_particles;
        new_particles.reserve(NUM_PARTICLES);
        
        // Calculate cumulative weights
        std::vector<double> cumulative_weights(NUM_PARTICLES);
        cumulative_weights[0] = particles[0].weight;
        for (int i = 1; i < NUM_PARTICLES; i++) {
            cumulative_weights[i] = cumulative_weights[i-1] + particles[i].weight;
        }
        
        // Systematic resampling
        std::uniform_real_distribution<double> uniform(0.0, 1.0 / NUM_PARTICLES);
        double r = uniform(gen);
        
        int i = 0;
        for (int m = 0; m < NUM_PARTICLES; m++) {
            double u = r + m / (double)NUM_PARTICLES;
            while (u > cumulative_weights[i]) {
                i++;
            }
            new_particles.push_back(particles[i]);
            new_particles.back().weight = 1.0 / NUM_PARTICLES;
        }
        
        particles = std::move(new_particles);
    }
    
    // Get estimated pose (weighted average of particles)
    lemlib::Pose getEstimatedPose() {
        double x_sum = 0.0, y_sum = 0.0;
        double sin_sum = 0.0, cos_sum = 0.0;
        
        for (const auto& particle : particles) {
            x_sum += particle.x * particle.weight;
            y_sum += particle.y * particle.weight;
            sin_sum += sin(particle.theta) * particle.weight;
            cos_sum += cos(particle.theta) * particle.weight;
        }
        
        double estimated_theta = atan2(sin_sum, cos_sum);
        
        return lemlib::Pose(x_sum, y_sum, estimated_theta * 180.0 / M_PI); // Convert to degrees for LemLib
    }
    
    // Calculate pose error in global coordinates
    lemlib::Pose calculateGlobalError(const lemlib::Pose& reference_pose) {
        lemlib::Pose estimated = getEstimatedPose();
        
        // Calculate position error in global coordinates
        double dx = estimated.x - reference_pose.x;
        double dy = estimated.y - reference_pose.y;
        
        // Calculate angular error
        double dtheta = estimated.theta - reference_pose.theta;
        while (dtheta > 180.0) dtheta -= 360.0;
        while (dtheta < -180.0) dtheta += 360.0;
        
        return lemlib::Pose(dx, dy, dtheta);
    }
    
    // Get particle spread (measure of uncertainty)
    double getParticleSpread() {
        lemlib::Pose mean = getEstimatedPose();
        double spread = 0.0;
        
        for (const auto& particle : particles) {
            double dx = particle.x - mean.x;
            double dy = particle.y - mean.y;
            spread += sqrt(dx * dx + dy * dy) * particle.weight;
        }
        
        return spread;
    }
    
    // Check if localization is converged
    bool isConverged(double threshold = 5.0) {
        return getParticleSpread() < threshold;
    }
    
    // Reset MCL (useful for kidnapped robot problem)
    void reset() {
        initializeParticles();
        initialized = false;
    }
};

// Global MCL controller instance
MCLController mcl_controller;

// Main MCL update function to be called periodically
void updateMCL() {
    extern lemlib::Chassis chassis; // Reference to chassis from drive.cpp
    
    // Get current odometry pose (this might include previous MCL corrections)
    lemlib::Pose current_pose = chassis.getPose();
    
    // Prediction step (uses odometry deltas for particle movement)
    mcl_controller.predict(current_pose);
    
    // Update step (measurement update using distance sensors)
    mcl_controller.update();
    
    // Resample particles
    mcl_controller.resample();
    
    // Note: We don't automatically update chassis pose here to avoid circular dependency
    // Use updateChassisPoseFromMCL() when you want to apply MCL corrections
}

// Get MCL estimated pose
lemlib::Pose getMCLPose() {
    return mcl_controller.getEstimatedPose();
}

// Get pose error in global coordinates
lemlib::Pose getMCLError(const lemlib::Pose& reference_pose) {
    return mcl_controller.calculateGlobalError(reference_pose);
}

// Check if MCL has converged
bool isMCLConverged() {
    return mcl_controller.isConverged();
}

// Get localization uncertainty
double getMCLUncertainty() {
    return mcl_controller.getParticleSpread();
}

// Reset MCL
void resetMCL() {
    mcl_controller.reset();
}

// Update the chassis pose with the MCL estimated pose
void updateChassisPoseFromMCL(bool force_update) {
    extern lemlib::Chassis chassis; // Reference to chassis from drive.cpp
    extern pros::Controller master; // Reference to master controller from main.cpp
    
    // Only update if MCL has converged or if forced, and limit frequency
    static uint32_t last_update_time = 0;
    static uint32_t last_controller_update_time = 0;
    uint32_t current_time = pros::millis();
    
    // Only update every 5 seconds (5000ms) when converged, or immediately if forced
    bool time_to_update = (current_time - last_update_time) > 5000;
    
    if ((force_update || (mcl_controller.isConverged() && time_to_update)) && 
        mcl_controller.getParticleSpread() < 3.0) { // Only if uncertainty is low
        
        lemlib::Pose mcl_pose = mcl_controller.getEstimatedPose();
        lemlib::Pose current_pose = chassis.getPose();
        
        // Calculate the correction needed
        lemlib::Pose error = mcl_controller.calculateGlobalError(current_pose);
        
        // Only apply correction if error is significant but not too large
        double position_error = sqrt(error.x * error.x + error.y * error.y);
        if (position_error > 2.0 && position_error < 24.0) { // Between 2-24 inches
            chassis.setPose(mcl_pose);
            last_update_time = current_time;
            
            printf("MCL Correction Applied: Pos Error=%.2f, Angle Error=%.2f\n", 
                   position_error, error.theta);
            
            // Display MCL convergence and pose update on controller screen
            // Update controller screen every 500ms to avoid overwhelming the display
            if ((current_time - last_controller_update_time) > 500) {
                master.clear();
                pros::delay(50);
                master.print(0, 0, "MCL CONVERGED!");
                pros::delay(50);
                master.print(1, 0, "X:%.1f Y:%.1f", mcl_pose.x, mcl_pose.y);
                pros::delay(50);
                master.print(2, 0, "Theta:%.1f Err:%.1f", mcl_pose.theta, position_error);
                pros::delay(50);
                last_controller_update_time = current_time;
            }
        }
    }
    
    // Also display convergence status periodically even when not updating pose
    if (mcl_controller.isConverged() && (current_time - last_controller_update_time) > 2000) {
        lemlib::Pose mcl_pose = mcl_controller.getEstimatedPose();
        double uncertainty = mcl_controller.getParticleSpread();
        
        master.clear();
        pros::delay(50);
        master.print(0, 0, "MCL: GOOD");
        pros::delay(50);
        master.print(1, 0, "X:%.1f Y:%.1f", mcl_pose.x, mcl_pose.y);
        pros::delay(50);
        master.print(2, 0, "H:%.1f U:%.1f", mcl_pose.theta, uncertainty);
        pros::delay(50);
        last_controller_update_time = current_time;
    } else if (!mcl_controller.isConverged() && (current_time - last_controller_update_time) > 3000) {
        double uncertainty = mcl_controller.getParticleSpread();
        
        master.clear();
        pros::delay(50);
        master.print(0, 0, "MCL: SEARCHING");
        pros::delay(50);
        master.print(1, 0, "Uncertainty:");
        pros::delay(50);
        master.print(2, 0, "%.1f inches", uncertainty);
        pros::delay(50);
        last_controller_update_time = current_time;
    }
}

// MCL task function to run in background
void mclTask(void* param) {
    while (true) {
        updateMCL();
        
        // Optionally update chassis pose periodically when converged
        // Uncomment the line below to enable automatic chassis pose updates
        // updateChassisPoseFromMCL(false);
        
        pros::delay(50); // Update at 20Hz
    }
}

// Initialize MCL system
void initializeMCL() {
    pros::Task mcl_task(mclTask, (void*)"MCL");
}
