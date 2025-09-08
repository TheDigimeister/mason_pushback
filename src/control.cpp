#include "main.h"
#include <limits>
#include <random>

// MCL (Monte Carlo Localization) Implementation - SIMPLIFIED FOR RELIABLE IMU
// Uses distance sensors on all four sides of the robot for pose estimation
// 
// MODIFICATIONS: 
// 1. Assumes IMU is very reliable for heading measurements
//    - All particles share the same heading from IMU (no angular uncertainty)
//    - MCL focuses only on position (x, y) estimation
//    - Removes angular noise and theta error calculations
// 2. Uses WEIGHTED SUM sensor fusion instead of multiplicative
//    - Robust to individual sensor failures/noise
//    - One bad sensor doesn't kill entire particle weight
//    - Averages likelihood across all valid sensors
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
const int NUM_PARTICLES = 1000;
const double FIELD_WIDTH = 144.0;  // VRC field width in inches
const double FIELD_LENGTH = 144.0; // VRC field length in inches
const double BASE_SENSOR_NOISE_STD = 0.5; // Base standard deviation for sensor noise
const double MOTION_NOISE_STD = 0.5; // Standard deviation for motion noise
// ANGLE_NOISE_STD removed - using reliable IMU heading directly

// Sensor weights for weighted sum fusion (can be adjusted based on sensor reliability)
const std::vector<double> SENSOR_WEIGHTS = {
    1.0,  // Front sensor weight
    1.0,  // Back sensor weight  
    1.0,  // Left sensor weight
    1.0   // Right sensor weight
};

// Field boundaries (VRC field centered at origin)
const double FIELD_X_MIN = -FIELD_WIDTH / 2.0;   // -72 inches
const double FIELD_X_MAX = FIELD_WIDTH / 2.0;    // +72 inches  
const double FIELD_Y_MIN = -FIELD_LENGTH / 2.0;  // -72 inches
const double FIELD_Y_MAX = FIELD_LENGTH / 2.0;   // +72 inches

// Robot dimensions (adjust based on your robot)
const double ROBOT_WIDTH = 11.5;  // Robot width in inches
const double ROBOT_LENGTH = 11.0; // Robot length in inches

// Distance from robot center to each sensor (adjust based on your robot)
const double FRONT_SENSOR_OFFSET = ROBOT_LENGTH / 2.0 + 1.0;
const double BACK_SENSOR_OFFSET = ROBOT_LENGTH / 2.0 - 1.0;
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
    std::normal_distribution<double> sensor_noise;
    
    lemlib::Pose last_pose;
    bool initialized;

public:
    // Helper function to keep particles within field bounds
    void clampParticleToField(Particle& particle) {
        particle.x = std::max(FIELD_X_MIN + ROBOT_WIDTH/2, std::min(particle.x, FIELD_X_MAX - ROBOT_WIDTH/2));
        particle.y = std::max(FIELD_Y_MIN + ROBOT_LENGTH/2, std::min(particle.y, FIELD_Y_MAX - ROBOT_LENGTH/2));
    }
    
    MCLController() : gen(rd()), 
                      motion_noise(0.0, MOTION_NOISE_STD),
                      sensor_noise(0.0, BASE_SENSOR_NOISE_STD),
                      initialized(false),
                      last_pose(chassis.getPose()) {
        particles.resize(NUM_PARTICLES);
        initializeParticles();
    }
    
    // Initialize particles randomly across the field
    void initializeParticles() {
        extern lemlib::Chassis chassis; // Reference to chassis from drive.cpp
        
        // Get current odometry pose for intelligent initialization
        lemlib::Pose odom_pose = chassis.getPose();
        
        // std::printf("MCL: Initializing particles around odometry pose (%.2f, %.2f, %.2f°)\n", 
        //        odom_pose.x, odom_pose.y, odom_pose.theta);
        
        // Initialize particles around current odometry position with some spread
        // Use IMU heading directly - no angular uncertainty
        std::normal_distribution<double> x_dist(odom_pose.x, 12.0);  // 12 inch standard deviation
        std::normal_distribution<double> y_dist(odom_pose.y, 12.0);  // 12 inch standard deviation
        double imu_heading_rad = odom_pose.theta * M_PI / 180.0; // Convert to radians, trust IMU
        
        for (auto& particle : particles) {
            particle.x = x_dist(gen);
            particle.y = y_dist(gen);
            particle.theta = imu_heading_rad; // All particles use same IMU heading
            particle.weight = 1.0 / NUM_PARTICLES;
            
            // Keep particles within field bounds
            clampParticleToField(particle);
        }
    }
    
    // Prediction step: move particles based on odometry
    void predict(const lemlib::Pose& current_pose) {
        if (!initialized) {
            last_pose = current_pose;
            initialized = true;
            return;
        }
        
        // Calculate motion delta (position only - ignore theta since using IMU directly)
        double dx = current_pose.x - last_pose.x;
        double dy = current_pose.y - last_pose.y;
        
        // Get current IMU heading for all particles
        double current_imu_heading = current_pose.theta * M_PI / 180.0; // Convert to radians
        
        // Move each particle
        for (auto& particle : particles) {
            // Add motion noise only to position
            double noisy_dx = dx + motion_noise(gen);
            double noisy_dy = dy + motion_noise(gen);
            
            // Update particle position
            particle.x += noisy_dx;
            particle.y += noisy_dy;
            particle.theta = current_imu_heading; // All particles use reliable IMU heading
            
            // Keep particles within field bounds
            clampParticleToField(particle);
        }
        
        last_pose = current_pose;
    }
    
    // Helper function to check ray intersection with a single wall
    double rayIntersectWall(double sensor_x, double sensor_y, double ray_dx, double ray_dy,
                           double wall_pos, bool is_vertical, double wall_min, double wall_max) {
        const double EPSILON = 1e-10;
        
        if (is_vertical) { // Vertical wall (constant X)
            if (std::abs(ray_dx) <= EPSILON) return std::numeric_limits<double>::max();
            double t = (wall_pos - sensor_x) / ray_dx;
            if (t <= 0.0) return std::numeric_limits<double>::max(); // Behind sensor
            double intersect_y = sensor_y + t * ray_dy;
            if (intersect_y >= wall_min && intersect_y <= wall_max) return t;
        } else { // Horizontal wall (constant Y)
            if (std::abs(ray_dy) <= EPSILON) return std::numeric_limits<double>::max();
            double t = (wall_pos - sensor_y) / ray_dy;
            if (t <= 0.0) return std::numeric_limits<double>::max(); // Behind sensor
            double intersect_x = sensor_x + t * ray_dx;
            if (intersect_x >= wall_min && intersect_x <= wall_max) return t;
        }
        return std::numeric_limits<double>::max();
    }
    
    // Get expected distance from particle to nearest obstacle using raycast in local sensor direction
    // VRC Coordinate System: X+ = Right, Y+ = North, 0° = North, Clockwise positive
    // Rotation math verification:
    // - At 0°: robot faces North (Y+), front sensor ray = (0,1), right sensor ray = (1,0)
    // - At 90°: robot faces East (X+), front sensor ray = (1,0), right sensor ray = (0,-1)
    // 
    // Obstacles detected: Field perimeter walls only (all other field geometry removed)
    double getExpectedDistance(const Particle& particle, int sensor_id) {
        double sensor_x, sensor_y;
        double ray_dx, ray_dy;  // Ray direction vector
        
        // Robot heading in radians (particle.theta)
        double robot_heading = particle.theta;
        
        // Calculate sensor position and ray direction in global coordinates
        switch (sensor_id) {
            case 0: // Front sensor (points forward relative to robot)
                // Position: offset forward from robot center
                sensor_x = particle.x + FRONT_SENSOR_OFFSET * sin(robot_heading);
                sensor_y = particle.y + FRONT_SENSOR_OFFSET * cos(robot_heading);
                // Ray direction: forward relative to robot (in global coordinates)
                ray_dx = sin(robot_heading);   // At 0°: sin(0°)=0, at 90°: sin(90°)=1 ✓
                ray_dy = cos(robot_heading);   // At 0°: cos(0°)=1, at 90°: cos(90°)=0 ✓
                break;
                
            case 1: // Back sensor (points backward relative to robot)
                // Position: offset backward from robot center
                sensor_x = particle.x - BACK_SENSOR_OFFSET * sin(robot_heading);
                sensor_y = particle.y - BACK_SENSOR_OFFSET * cos(robot_heading);
                // Ray direction: backward relative to robot (in global coordinates)
                ray_dx = -sin(robot_heading);
                ray_dy = -cos(robot_heading);
                break;
                
            case 2: // Left sensor (points left relative to robot)
                // Position: offset left from robot center
                sensor_x = particle.x - LEFT_SENSOR_OFFSET * cos(robot_heading);
                sensor_y = particle.y + LEFT_SENSOR_OFFSET * sin(robot_heading);
                // Ray direction: left relative to robot (90° counter-clockwise from forward)
                ray_dx = -cos(robot_heading);  // At 0°: -cos(0°)=-1, at 90°: -cos(90°)=0 ✓
                ray_dy = sin(robot_heading);   // At 0°: sin(0°)=0, at 90°: sin(90°)=1 ✓
                break;
                
            case 3: // Right sensor (points right relative to robot)
                // Position: offset right from robot center  
                sensor_x = particle.x + RIGHT_SENSOR_OFFSET * cos(robot_heading);
                sensor_y = particle.y - RIGHT_SENSOR_OFFSET * sin(robot_heading);
                // Ray direction: right relative to robot (90° clockwise from forward)
                ray_dx = cos(robot_heading);   // At 0°: cos(0°)=1, at 90°: cos(90°)=0 ✓
                ray_dy = -sin(robot_heading);  // At 0°: -sin(0°)=0, at 90°: -sin(90°)=-1 ✓
                break;
                
            default:
                return 0.0;
        }
        
        // Perform raycast to find intersection with field boundaries
        double min_distance = std::numeric_limits<double>::max();
        
        // Check intersection with all four walls
        struct {double pos; bool vertical; double min_coord; double max_coord;} walls[] = {
            {FIELD_Y_MAX, false, FIELD_X_MIN, FIELD_X_MAX}, // North wall
            {FIELD_Y_MIN, false, FIELD_X_MIN, FIELD_X_MAX}, // South wall  
            {FIELD_X_MAX, true, FIELD_Y_MIN, FIELD_Y_MAX},  // East wall
            {FIELD_X_MIN, true, FIELD_Y_MIN, FIELD_Y_MAX}   // West wall
        };
        
        for (const auto& wall : walls) {
            double wall_dist = rayIntersectWall(sensor_x, sensor_y, ray_dx, ray_dy,
                                               wall.pos, wall.vertical, wall.min_coord, wall.max_coord);
            if (wall_dist < min_distance) {
                min_distance = wall_dist;
            }
        }
        
        // Return the closest intersection distance (or reasonable default if no valid intersection)
        if (min_distance == std::numeric_limits<double>::max()) {
            // No valid intersection found - return a reasonable default distance
            // This can happen if sensor is pointing parallel to walls or in edge cases
            return 50.0; // Default to 50 inches (reasonable sensor reading)
        }
        
        // Clamp the result to reasonable sensor range
        return std::max(0.1, std::min(min_distance, 42.0)); // Clamp between 0.1 and 200 inches
    }
    
    // Helper function to process and validate a single sensor reading
    struct SensorReading {
        double value;
        bool is_valid;
        bool is_open_space;
        
        SensorReading(double raw_reading) {
            // Convert from mm to inches and validate
            value = std::isfinite(raw_reading) ? raw_reading / 25.4 : -1;
            is_valid = (value > 0 && value < 42);
            is_open_space = (value >= 42 && std::isfinite(value));
        }
    };
    
    // Update step: weight particles based on sensor measurements
    void update() {
        // Get and process sensor readings
        std::vector<SensorReading> sensors = {
            SensorReading(front_dist.get()),
            SensorReading(back_dist.get()),
            SensorReading(left_dist.get()),
            SensorReading(right_dist.get())
        };
        
        // Count sensors with usable information (either valid close readings or open space readings)
        int usable_sensor_count = 0;
        for (const auto& sensor : sensors) {
            if (sensor.is_valid || sensor.is_open_space) usable_sensor_count++;
        }
        
        // Skip update only if NO sensors provide usable information
        if (usable_sensor_count == 0) {
            return;
        }
        
        double total_weight = 0.0;
        
        // Update particle weights based on weighted sum sensor likelihood
        for (auto& particle : particles) {
            double weight_sum = 0.0; // Start with zero for addition
            double total_sensor_weight = 0.0; // Track total weight for normalization
            
            // Calculate likelihood for each sensor (additive approach)
            for (int i = 0; i < 4; i++) {
                // Use configurable sensor weights (can adjust for sensor reliability)
                double sensor_weight = (i < SENSOR_WEIGHTS.size()) ? SENSOR_WEIGHTS[i] : 1.0;
                
                if (sensors[i].is_valid) {
                    // Use actual sensor reading for valid close-range sensors
                    double expected = getExpectedDistance(particle, i);
                    double error = sensors[i].value - expected;
                    
                    // Gaussian likelihood function using base sensor noise
                    double likelihood = exp(-0.5 * (error * error) / (BASE_SENSOR_NOISE_STD * BASE_SENSOR_NOISE_STD));
                    weight_sum += sensor_weight * likelihood; // Additive weighting - robust to single bad sensor
                    total_sensor_weight += sensor_weight;
                } else if (sensors[i].is_open_space) {
                    // High readings indicate open space - give high likelihood if particle expects open space
                    double expected = getExpectedDistance(particle, i);
                    
                    if (expected >= 42.0) {
                        // Particle expects open space and sensor sees open space - high likelihood
                        weight_sum += sensor_weight * 0.95; // High confidence match
                    } else {
                        // Particle expects obstacle but sensor sees open space - low likelihood
                        weight_sum += sensor_weight * 0.1; // Disagreement penalty (but not fatal)
                    }
                    total_sensor_weight += sensor_weight;
                } else {
                    // Invalid sensors (≤0 or non-finite) - skip this sensor entirely
                    // Don't add to weight_sum or total_sensor_weight
                }
            }
            
            // Normalize by total sensor weight to get average likelihood
            if (total_sensor_weight > 0.0) {
                particle.weight = weight_sum / total_sensor_weight;
            } else {
                // No valid sensors - set neutral weight
                particle.weight = 0.5; // Neutral likelihood when no sensor information
            }
            total_weight += particle.weight;
        }
        
        // Normalize weights
        if (total_weight > 1e-10) {  // Use epsilon to avoid division by very small numbers
            for (auto& particle : particles) {
                particle.weight /= total_weight;
            }
        } else {
            // If total weight is too small or zero, reset all weights to uniform
            // std::printf("Warning: MCL total weight too small (%.2e), resetting to uniform distribution\n", total_weight);
            for (auto& particle : particles) {
                particle.weight = 1.0 / NUM_PARTICLES;
            }
        }
    }
    
    // Resample particles based on weights (hybrid: 80% systematic resampling, 20% random)
    void resample(const lemlib::Pose& current_pose) {
        
        std::vector<Particle> new_particles;
        new_particles.reserve(NUM_PARTICLES);
        
        // Calculate how many particles for each method
        int resampled_count = (NUM_PARTICLES * 95) / 100;      // 80% from systematic resampling
        int random_count = NUM_PARTICLES - resampled_count;  // 20% random (handles rounding)
        
        // Part 1: Systematic resampling for 80% of particles
        if (resampled_count > 0) {
            // Calculate cumulative weights
            std::vector<double> cumulative_weights(NUM_PARTICLES);
            cumulative_weights[0] = particles[0].weight;
            for (int i = 1; i < NUM_PARTICLES; i++) {
                cumulative_weights[i] = cumulative_weights[i-1] + particles[i].weight;
            }
            
            // Systematic resampling for 80% of particles
            std::uniform_real_distribution<double> uniform(0.0, 1.0 / resampled_count);
            double r = uniform(gen);
            
            int i = 0;
            for (int m = 0; m < resampled_count; m++) {
                double u = r + m / (double)resampled_count;
                while (u > cumulative_weights[i] && i < NUM_PARTICLES - 1) {
                    i++;
                }
                new_particles.push_back(particles[i]);
                new_particles.back().weight = 1.0 / NUM_PARTICLES;
            }
        }
        
        // Part 2: Generate random particles for remaining 20%
        // Get current estimated pose for intelligent random particle generation
        lemlib::Pose estimated_pose = getEstimatedPose(current_pose);
        bool use_pose_guidance = std::isfinite(estimated_pose.x) && std::isfinite(estimated_pose.y) && std::isfinite(estimated_pose.theta);
        
        // bool use_pose_guidance = false;
        if (use_pose_guidance) {
            // Generate random particles around current estimated pose (with wider spread than initialization)
            // Use IMU heading directly - no angular uncertainty
            std::normal_distribution<double> x_dist(estimated_pose.x, 12.0);  // 20 inch standard deviation
            std::normal_distribution<double> y_dist(estimated_pose.y, 12.0);  // 20 inch standard deviation
            double current_imu_heading = estimated_pose.theta * M_PI / 180.0; // Trust IMU heading
            
            for (int m = 0; m < random_count; m++) {
                Particle random_particle;
                random_particle.x = x_dist(gen);
                random_particle.y = y_dist(gen);
                random_particle.theta = current_imu_heading; // All particles use same IMU heading
                random_particle.weight = 1.0 / NUM_PARTICLES;
                
                // Keep particles within field bounds
                clampParticleToField(random_particle);
                
                // Validate particle before adding
                if (std::isfinite(random_particle.x) && std::isfinite(random_particle.y) && 
                    std::isfinite(random_particle.theta) && std::isfinite(random_particle.weight)) {
                    new_particles.push_back(random_particle);
                } else {
                    // Use odometry-centered distribution if guided generation fails
                    std::normal_distribution<double> x_odom(current_pose.x, 12.0);  // 15 inch std dev around odom
                    std::normal_distribution<double> y_odom(current_pose.y, 12.0);
                    double imu_heading_rad = current_pose.theta * M_PI / 180.0; // Use IMU heading directly
                    
                    random_particle.x = x_odom(gen);
                    random_particle.y = y_odom(gen);
                    random_particle.theta = imu_heading_rad; // Use IMU heading directly
                    
                    // Keep within bounds
                    clampParticleToField(random_particle);
                    
                    random_particle.weight = 1.0 / NUM_PARTICLES;
                    new_particles.push_back(random_particle);
                }
            }
        } else {
            // Use odometry-centered random distribution when MCL pose estimate is invalid
            // Generate particles around odometry pose
            // Use IMU heading directly - no angular uncertainty
            std::normal_distribution<double> x_dist(current_pose.x, 12.0);  // 15 inch standard deviation
            std::normal_distribution<double> y_dist(current_pose.y, 12.0);  // 15 inch standard deviation
            double current_imu_heading = current_pose.theta * M_PI / 180.0; // Trust IMU heading
            
            for (int m = 0; m < random_count; m++) {
                Particle random_particle;
                random_particle.x = x_dist(gen);
                random_particle.y = y_dist(gen);
                random_particle.theta = current_imu_heading; // All particles use same IMU heading
                random_particle.weight = 1.0 / NUM_PARTICLES;
                
                // Keep particles within field bounds
                clampParticleToField(random_particle);
                
                new_particles.push_back(random_particle);
            }
        }
        
        particles = std::move(new_particles);
    }
    
    // Overloaded version that calls chassis.getPose() for backward compatibility
    void resample() {
        extern lemlib::Chassis chassis; // Reference to chassis from drive.cpp
        resample(chassis.getPose());
    }
    
    // Get estimated pose (weighted average of particles for position, IMU for heading)
    lemlib::Pose getEstimatedPose(const lemlib::Pose& current_pose) {
        double x_sum = 0.0, y_sum = 0.0;
        double weight_sum = 0.0;
        
        for (const auto& particle : particles) {
            x_sum += particle.x * particle.weight;
            y_sum += particle.y * particle.weight;
            weight_sum += particle.weight;
        }
        
        // Use IMU heading directly (already in degrees from current pose)
        double estimated_theta = current_pose.theta; 
        
        return lemlib::Pose(x_sum / weight_sum, y_sum / weight_sum, estimated_theta);
    }
    
    // Overloaded version that calls chassis.getPose() for backward compatibility
    lemlib::Pose getEstimatedPose() {
        extern lemlib::Chassis chassis; // Reference to chassis from drive.cpp
        return getEstimatedPose(chassis.getPose());
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
    bool isConverged(double threshold = 6.0) {
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
lemlib::Pose updateMCL() {
    extern lemlib::Chassis chassis; // Reference to chassis from drive.cpp
    
    // Get current odometry pose ONCE per MCL loop (optimization: shared across predict/update/resample steps)
    lemlib::Pose current_pose = chassis.getPose();
    
    // Prediction step (uses odometry deltas for particle movement)
    mcl_controller.predict(current_pose);
    
    // Update step (measurement update using distance sensors)
    mcl_controller.update();
    
    // Resample particles (uses the same pose to avoid redundant chassis.getPose() calls)
    mcl_controller.resample(current_pose);
    
    // Return the pose so it can be shared with other functions
    return current_pose;
    
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
void updateChassisPoseFromMCL(bool force_update, const lemlib::Pose& current_pose) {
    extern pros::Controller master; // Reference to master controller from main.cpp
    
    // Only update if MCL has converged or if forced, and limit frequency
    static uint32_t last_update_time = 0;
    static uint32_t last_controller_update_time = 0;
    uint32_t current_time = pros::millis();
    
    // Only update every 5 seconds (5000ms) when converged, or immediately if forced
    bool time_to_update = (current_time - last_update_time) > 5000;
    
    if ((force_update || (mcl_controller.isConverged() && time_to_update))) { // Only if uncertainty is low
        
        lemlib::Pose mcl_pose = mcl_controller.getEstimatedPose(current_pose);
        
        // Calculate the correction needed
        lemlib::Pose error = mcl_controller.calculateGlobalError(current_pose);
        
        // Only apply correction if error is significant but not too large
        double position_error = sqrt(error.x * error.x + error.y * error.y);
        if (position_error > 2.0 && position_error < 24.0) { // Between 2-24 inches
            extern lemlib::Chassis chassis; // Reference to chassis from drive.cpp
            chassis.setPose(mcl_pose);
            last_update_time = current_time;
            
            // std::printf("MCL Correction Applied: Pos Error=%.2f, Angle Error=%.2f\n", 
                //    position_error, error.theta);
            
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
        lemlib::Pose mcl_pose = mcl_controller.getEstimatedPose(current_pose);
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

// Overloaded version that calls chassis.getPose() for backward compatibility
void updateChassisPoseFromMCL(bool force_update) {
    extern lemlib::Chassis chassis; // Reference to chassis from drive.cpp
    updateChassisPoseFromMCL(force_update, chassis.getPose());
}

// MCL task function to run in background
void mclTask(void* param) {
    while (true) {
        // Get current pose and update MCL - share the pose to avoid redundant chassis.getPose() calls
        lemlib::Pose current_pose = updateMCL();
        
        // Optionally update chassis pose periodically when converged
        // Uncomment the line below to enable automatic chassis pose updates
        updateChassisPoseFromMCL(false, current_pose);
        
        pros::delay(50); // Update at 20Hz
    }
}

// Initialize MCL system
void initializeMCL() {
    pros::Task mcl_task(mclTask, (void*)"MCL");
}
