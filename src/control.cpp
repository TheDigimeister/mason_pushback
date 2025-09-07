#include "main.h"
#include <limits>

// MCL (Monte Carlo Localization) Implementation - SIMPLIFIED FOR RELIABLE IMU
// Uses distance sensors on all four sides of the robot for pose estimation
// 
// MODIFICATION: Assumes IMU is very reliable for heading measurements
// - All particles share the same heading from IMU (no angular uncertainty)
// - MCL focuses only on position (x, y) estimation
// - Removes angular noise and theta error calculations
// - Simplified particle filter with better computational efficiency
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
const double BASE_SENSOR_NOISE_STD = 1.5; // Base standard deviation for sensor noise
const double MOTION_NOISE_STD = 1.0; // Standard deviation for motion noise
// ANGLE_NOISE_STD removed - using reliable IMU heading directly

// Field boundaries (VRC field centered at origin)
const double FIELD_X_MIN = -FIELD_WIDTH / 2.0;   // -72 inches
const double FIELD_X_MAX = FIELD_WIDTH / 2.0;    // +72 inches  
const double FIELD_Y_MIN = -FIELD_LENGTH / 2.0;  // -72 inches
const double FIELD_Y_MAX = FIELD_LENGTH / 2.0;   // +72 inches

// Robot dimensions (adjust based on your robot)
const double ROBOT_WIDTH = 11.5;  // Robot width in inches
const double ROBOT_LENGTH = 8.5; // Robot length in inches

// Distance from robot center to each sensor (adjust based on your robot)
const double FRONT_SENSOR_OFFSET = ROBOT_LENGTH / 2.0;
const double BACK_SENSOR_OFFSET = ROBOT_LENGTH / 2.0;
const double LEFT_SENSOR_OFFSET = ROBOT_WIDTH / 2.0;
const double RIGHT_SENSOR_OFFSET = ROBOT_WIDTH / 2.0;

// VEX Push Back Field Obstacles (from official game manual)
// Center Goals: 22.6" length, stacked on top of each other at field center
const double CENTER_GOAL_LENGTH = 22.6;
const double CENTER_GOAL_WIDTH = 22.6;  // Estimated width for collision detection
const double CENTER_GOAL_HEIGHT = 12.0; // Height affects sensor detection

// Center Goals (Upper and Lower stacked at field center)
const double CENTER_GOAL_X = 0.0;    // Centered on field
const double CENTER_GOAL_Y = 0.0;    // Centered on field

// Loaders: 21.34" tall, positioned at field locations
const double LOADER_HEIGHT = 21.34;
const double LOADER_WIDTH = 8.0;   // Estimated width for collision detection
const double LOADER_DEPTH = 8.0;   // Estimated depth for collision detection

// Red Alliance Loaders
const double RED_LOADER_1_X = -67.0;  // Left red loader
const double RED_LOADER_1_Y = 47.0;   // Upper position
const double RED_LOADER_2_X = -67.0;  // Right red loader
const double RED_LOADER_2_Y = -47.0;  // Lower position

// Blue Alliance Loaders
const double BLUE_LOADER_1_X = 67.0;   // Left blue loader
const double BLUE_LOADER_1_Y = 47.0;   // Upper position
const double BLUE_LOADER_2_X = 67.0;   // Right blue loader
const double BLUE_LOADER_2_Y = -47.0;  // Lower position

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
        
        // Try to get current odometry pose for intelligent initialization
        lemlib::Pose odom_pose = chassis.getPose();
        bool use_odom_center = std::isfinite(odom_pose.x) && std::isfinite(odom_pose.y) && std::isfinite(odom_pose.theta);
        
        if (use_odom_center) {
            std::printf("MCL: Initializing particles around odometry pose (%.2f, %.2f, %.2f°)\n", 
                   odom_pose.x, odom_pose.y, odom_pose.theta);
            
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
                particle.x = std::max(FIELD_X_MIN + ROBOT_WIDTH/2, std::min(particle.x, FIELD_X_MAX - ROBOT_WIDTH/2));
                particle.y = std::max(FIELD_Y_MIN + ROBOT_LENGTH/2, std::min(particle.y, FIELD_Y_MAX - ROBOT_LENGTH/2));
                
                // Validate initialized particle
                if (!std::isfinite(particle.x) || !std::isfinite(particle.y) || 
                    !std::isfinite(particle.theta) || !std::isfinite(particle.weight)) {
                    std::printf("Warning: Invalid particle initialized, resetting to odometry pose\n");
                    particle.x = odom_pose.x;
                    particle.y = odom_pose.y;
                    particle.theta = imu_heading_rad; // Use IMU heading directly
                    particle.weight = 1.0 / NUM_PARTICLES;
                }
            }
        } else {
            std::printf("MCL: Odometry pose invalid, using uniform field distribution\n");
            
            // Fall back to uniform distribution across entire field
            // Use current IMU heading (or default 0) for all particles
            std::uniform_real_distribution<double> x_dist(FIELD_X_MIN + ROBOT_WIDTH/2, FIELD_X_MAX - ROBOT_WIDTH/2);
            std::uniform_real_distribution<double> y_dist(FIELD_Y_MIN + ROBOT_LENGTH/2, FIELD_Y_MAX - ROBOT_LENGTH/2);
            double default_heading = 0.0; // Default to facing north if no IMU data
            
            for (auto& particle : particles) {
                particle.x = x_dist(gen);
                particle.y = y_dist(gen);
                particle.theta = default_heading; // All particles use same heading
                particle.weight = 1.0 / NUM_PARTICLES;
                
                // Validate initialized particle
                if (!std::isfinite(particle.x) || !std::isfinite(particle.y) || 
                    !std::isfinite(particle.theta) || !std::isfinite(particle.weight)) {
                    std::printf("Warning: Invalid particle initialized, resetting to default\n");
                    particle.x = 0.0;
                    particle.y = 0.0;
                    particle.theta = default_heading;
                    particle.weight = 1.0 / NUM_PARTICLES;
                }
            }
        }
    }
    
    // Prediction step: move particles based on odometry
    void predict(const lemlib::Pose& current_pose) {
        // Validate input pose
        if (!std::isfinite(current_pose.x) || !std::isfinite(current_pose.y) || !std::isfinite(current_pose.theta)) {
            std::printf("Warning: Invalid odometry pose received, skipping prediction step\n");
            return;
        }
        
        if (!initialized) {
            last_pose = current_pose;
            initialized = true;
            return;
        }
        
        // Calculate motion delta (position only - ignore theta since using IMU directly)
        double dx = current_pose.x - last_pose.x;
        double dy = current_pose.y - last_pose.y;
        
        // Validate motion deltas
        if (!std::isfinite(dx) || !std::isfinite(dy)) {
            std::printf("Warning: Invalid motion delta calculated, skipping prediction\n");
            return;
        }
        
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
            particle.x = std::max(FIELD_X_MIN + ROBOT_WIDTH/2, std::min(particle.x, FIELD_X_MAX - ROBOT_WIDTH/2));
            particle.y = std::max(FIELD_Y_MIN + ROBOT_LENGTH/2, std::min(particle.y, FIELD_Y_MAX - ROBOT_LENGTH/2));
        }
        
        last_pose = current_pose;
    }
    
    // Helper function to check ray intersection with a rectangular obstacle
    // Returns the distance to intersection, or infinity if no intersection
    double rayIntersectRectangle(double sensor_x, double sensor_y, double ray_dx, double ray_dy,
                                double rect_center_x, double rect_center_y, 
                                double rect_width, double rect_height) {
        const double EPSILON = 1e-10;
        
        // Rectangle bounds
        double rect_left = rect_center_x - rect_width/2;
        double rect_right = rect_center_x + rect_width/2;
        double rect_bottom = rect_center_y - rect_height/2;
        double rect_top = rect_center_y + rect_height/2;
        
        double min_distance = std::numeric_limits<double>::max();
        
        // Check intersection with each edge of the rectangle
        // Left edge (X = rect_left)
        if (std::abs(ray_dx) > EPSILON) {
            double t = (rect_left - sensor_x) / ray_dx;
            if (t > 0.0) {  // Intersection in front of sensor
                double intersect_y = sensor_y + t * ray_dy;
                if (intersect_y >= rect_bottom && intersect_y <= rect_top) {
                    min_distance = std::min(min_distance, t);
                }
            }
        }
        
        // Right edge (X = rect_right)
        if (std::abs(ray_dx) > EPSILON) {
            double t = (rect_right - sensor_x) / ray_dx;
            if (t > 0.0) {  // Intersection in front of sensor
                double intersect_y = sensor_y + t * ray_dy;
                if (intersect_y >= rect_bottom && intersect_y <= rect_top) {
                    min_distance = std::min(min_distance, t);
                }
            }
        }
        
        // Bottom edge (Y = rect_bottom)
        if (std::abs(ray_dy) > EPSILON) {
            double t = (rect_bottom - sensor_y) / ray_dy;
            if (t > 0.0) {  // Intersection in front of sensor
                double intersect_x = sensor_x + t * ray_dx;
                if (intersect_x >= rect_left && intersect_x <= rect_right) {
                    min_distance = std::min(min_distance, t);
                }
            }
        }
        
        // Top edge (Y = rect_top)
        if (std::abs(ray_dy) > EPSILON) {
            double t = (rect_top - sensor_y) / ray_dy;
            if (t > 0.0) {  // Intersection in front of sensor
                double intersect_x = sensor_x + t * ray_dx;
                if (intersect_x >= rect_left && intersect_x <= rect_right) {
                    min_distance = std::min(min_distance, t);
                }
            }
        }
        
        return min_distance;
    }
    
    // Get expected distance from particle to nearest obstacle using raycast in local sensor direction
    // VRC Coordinate System: X+ = Right, Y+ = North, 0° = North, Clockwise positive
    // Rotation math verification:
    // - At 0°: robot faces North (Y+), front sensor ray = (0,1), right sensor ray = (1,0)
    // - At 90°: robot faces East (X+), front sensor ray = (1,0), right sensor ray = (0,-1)
    // 
    // Obstacles detected: Field perimeter walls, Center Goals (stacked at origin), Loaders
    // Note: Long Goals skipped (too high), Park Zones removed (per request)
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
        // Ray equation: P(t) = sensor_position + t * ray_direction
        // Wall intersection: solve for t where ray intersects each wall boundary
        // Choose the minimum positive t (closest wall in sensor direction)
        double min_distance = std::numeric_limits<double>::max();
        
        // Check intersection with each wall (only if ray is pointing toward the wall)
        // Add small epsilon to avoid division by zero
        const double EPSILON = 1e-10;
        
        // North wall (Y = FIELD_Y_MAX)
        if (ray_dy > EPSILON) {  // Ray pointing north (avoid division by zero)
            double t = (FIELD_Y_MAX - sensor_y) / ray_dy;
            if (t > 0.0) {  // Intersection is in front of sensor
                double intersect_x = sensor_x + t * ray_dx;
                if (intersect_x >= FIELD_X_MIN && intersect_x <= FIELD_X_MAX) {
                    min_distance = std::min(min_distance, t);
                }
            }
        }
        
        // South wall (Y = FIELD_Y_MIN)
        if (ray_dy < -EPSILON) {  // Ray pointing south (avoid division by zero)
            double t = (FIELD_Y_MIN - sensor_y) / ray_dy;
            if (t > 0.0) {  // Intersection is in front of sensor
                double intersect_x = sensor_x + t * ray_dx;
                if (intersect_x >= FIELD_X_MIN && intersect_x <= FIELD_X_MAX) {
                    min_distance = std::min(min_distance, t);
                }
            }
        }
        
        // East wall (X = FIELD_X_MAX)
        if (ray_dx > EPSILON) {  // Ray pointing east (avoid division by zero)
            double t = (FIELD_X_MAX - sensor_x) / ray_dx;
            if (t > 0.0) {  // Intersection is in front of sensor
                double intersect_y = sensor_y + t * ray_dy;
                if (intersect_y >= FIELD_Y_MIN && intersect_y <= FIELD_Y_MAX) {
                    min_distance = std::min(min_distance, t);
                }
            }
        }
        
        // West wall (X = FIELD_X_MIN)
        if (ray_dx < -EPSILON) {  // Ray pointing west (avoid division by zero)
            double t = (FIELD_X_MIN - sensor_x) / ray_dx;
            if (t > 0.0) {  // Intersection is in front of sensor
                double intersect_y = sensor_y + t * ray_dy;
                if (intersect_y >= FIELD_Y_MIN && intersect_y <= FIELD_Y_MAX) {
                    min_distance = std::min(min_distance, t);
                }
            }
        }
        
        // Check intersections with field obstacles (Center Goals, Loaders)
        
        // Center Goals - Treated as single stacked object at field center
        double center_goal_dist = rayIntersectRectangle(sensor_x, sensor_y, ray_dx, ray_dy,
                                                       CENTER_GOAL_X, CENTER_GOAL_Y,
                                                       CENTER_GOAL_WIDTH, CENTER_GOAL_LENGTH);
        if (center_goal_dist < min_distance) {
            min_distance = center_goal_dist;
        }
        
        // Loaders - Red Alliance
        double red_loader_1_dist = rayIntersectRectangle(sensor_x, sensor_y, ray_dx, ray_dy,
                                                        RED_LOADER_1_X, RED_LOADER_1_Y,
                                                        LOADER_WIDTH, LOADER_DEPTH);
        if (red_loader_1_dist < min_distance) {
            min_distance = red_loader_1_dist;
        }
        
        double red_loader_2_dist = rayIntersectRectangle(sensor_x, sensor_y, ray_dx, ray_dy,
                                                        RED_LOADER_2_X, RED_LOADER_2_Y,
                                                        LOADER_WIDTH, LOADER_DEPTH);
        if (red_loader_2_dist < min_distance) {
            min_distance = red_loader_2_dist;
        }
        
        // Loaders - Blue Alliance
        double blue_loader_1_dist = rayIntersectRectangle(sensor_x, sensor_y, ray_dx, ray_dy,
                                                         BLUE_LOADER_1_X, BLUE_LOADER_1_Y,
                                                         LOADER_WIDTH, LOADER_DEPTH);
        if (blue_loader_1_dist < min_distance) {
            min_distance = blue_loader_1_dist;
        }
        
        double blue_loader_2_dist = rayIntersectRectangle(sensor_x, sensor_y, ray_dx, ray_dy,
                                                         BLUE_LOADER_2_X, BLUE_LOADER_2_Y,
                                                         LOADER_WIDTH, LOADER_DEPTH);
        if (blue_loader_2_dist < min_distance) {
            min_distance = blue_loader_2_dist;
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
    
    // Update step: weight particles based on sensor measurements
    void update() {
        // Get sensor readings (convert from mm to inches)
        double front_reading = front_dist.get() / 25.4;
        double back_reading = back_dist.get() / 25.4;
        double left_reading = left_dist.get() / 25.4;
        double right_reading = right_dist.get() / 25.4;
        
        // Validate sensor readings for infinity/NaN
        if (!std::isfinite(front_reading)) front_reading = -1;
        if (!std::isfinite(back_reading)) back_reading = -1;
        if (!std::isfinite(left_reading)) left_reading = -1;
        if (!std::isfinite(right_reading)) right_reading = -1;
        
        // Check sensor reading categories
        std::vector<double> readings = {front_reading, back_reading, left_reading, right_reading};
        std::vector<bool> sensor_valid = {
            front_reading > 0 && front_reading < 42,  // Valid close-range readings
            back_reading > 0 && back_reading < 42,
            left_reading > 0 && left_reading < 42,
            right_reading > 0 && right_reading < 42
        };
        std::vector<bool> sensor_open_space = {
            front_reading >= 42 && std::isfinite(front_reading),  // High readings indicating open space
            back_reading >= 42 && std::isfinite(back_reading),
            left_reading >= 42 && std::isfinite(left_reading),
            right_reading >= 42 && std::isfinite(right_reading)
        };
        
        // Count sensors with usable information (either valid close readings or open space readings)
        int usable_sensor_count = 0;
        for (int i = 0; i < 4; i++) {
            if (sensor_valid[i] || sensor_open_space[i]) usable_sensor_count++;
        }
        
        // Skip update only if NO sensors provide usable information
        if (usable_sensor_count == 0) {
            return;
        }
        
        double total_weight = 0.0;
        
        // Update particle weights based on multiplicative sensor likelihood
        for (auto& particle : particles) {
            double weight = 1.0; // Start with neutral weight for multiplication
            
            // Calculate likelihood for each sensor (multiplicative approach)
            for (int i = 0; i < 4; i++) {
                if (sensor_valid[i]) {
                    // Use actual sensor reading for valid close-range sensors
                    double expected = getExpectedDistance(particle, i);
                    double error = readings[i] - expected;
                    
                    // Gaussian likelihood function using base sensor noise
                    double likelihood = exp(-0.5 * (error * error) / (BASE_SENSOR_NOISE_STD * BASE_SENSOR_NOISE_STD));
                    weight *= likelihood; // Multiplicative weighting - all sensors must agree
                } else if (sensor_open_space[i]) {
                    // High readings indicate open space - give high likelihood if particle expects open space
                    double expected = getExpectedDistance(particle, i);
                    
                    if (expected >= 42.0) {
                        // Particle expects open space and sensor sees open space - high likelihood
                        weight *= 0.95; // High confidence match
                    } else {
                        // Particle expects obstacle but sensor sees open space - low likelihood
                        weight *= 0.1; // Strong disagreement penalty
                    }
                } else {
                    // Invalid sensors (≤0 or non-finite), apply neutral weight (no information)
                    weight *= 1.0; // Neutral - no information from this sensor
                }
            }
            
            particle.weight = weight;
            total_weight += weight;
        }
        
        // Normalize weights
        if (total_weight > 1e-10) {  // Use epsilon to avoid division by very small numbers
            for (auto& particle : particles) {
                particle.weight /= total_weight;
            }
        } else {
            // If total weight is too small or zero, reset all weights to uniform
            std::printf("Warning: MCL total weight too small (%.2e), resetting to uniform distribution\n", total_weight);
            for (auto& particle : particles) {
                particle.weight = 1.0 / NUM_PARTICLES;
            }
        }
    }
    
    // Resample particles based on weights (hybrid: 80% systematic resampling, 20% random)
    void resample() {
        extern lemlib::Chassis chassis; // Reference to chassis from drive.cpp
        
        std::vector<Particle> new_particles;
        new_particles.reserve(NUM_PARTICLES);
        
        // Calculate how many particles for each method
        int resampled_count = (NUM_PARTICLES)* 0.9;      // 80% from systematic resampling
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
        lemlib::Pose estimated_pose = getEstimatedPose();
        bool use_pose_guidance = std::isfinite(estimated_pose.x) && std::isfinite(estimated_pose.y) && std::isfinite(estimated_pose.theta);
        
        if (use_pose_guidance) {
            // Generate random particles around current estimated pose (with wider spread than initialization)
            // Use IMU heading directly - no angular uncertainty
            std::normal_distribution<double> x_dist(estimated_pose.x, 20.0);  // 20 inch standard deviation
            std::normal_distribution<double> y_dist(estimated_pose.y, 20.0);  // 20 inch standard deviation
            double current_imu_heading = estimated_pose.theta * M_PI / 180.0; // Trust IMU heading
            
            for (int m = 0; m < random_count; m++) {
                Particle random_particle;
                random_particle.x = x_dist(gen);
                random_particle.y = y_dist(gen);
                random_particle.theta = current_imu_heading; // All particles use same IMU heading
                random_particle.weight = 1.0 / NUM_PARTICLES;
                
                // Keep particles within field bounds
                random_particle.x = std::max(FIELD_X_MIN + ROBOT_WIDTH/2, std::min(random_particle.x, FIELD_X_MAX - ROBOT_WIDTH/2));
                random_particle.y = std::max(FIELD_Y_MIN + ROBOT_LENGTH/2, std::min(random_particle.y, FIELD_Y_MAX - ROBOT_LENGTH/2));
                
                // Validate particle before adding
                if (std::isfinite(random_particle.x) && std::isfinite(random_particle.y) && 
                    std::isfinite(random_particle.theta) && std::isfinite(random_particle.weight)) {
                    new_particles.push_back(random_particle);
                } else {
                    // Fallback to odometry-centered distribution if guided generation fails
                    lemlib::Pose odom_pose = chassis.getPose();
                    if (std::isfinite(odom_pose.x) && std::isfinite(odom_pose.y) && std::isfinite(odom_pose.theta)) {
                        std::normal_distribution<double> x_odom(odom_pose.x, 15.0);  // 15 inch std dev around odom
                        std::normal_distribution<double> y_odom(odom_pose.y, 15.0);
                        std::normal_distribution<double> theta_odom(odom_pose.theta * M_PI / 180.0, M_PI/4); // ±45° std dev
                        
                        random_particle.x = x_odom(gen);
                        random_particle.y = y_odom(gen);
                        random_particle.theta = theta_odom(gen);
                        
                        // Keep within bounds
                        random_particle.x = std::max(FIELD_X_MIN + ROBOT_WIDTH/2, std::min(random_particle.x, FIELD_X_MAX - ROBOT_WIDTH/2));
                        random_particle.y = std::max(FIELD_Y_MIN + ROBOT_LENGTH/2, std::min(random_particle.y, FIELD_Y_MAX - ROBOT_LENGTH/2));
                        while (random_particle.theta > M_PI) random_particle.theta -= 2 * M_PI;
                        while (random_particle.theta < -M_PI) random_particle.theta += 2 * M_PI;
                    } else {
                        // Ultimate fallback: uniform distribution if both MCL and odom fail
                        std::uniform_real_distribution<double> x_uniform(FIELD_X_MIN + ROBOT_WIDTH/2, FIELD_X_MAX - ROBOT_WIDTH/2);
                        std::uniform_real_distribution<double> y_uniform(FIELD_Y_MIN + ROBOT_LENGTH/2, FIELD_Y_MAX - ROBOT_LENGTH/2);
                        std::uniform_real_distribution<double> theta_uniform(-M_PI, M_PI);
                        
                        random_particle.x = x_uniform(gen);
                        random_particle.y = y_uniform(gen);
                        random_particle.theta = theta_uniform(gen);
                    }
                    random_particle.weight = 1.0 / NUM_PARTICLES;
                    new_particles.push_back(random_particle);
                }
            }
        } else {
            // Fallback: odometry-centered random distribution when MCL pose estimate is invalid
            lemlib::Pose odom_pose = chassis.getPose();
            if (std::isfinite(odom_pose.x) && std::isfinite(odom_pose.y) && std::isfinite(odom_pose.theta)) {
                // Generate particles around odometry pose
                // Use IMU heading directly - no angular uncertainty
                std::normal_distribution<double> x_dist(odom_pose.x, 15.0);  // 15 inch standard deviation
                std::normal_distribution<double> y_dist(odom_pose.y, 15.0);  // 15 inch standard deviation
                double current_imu_heading = odom_pose.theta * M_PI / 180.0; // Trust IMU heading
                
                for (int m = 0; m < random_count; m++) {
                    Particle random_particle;
                    random_particle.x = x_dist(gen);
                    random_particle.y = y_dist(gen);
                    random_particle.theta = current_imu_heading; // All particles use same IMU heading
                    random_particle.weight = 1.0 / NUM_PARTICLES;
                    
                    // Keep particles within field bounds
                    random_particle.x = std::max(FIELD_X_MIN + ROBOT_WIDTH/2, std::min(random_particle.x, FIELD_X_MAX - ROBOT_WIDTH/2));
                    random_particle.y = std::max(FIELD_Y_MIN + ROBOT_LENGTH/2, std::min(random_particle.y, FIELD_Y_MAX - ROBOT_LENGTH/2));
                    
                    new_particles.push_back(random_particle);
                }
            } else {
                // Ultimate fallback: uniform random distribution if odometry is also invalid
                // Use default heading (north) for all particles
                std::uniform_real_distribution<double> x_dist(FIELD_X_MIN + ROBOT_WIDTH/2, FIELD_X_MAX - ROBOT_WIDTH/2);
                std::uniform_real_distribution<double> y_dist(FIELD_Y_MIN + ROBOT_LENGTH/2, FIELD_Y_MAX - ROBOT_LENGTH/2);
                double default_heading = 0.0; // Default to facing north
                
                for (int m = 0; m < random_count; m++) {
                    Particle random_particle;
                    random_particle.x = x_dist(gen);
                    random_particle.y = y_dist(gen);
                    random_particle.theta = default_heading; // All particles use same default heading
                    random_particle.weight = 1.0 / NUM_PARTICLES;
                    new_particles.push_back(random_particle);
                }
            }
        }
        
        particles = std::move(new_particles);
    }
    
    // Get estimated pose (weighted average of particles for position, IMU for heading)
    lemlib::Pose getEstimatedPose() {
        extern lemlib::Chassis chassis; // Reference to chassis from drive.cpp
        
        double x_sum = 0.0, y_sum = 0.0;
        double weight_sum = 0.0;
        
        for (const auto& particle : particles) {
            // Validate particle values before using them
            if (!std::isfinite(particle.x) || !std::isfinite(particle.y) || 
                !std::isfinite(particle.theta) || !std::isfinite(particle.weight)) {
                std::printf("Warning: Invalid particle detected: x=%.3f, y=%.3f, theta=%.3f, weight=%.3f\n",
                       particle.x, particle.y, particle.theta, particle.weight);
                continue; // Skip this particle
            }
            
            x_sum += particle.x * particle.weight;
            y_sum += particle.y * particle.weight;
            weight_sum += particle.weight;
        }
        
        // Get current odometry pose as fallback
        lemlib::Pose odom_pose = chassis.getPose();
        
        // Validate odometry pose and provide ultimate fallback
        if (!std::isfinite(odom_pose.x) || !std::isfinite(odom_pose.y) || !std::isfinite(odom_pose.theta)) {
            std::printf("Warning: Odometry pose is also invalid, using last known pose\n");
            odom_pose = last_pose; // Use last known good pose
            
            // If even last_pose is invalid, use origin
            if (!std::isfinite(odom_pose.x) || !std::isfinite(odom_pose.y) || !std::isfinite(odom_pose.theta)) {
                odom_pose = lemlib::Pose(0.0, 0.0, 0.0);
            }
        }
        
        // Validate results
        if (weight_sum < 1e-10) {
            std::printf("Warning: MCL weight sum too small, returning odometry pose (%.2f, %.2f, %.2f)\n", 
                   odom_pose.x, odom_pose.y, odom_pose.theta);
            return odom_pose;
        }
        
        // Use IMU heading directly (already in degrees from odometry)
        double estimated_theta = odom_pose.theta; 
        
        // Validate final result
        if (!std::isfinite(x_sum) || !std::isfinite(y_sum) || !std::isfinite(estimated_theta)) {
            std::printf("Warning: MCL pose calculation resulted in non-finite values, returning odometry pose (%.2f, %.2f, %.2f)\n",
                   odom_pose.x, odom_pose.y, odom_pose.theta);
            return odom_pose;
        }
        
        return lemlib::Pose(x_sum, y_sum, estimated_theta); // Use IMU heading directly
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
    
    if ((force_update || (mcl_controller.isConverged() && time_to_update))) { // Only if uncertainty is low
        
        lemlib::Pose mcl_pose = mcl_controller.getEstimatedPose();
        lemlib::Pose current_pose = chassis.getPose();
        
        // Calculate the correction needed
        lemlib::Pose error = mcl_controller.calculateGlobalError(current_pose);
        
        // Only apply correction if error is significant but not too large
        double position_error = sqrt(error.x * error.x + error.y * error.y);
        if (position_error > 2.0 && position_error < 24.0) { // Between 2-24 inches
            chassis.setPose(mcl_pose);
            last_update_time = current_time;
            
            std::printf("MCL Correction Applied: Pos Error=%.2f, Angle Error=%.2f\n", 
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
        updateChassisPoseFromMCL(false);
        
        pros::delay(50); // Update at 20Hz
    }
}

// Initialize MCL system
void initializeMCL() {
    pros::Task mcl_task(mclTask, (void*)"MCL");
}
