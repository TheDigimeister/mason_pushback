#include "main.h"
#include <limits>
#include <random>

// MCL (Monte Carlo Localization) Implementation - FULL 6DOF POSE ESTIMATION
// Uses distance sensors on all four sides of the robot for pose estimation
// 
// FEATURES: 
// 1. Full state estimation (x, y, theta) with angular uncertainty
//    - Particles maintain independent heading estimates with angular noise
//    - MCL estimates complete robot pose including orientation
//    - Includes motion model for both translational and rotational movement
// 2. Uses CONFIDENCE-WEIGHTED SUM sensor fusion with LOG-LIKELIHOOD
//    - Uses sensor confidence (0-63) from get_confidence() to weight contributions
//    - Adaptive noise model: lower confidence = higher effective noise
//    - No hard sensor range limits - uses raw raycast distances  
//    - Robust to individual sensor failures/noise
//    - Converts back to linear space for final particle weights
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
const int NUM_PARTICLES = 50;  // Reduced from 350 to prevent memory overflow
const double FIELD_WIDTH = 144.0;  // VRC field width in inches
const double FIELD_LENGTH = 144.0; // VRC field length in inches
const double BASE_SENSOR_NOISE_STD = 0.5; // Base standard deviation for sensor noise
const double MOTION_NOISE_STD = 4.0; // Standard deviation for motion noise
const double ANGLE_NOISE_STD = 2.0; // Standard deviation for angular noise (degrees)

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
    std::normal_distribution<double> angle_noise;
    
    lemlib::Pose last_pose;
    bool initialized;
    
    // Balanced distance cache with 256KB budget for practical performance
    struct AggressiveDistanceCache {
        static const int GRID_SIZE = 32;          // 4.5" spatial resolution (144"/4.5 = 32)
        static const int ANGLE_INTERVALS = 8;     // 45° angular resolution (360°/45° = 8)
        static const double ANGLE_STEP;           // = 45.0 degrees
        static const double CELL_SIZE;            // = 4.5 inches
        
        // [grid_x][grid_y][angle_index][sensor_id] = distance
        // Memory: 32 × 32 × 8 × 4 × 8 bytes = 262,144 bytes = 256 KB
        double distances[GRID_SIZE][GRID_SIZE][ANGLE_INTERVALS][4];
        bool initialized = false;
        
        // Memory monitoring
        struct MemoryMonitor {
            static void reportUsage() {
                size_t cache_size = sizeof(distances);
                printf("MCL: Balanced cache allocated: %.0f KB\n", cache_size / 1024.0);
                printf("MCL: Grid resolution: %d×%d cells (%.1f\" spacing)\n", 
                       GRID_SIZE, GRID_SIZE, CELL_SIZE);
                printf("MCL: Angular resolution: %d intervals (%.1f° spacing)\n", 
                       ANGLE_INTERVALS, ANGLE_STEP);
                
                // Test memory availability  
                void* test_alloc = malloc(512 * 1024); // Try 512KB allocation
                if (test_alloc) {
                    free(test_alloc);
                    printf("MCL: Memory check: OK (>512KB available)\n");
                } else {
                    printf("MCL: WARNING - Memory may be constrained!\n");
                }
            }
        };
        
        // Convert world coordinates to grid indices with bounds checking
        std::pair<int, int> worldToGrid(double x, double y) {
            int grid_x = std::max(0, std::min(GRID_SIZE-1, 
                (int)((x - FIELD_X_MIN) / CELL_SIZE)));
            int grid_y = std::max(0, std::min(GRID_SIZE-1, 
                (int)((y - FIELD_Y_MIN) / CELL_SIZE)));
            return {grid_x, grid_y};
        }
        
        // Get grid position with fractional parts for bilinear interpolation
        struct GridPosition {
            int gx0, gy0;      // Lower grid indices
            int gx1, gy1;      // Upper grid indices  
            double fx, fy;     // Interpolation factors [0, 1]
        };
        
        GridPosition worldToGridInterp(double x, double y) {
            // Calculate continuous grid position
            double grid_x_f = (x - FIELD_X_MIN) / CELL_SIZE;
            double grid_y_f = (y - FIELD_Y_MIN) / CELL_SIZE;
            
            // Get integer grid indices and clamp to valid range for interpolation
            int gx0 = std::max(0, std::min(GRID_SIZE-2, (int)grid_x_f));
            int gy0 = std::max(0, std::min(GRID_SIZE-2, (int)grid_y_f));
            int gx1 = gx0 + 1;
            int gy1 = gy0 + 1;
            
            // Calculate interpolation factors [0, 1]
            double fx = std::max(0.0, std::min(1.0, grid_x_f - gx0));
            double fy = std::max(0.0, std::min(1.0, grid_y_f - gy0));
            
            return {gx0, gy0, gx1, gy1, fx, fy};
        }
        
        // Convert heading to angle index and interpolation data
        struct AngleInterp {
            int lower_idx;      // Lower cached angle index
            int upper_idx;      // Upper cached angle index  
            double factor;      // Interpolation factor [0, 1]
        };
        
        AngleInterp headingToInterp(double heading_degrees) {
            // Normalize to [0, 360)
            while (heading_degrees < 0) heading_degrees += 360.0;
            while (heading_degrees >= 360.0) heading_degrees -= 360.0;
            
            // Find position within 15° intervals
            double angle_position = heading_degrees / ANGLE_STEP;
            int lower_idx = (int)angle_position % ANGLE_INTERVALS;
            int upper_idx = (lower_idx + 1) % ANGLE_INTERVALS;
            double factor = angle_position - (int)angle_position;
            
            return {lower_idx, upper_idx, factor};
        }
        
        // Static raycast helper for cache pre-computation
        static double rayIntersectWall(double sensor_x, double sensor_y, double ray_dx, double ray_dy,
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
        
        void precompute() {
            if (initialized) return;
            
            printf("MCL: Building balanced distance cache...\n");
            MemoryMonitor::reportUsage();
            
            uint32_t start_time = pros::millis();
            
            int total_ops = GRID_SIZE * GRID_SIZE * ANGLE_INTERVALS * 4;
            int completed = 0;
            int last_progress = -1;
            
            for (int gx = 0; gx < GRID_SIZE; gx++) {
                for (int gy = 0; gy < GRID_SIZE; gy++) {
                    // Convert grid to world coordinates (cell center)
                    double world_x = FIELD_X_MIN + (gx + 0.5) * CELL_SIZE;
                    double world_y = FIELD_Y_MIN + (gy + 0.5) * CELL_SIZE;
                    
                    // Pre-compute for each 15° angle interval
                    for (int angle_idx = 0; angle_idx < ANGLE_INTERVALS; angle_idx++) {
                        double heading_deg = angle_idx * ANGLE_STEP;
                        double heading_rad = heading_deg * M_PI / 180.0;
                        
                        // Compute distances for all 4 sensors at this position/heading
                        for (int sensor_id = 0; sensor_id < 4; sensor_id++) {
                            distances[gx][gy][angle_idx][sensor_id] = 
                                computeDistanceAtHeading(world_x, world_y, heading_rad, sensor_id);
                            completed++;
                        }
                    }
                    
                    // Progress reporting every 5%
                    int progress = (completed * 100) / total_ops;
                    if (progress != last_progress && progress % 5 == 0) {
                        printf("MCL Cache: %d%% complete (%d/%d ops)\n", progress, completed, total_ops);
                        last_progress = progress;
                        pros::delay(1); // Brief yield to prevent watchdog timeout
                    }
                }
            }
            
            initialized = true;
            uint32_t elapsed = pros::millis() - start_time;
            printf("MCL: Balanced cache ready in %dms (%.0f KB)\n", 
                   elapsed, sizeof(distances) / 1024.0);
        }
        
        double computeDistanceAtHeading(double x, double y, double heading_rad, int sensor_id) {
            double sensor_x, sensor_y;
            double ray_dx, ray_dy;
            
            // Calculate sensor position and ray direction (same logic as original getExpectedDistance)
            switch (sensor_id) {
                case 0: // Front sensor
                    sensor_x = x + FRONT_SENSOR_OFFSET * sin(heading_rad);
                    sensor_y = y + FRONT_SENSOR_OFFSET * cos(heading_rad);
                    ray_dx = sin(heading_rad);
                    ray_dy = cos(heading_rad);
                    break;
                    
                case 1: // Back sensor
                    sensor_x = x - BACK_SENSOR_OFFSET * sin(heading_rad);
                    sensor_y = y - BACK_SENSOR_OFFSET * cos(heading_rad);
                    ray_dx = -sin(heading_rad);
                    ray_dy = -cos(heading_rad);
                    break;
                    
                case 2: // Left sensor
                    sensor_x = x - LEFT_SENSOR_OFFSET * cos(heading_rad);
                    sensor_y = y + LEFT_SENSOR_OFFSET * sin(heading_rad);
                    ray_dx = -cos(heading_rad);
                    ray_dy = sin(heading_rad);
                    break;
                    
                case 3: // Right sensor
                    sensor_x = x + RIGHT_SENSOR_OFFSET * cos(heading_rad);
                    sensor_y = y - RIGHT_SENSOR_OFFSET * sin(heading_rad);
                    ray_dx = cos(heading_rad);
                    ray_dy = -sin(heading_rad);
                    break;
                    
                default:
                    return 50.0;
            }
            
            // Raycast to walls (same wall intersection logic as original)
            double min_distance = std::numeric_limits<double>::max();
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
            
            return (min_distance == std::numeric_limits<double>::max()) ? 50.0 : std::max(0.1, min_distance);
        }
        
        // Maximum accuracy lookup with bilinear spatial + angular interpolation
        double lookup(double x, double y, double heading_degrees, int sensor_id) {
            if (!initialized) precompute();
            
            // Get spatial interpolation data (3" grid with bilinear interpolation)
            GridPosition pos = worldToGridInterp(x, y);
            
            // Get angular interpolation data (15° intervals with linear interpolation)
            AngleInterp angle = headingToInterp(heading_degrees);
            
            // Bilinear spatial interpolation at both cached angles
            auto spatialInterp = [&](int angle_idx) -> double {
                // Get the 4 corner values for bilinear interpolation
                double d00 = distances[pos.gx0][pos.gy0][angle_idx][sensor_id]; // Bottom-left
                double d10 = distances[pos.gx1][pos.gy0][angle_idx][sensor_id]; // Bottom-right
                double d01 = distances[pos.gx0][pos.gy1][angle_idx][sensor_id]; // Top-left
                double d11 = distances[pos.gx1][pos.gy1][angle_idx][sensor_id]; // Top-right
                
                // Bilinear interpolation formula
                double d_bottom = d00 + pos.fx * (d10 - d00); // Bottom edge interpolation
                double d_top = d01 + pos.fx * (d11 - d01);    // Top edge interpolation
                return d_bottom + pos.fy * (d_top - d_bottom); // Final Y interpolation
            };
            
            // Get spatially interpolated distances at both cached angles
            double lower_dist = spatialInterp(angle.lower_idx);
            double upper_dist = spatialInterp(angle.upper_idx);
            
            // Final angular interpolation between spatially interpolated values
            return lower_dist + angle.factor * (upper_dist - lower_dist);
        }
        
        // Fast lookup with nearest neighbor spatial + angular interpolation
        double lookupFast(double x, double y, double heading_degrees, int sensor_id) {
            if (!initialized) precompute();
            
            // Spatial lookup using nearest neighbor (faster)
            auto [gx, gy] = worldToGrid(x, y);
            
            // Angular interpolation between cached 15° intervals
            AngleInterp angle = headingToInterp(heading_degrees);
            
            // Get distances at lower and upper cached angles
            double lower_dist = distances[gx][gy][angle.lower_idx][sensor_id];
            double upper_dist = distances[gx][gy][angle.upper_idx][sensor_id];
            
            // Linear interpolation between cached angles
            return lower_dist + angle.factor * (upper_dist - lower_dist);
        }
    };
    
    AggressiveDistanceCache distance_cache;

public:
    // Helper function to keep particles within field bounds
    void clampParticleToField(Particle& particle) {
        particle.x = std::max(FIELD_X_MIN + ROBOT_WIDTH/2, std::min(particle.x, FIELD_X_MAX - ROBOT_WIDTH/2));
        particle.y = std::max(FIELD_Y_MIN + ROBOT_LENGTH/2, std::min(particle.y, FIELD_Y_MAX - ROBOT_LENGTH/2));
    }
    
    MCLController() : gen(rd()), 
                      motion_noise(0.0, MOTION_NOISE_STD),
                      sensor_noise(0.0, BASE_SENSOR_NOISE_STD),
                      angle_noise(0.0, ANGLE_NOISE_STD * M_PI / 180.0), // Convert degrees to radians
                      initialized(false),
                      last_pose(chassis.getPose()) {
        particles.resize(NUM_PARTICLES);
        
        // TEMPORARILY DISABLE CACHE TO PREVENT MEMORY OVERFLOW
        // Pre-compute aggressive distance cache during initialization
        // This runs once at startup (~1-2 seconds), then provides O(1) interpolated lookups
        printf("MCL: Cache disabled for memory safety - using direct raycast\n");
        // distance_cache.precompute();
        
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
        // Include angular uncertainty around IMU heading
        std::normal_distribution<double> x_dist(odom_pose.x, 12.0);  // 12 inch standard deviation
        std::normal_distribution<double> y_dist(odom_pose.y, 12.0);  // 12 inch standard deviation
        double imu_heading_rad = odom_pose.theta * M_PI / 180.0; // Convert to radians
        std::normal_distribution<double> theta_dist(imu_heading_rad, ANGLE_NOISE_STD * M_PI / 180.0); // Angular uncertainty
        
        for (auto& particle : particles) {
            particle.x = x_dist(gen);
            particle.y = y_dist(gen);
            particle.theta = theta_dist(gen); // Add angular uncertainty around IMU heading
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
        
        // Calculate motion delta (position and orientation)
        double dx = current_pose.x - last_pose.x;
        double dy = current_pose.y - last_pose.y;
        double dtheta = (current_pose.theta - last_pose.theta) * M_PI / 180.0; // Convert to radians
        
        // Normalize angular delta to [-π, π]
        while (dtheta > M_PI) dtheta -= 2.0 * M_PI;
        while (dtheta < -M_PI) dtheta += 2.0 * M_PI;
        
        // Move each particle
        for (auto& particle : particles) {
            // Add motion noise to position and orientation
            double noisy_dx = dx + motion_noise(gen);
            double noisy_dy = dy + motion_noise(gen);
            double noisy_dtheta = dtheta + angle_noise(gen);
            
            // Update particle state
            particle.x += noisy_dx;
            particle.y += noisy_dy;
            particle.theta += noisy_dtheta;
            
            // Normalize particle theta to [-π, π]
            while (particle.theta > M_PI) particle.theta -= 2.0 * M_PI;
            while (particle.theta < -M_PI) particle.theta += 2.0 * M_PI;
            
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
    // MEMORY-SAFE: Using direct raycast calculation instead of cache to prevent overflow
    double getExpectedDistance(const Particle& particle, int sensor_id) {
        // Use direct raycast calculation to prevent memory overflow
        return getExpectedDistanceExact(particle, sensor_id);
        
        // Original cache code (disabled for memory safety):
        // double heading_degrees = particle.theta * 180.0 / M_PI;
        // return distance_cache.lookup(particle.x, particle.y, heading_degrees, sensor_id);
    }
    
    // Original raycast implementation (kept for debugging/validation)
    double getExpectedDistanceExact(const Particle& particle, int sensor_id) {
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
        
        // No clamping - return raw raycast distance
        return std::max(0.1, min_distance); // Only prevent negative/zero distances
    }
    


    
    // Helper function to process and validate a single sensor reading
    struct SensorReading {
        double value;
        int32_t confidence;
        bool is_valid;
        
        SensorReading(double raw_reading, int32_t raw_confidence) {
            // Convert from mm to inches
            value = std::isfinite(raw_reading) ? raw_reading / 25.4 : -1;
            confidence = raw_confidence;
            
            // Use confidence-based validation instead of hard range limits
            is_valid = (confidence > 0 && value > 0 && std::isfinite(value));
        }
    };
    
    // Update step: weight particles based on sensor measurements
    void update() {
        // Get and process sensor readings with confidence values
        std::vector<SensorReading> sensors = {
            SensorReading(front_dist.get(), front_dist.get_confidence()),
            SensorReading(back_dist.get(), back_dist.get_confidence()),
            SensorReading(left_dist.get(), left_dist.get_confidence()),
            SensorReading(right_dist.get(), right_dist.get_confidence())
        };
        
        // Count sensors with usable information (confidence > 0)
        int usable_sensor_count = 0;
        for (const auto& sensor : sensors) {
            if (sensor.is_valid) usable_sensor_count++;
        }
        
        // Skip update only if NO sensors provide usable information
        if (usable_sensor_count == 0) {
            return;
        }
        
        double total_weight = 0.0;
        
        // Update particle weights based on confidence-weighted sensor log-likelihood
        for (auto& particle : particles) {
            double log_weight_sum = 0.0; // Start with zero for addition in log space
            double total_sensor_weight = 0.0; // Track total weight for normalization
            
            // Calculate log-likelihood for each sensor (confidence-weighted additive approach)
            for (int i = 0; i < 4; i++) {
                if (sensors[i].is_valid) {
                    // Base sensor weight from configuration
                    double base_sensor_weight = (i < SENSOR_WEIGHTS.size()) ? SENSOR_WEIGHTS[i] : 1.0;
                    
                    // Adjust weight based on sensor confidence (0-63 range)
                    // Normalize confidence to [0, 1] and use as multiplier
                    double confidence_factor = std::min(sensors[i].confidence, static_cast<int32_t>(63)) / 63.0;
                    double effective_sensor_weight = base_sensor_weight * confidence_factor;
                    
                    // Calculate expected distance and error
                    double expected = getExpectedDistance(particle, i);
                    double error = sensors[i].value - expected;
                    
                    // Adjust noise model based on confidence
                    // Lower confidence = higher effective noise
                    double confidence_adjusted_noise = BASE_SENSOR_NOISE_STD / (0.1 + 0.9 * confidence_factor);
                    
                    // Gaussian log-likelihood function with confidence-adjusted noise
                    double log_likelihood = -0.5 * (error * error) / (confidence_adjusted_noise * confidence_adjusted_noise);
                    
                    // Weight the log-likelihood by effective sensor weight
                    log_weight_sum += effective_sensor_weight * log_likelihood;
                    total_sensor_weight += effective_sensor_weight;
                }
                // Skip invalid sensors (confidence = 0 or non-finite readings)
            }
            
            // Normalize by total sensor weight and convert back to linear space
            if (total_sensor_weight > 0.0) {
                double avg_log_likelihood = log_weight_sum / total_sensor_weight;
                particle.weight = exp(avg_log_likelihood); // Convert back to linear space [0, 1]
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
            // Include angular uncertainty around current heading estimate
            std::normal_distribution<double> x_dist(estimated_pose.x, 12.0);  // 12 inch standard deviation
            std::normal_distribution<double> y_dist(estimated_pose.y, 12.0);  // 12 inch standard deviation
            double current_heading_rad = estimated_pose.theta * M_PI / 180.0;
            std::normal_distribution<double> theta_dist(current_heading_rad, ANGLE_NOISE_STD * M_PI / 180.0); // Angular uncertainty
            
            for (int m = 0; m < random_count; m++) {
                Particle random_particle;
                random_particle.x = x_dist(gen);
                random_particle.y = y_dist(gen);
                random_particle.theta = theta_dist(gen); // Add angular uncertainty
                random_particle.weight = 1.0 / NUM_PARTICLES;
                
                // Keep particles within field bounds
                clampParticleToField(random_particle);
                
                // Validate particle before adding
                if (std::isfinite(random_particle.x) && std::isfinite(random_particle.y) && 
                    std::isfinite(random_particle.theta) && std::isfinite(random_particle.weight)) {
                    new_particles.push_back(random_particle);
                } else {
                    // Use odometry-centered distribution if guided generation fails
                    std::normal_distribution<double> x_odom(current_pose.x, 12.0);  // 12 inch std dev around odom
                    std::normal_distribution<double> y_odom(current_pose.y, 12.0);
                    double odom_heading_rad = current_pose.theta * M_PI / 180.0;
                    std::normal_distribution<double> theta_odom(odom_heading_rad, ANGLE_NOISE_STD * M_PI / 180.0); // Angular uncertainty
                    
                    random_particle.x = x_odom(gen);
                    random_particle.y = y_odom(gen);
                    random_particle.theta = theta_odom(gen); // Add angular uncertainty
                    
                    // Keep within bounds
                    clampParticleToField(random_particle);
                    
                    random_particle.weight = 1.0 / NUM_PARTICLES;
                    new_particles.push_back(random_particle);
                }
            }
        } else {
            // Use odometry-centered random distribution when MCL pose estimate is invalid
            // Generate particles around odometry pose with angular uncertainty
            std::normal_distribution<double> x_dist(current_pose.x, 15.0);  // 15 inch standard deviation
            std::normal_distribution<double> y_dist(current_pose.y, 15.0);  // 15 inch standard deviation
            double current_heading_rad = current_pose.theta * M_PI / 180.0;
            std::normal_distribution<double> theta_dist(current_heading_rad, ANGLE_NOISE_STD * M_PI / 180.0); // Angular uncertainty
            
            for (int m = 0; m < random_count; m++) {
                Particle random_particle;
                random_particle.x = x_dist(gen);
                random_particle.y = y_dist(gen);
                random_particle.theta = theta_dist(gen); // Add angular uncertainty
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
    
    // Get estimated pose (weighted average of particles for position and orientation)
    lemlib::Pose getEstimatedPose(const lemlib::Pose& current_pose) {
        double x_sum = 0.0, y_sum = 0.0;
        double cos_sum = 0.0, sin_sum = 0.0; // For circular mean of angles
        double weight_sum = 0.0;
        
        for (const auto& particle : particles) {
            x_sum += particle.x * particle.weight;
            y_sum += particle.y * particle.weight;
            cos_sum += cos(particle.theta) * particle.weight;
            sin_sum += sin(particle.theta) * particle.weight;
            weight_sum += particle.weight;
        }
        
        // Calculate estimated heading from particle distribution (circular mean)
        double estimated_theta_rad = atan2(sin_sum, cos_sum);
        double estimated_theta = estimated_theta_rad * 180.0 / M_PI; // Convert to degrees 
        
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
    
    // Get particle spread (measure of uncertainty including angular spread)
    double getParticleSpread() {
        lemlib::Pose mean = getEstimatedPose();
        double mean_theta_rad = mean.theta * M_PI / 180.0;
        double spread = 0.0;
        
        for (const auto& particle : particles) {
            double dx = particle.x - mean.x;
            double dy = particle.y - mean.y;
            
            // Calculate angular difference (wrapped to [-π, π])
            double dtheta = particle.theta - mean_theta_rad;
            while (dtheta > M_PI) dtheta -= 2.0 * M_PI;
            while (dtheta < -M_PI) dtheta += 2.0 * M_PI;
            
            // Combine position and angular spread (scale angular error to inches)
            double position_error = sqrt(dx * dx + dy * dy);
            double angular_error = fabs(dtheta) * 10.0; // Scale: 1 radian ≈ 10 inch equivalent
            
            spread += (position_error + angular_error) * particle.weight;
        }
        
        return spread;
    }
    
    // Check if localization is converged
    bool isConverged(double threshold = 4.0) {
        return getParticleSpread() < threshold;
    }
    
    // Reset MCL (useful for kidnapped robot problem)
    void reset() {
        initializeParticles();
        initialized = false;
    }
};

// Define static constants for balanced distance cache
const double MCLController::AggressiveDistanceCache::ANGLE_STEP = 45.0;
const double MCLController::AggressiveDistanceCache::CELL_SIZE = 4.5;

// Global MCL controller instance
MCLController mcl_controller;

// Main MCL update function to be called periodically
lemlib::Pose updateMCL() {
    extern lemlib::Chassis chassis; // Reference to chassis from drive.cpp
    
    try {
        // Get current odometry pose ONCE per MCL loop (optimization: shared across predict/update/resample steps)
        lemlib::Pose current_pose = chassis.getPose();
        
        // Validate pose to ensure chassis is initialized
        if (!std::isfinite(current_pose.x) || !std::isfinite(current_pose.y) || !std::isfinite(current_pose.theta)) {
            printf("MCL: ERROR - Invalid chassis pose, skipping MCL update\n");
            return lemlib::Pose(0, 0, 0);
        }
        
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
    } catch (...) {
        printf("MCL: EXCEPTION caught in updateMCL() - returning safe pose\n");
        return lemlib::Pose(0, 0, 0);
    }
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
        // updateChassisPoseFromMCL(false, current_pose);
        
        pros::delay(50); // Update at 20Hz
    }
}

// Initialize MCL system
void initializeMCL() {
    // Check available memory before starting MCL
    void* test_alloc = malloc(100000); // Test 100KB allocation
    if (test_alloc) {
        free(test_alloc);
        printf("MCL: Memory check passed - starting MCL task\n");
        pros::Task mcl_task(mclTask, (void*)"MCL");
    } else {
        printf("MCL: ERROR - Insufficient memory! MCL disabled.\n");
    }
}
