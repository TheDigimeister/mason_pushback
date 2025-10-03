#include "main.h"

struct SensorData {
    double offsetX;
    double offsetY;
    std::string direction;
    double measuredDistance;
};

double raycastToWall(double globalX, double globalY, double heading, 
                     double sensorOffsetX, double sensorOffsetY, 
                     const std::string& direction) {
    
    // Convert heading to radians (VRC uses degrees, clockwise from north)
    double headingRad = heading * M_PI / 180.0;
    
    // Transform sensor offset from robot frame to global frame
    double cosHeading = cos(headingRad);
    double sinHeading = sin(headingRad);
    
    // Robot frame to global frame transformation
    double sensorGlobalX = globalX + (sensorOffsetX * cosHeading - sensorOffsetY * sinHeading);
    double sensorGlobalY = globalY + (sensorOffsetX * sinHeading + sensorOffsetY * cosHeading);
    
    // Determine ray direction based on sensor direction
    double rayDirectionX = 0, rayDirectionY = 0;
    
    if (direction == "front") {
        rayDirectionX = sinHeading;   // North is positive Y in global frame
        rayDirectionY = cosHeading;
    } else if (direction == "back") {
        rayDirectionX = -sinHeading;
        rayDirectionY = -cosHeading;
    } else if (direction == "left") {
        rayDirectionX = -cosHeading;  // Left is 90 degrees counterclockwise from front
        rayDirectionY = sinHeading;
    } else if (direction == "right") {
        rayDirectionX = cosHeading;   // Right is 90 degrees clockwise from front
        rayDirectionY = -sinHeading;
    } else {
        return -1; // Invalid direction
    }
    
    // Calculate distances to each wall
    double distanceToWall = INFINITY;
    
    // Distance to positive X wall (x = 72)
    if (rayDirectionX > 0) {
        double t = (72.0 - sensorGlobalX) / rayDirectionX;
        if (t > 0) {
            double intersectY = sensorGlobalY + t * rayDirectionY;
            if (intersectY >= -72.0 && intersectY <= 72.0) {
                distanceToWall = std::min(distanceToWall, t);
            }
        }
    }
    
    // Distance to negative X wall (x = -72)
    if (rayDirectionX < 0) {
        double t = (-72.0 - sensorGlobalX) / rayDirectionX;
        if (t > 0) {
            double intersectY = sensorGlobalY + t * rayDirectionY;
            if (intersectY >= -72.0 && intersectY <= 72.0) {
                distanceToWall = std::min(distanceToWall, t);
            }
        }
    }
    
    // Distance to positive Y wall (y = 72)
    if (rayDirectionY > 0) {
        double t = (72.0 - sensorGlobalY) / rayDirectionY;
        if (t > 0) {
            double intersectX = sensorGlobalX + t * rayDirectionX;
            if (intersectX >= -72.0 && intersectX <= 72.0) {
                distanceToWall = std::min(distanceToWall, t);
            }
        }
    }
    
    // Distance to negative Y wall (y = -72)
    if (rayDirectionY < 0) {
        double t = (-72.0 - sensorGlobalY) / rayDirectionY;
        if (t > 0) {
            double intersectX = sensorGlobalX + t * rayDirectionX;
            if (intersectX >= -72.0 && intersectX <= 72.0) {
                distanceToWall = std::min(distanceToWall, t);
            }
        }
    }
    
    return distanceToWall == INFINITY ? -1 : distanceToWall;
}

double cheap_std_norm(double x) {
    return -0.5 * x * x;
}

// Calculate measured X and Y deviation by averaging sensor errors
std::pair<double, double> calculatePositionDeviation(double currentGlobalX, double currentGlobalY, 
                                                     double heading, 
                                                     const std::vector<SensorData>& sensors) {
    
    double totalXError = 0.0;
    double totalYError = 0.0;
    int validSensorCount = 0;
    
    // Convert heading to radians
    double headingRad = heading * M_PI / 180.0;
    double cosHeading = cos(headingRad);
    double sinHeading = sin(headingRad);
    
    for (const auto& sensor : sensors) {
        // Skip invalid measurements
        if (sensor.measuredDistance < 0) continue;
        
        // Get expected distance to wall
        double expectedDistance = raycastToWall(currentGlobalX, currentGlobalY, heading,
                                               sensor.offsetX, sensor.offsetY, sensor.direction);
        
        if (expectedDistance < 0) continue; // Skip if no wall found
        
        // Calculate error (positive means we're closer to wall than expected)
        double distanceError = expectedDistance - sensor.measuredDistance;
        
        // Transform sensor offset to global frame
        double sensorGlobalX = currentGlobalX + (sensor.offsetX * cosHeading - sensor.offsetY * sinHeading);
        double sensorGlobalY = currentGlobalY + (sensor.offsetX * sinHeading + sensor.offsetY * cosHeading);
        
        // Determine ray direction
        double rayDirectionX = 0, rayDirectionY = 0;
        
        if (sensor.direction == "front") {
            rayDirectionX = sinHeading;
            rayDirectionY = cosHeading;
        } else if (sensor.direction == "back") {
            rayDirectionX = -sinHeading;
            rayDirectionY = -cosHeading;
        } else if (sensor.direction == "left") {
            rayDirectionX = -cosHeading;
            rayDirectionY = sinHeading;
        } else if (sensor.direction == "right") {
            rayDirectionX = cosHeading;
            rayDirectionY = -sinHeading;
        } else {
            continue; // Skip invalid direction
        }
        
        // Project the distance error onto X and Y axes
        // Negative because if we're closer to wall than expected, we need to move away
        double xError = -distanceError * rayDirectionX;
        double yError = -distanceError * rayDirectionY;
        
        totalXError += xError;
        totalYError += yError;
        validSensorCount++;
    }
    
    if (validSensorCount == 0) {
        return {0.0, 0.0}; // No valid sensors
    }
    
    // Return average deviation
    return {totalXError / validSensorCount, totalYError / validSensorCount};
}

// Simplified version for common sensor configurations
std::pair<double, double> calculatePositionDeviationSimple(double currentGlobalX, double currentGlobalY, 
                                                          double heading,
                                                          double frontDistance, double frontOffsetX, double frontOffsetY,
                                                          double backDistance, double backOffsetX, double backOffsetY,
                                                          double leftDistance, double leftOffsetX, double leftOffsetY,
                                                          double rightDistance, double rightOffsetX, double rightOffsetY) {
    
    std::vector<SensorData> sensors;
    
    if (frontDistance >= 0) {
        sensors.push_back({frontOffsetX, frontOffsetY, "front", frontDistance});
    }
    if (backDistance >= 0) {
        sensors.push_back({backOffsetX, backOffsetY, "back", backDistance});
    }
    if (leftDistance >= 0) {
        sensors.push_back({leftOffsetX, leftOffsetY, "left", leftDistance});
    }
    if (rightDistance >= 0) {
        sensors.push_back({rightOffsetX, rightOffsetY, "right", rightDistance});
    }
    
    return calculatePositionDeviation(currentGlobalX, currentGlobalY, heading, sensors);
}