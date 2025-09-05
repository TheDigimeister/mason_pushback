#ifndef _CONTROL_HPP_
#define _CONTROL_HPP_

// #include "robot.hpp"
#include "lemlib/api.hpp"
// #include "pros/distance.hpp"
// #include "pros/rtos.hpp"
// #include <vector>
#include <random>
// #include <cmath>
// #include <algorithm>

// MCL (Monte Carlo Localization) function declarations

/**
 * Initialize the MCL system
 * Call this once during initialization
 */
void initializeMCL();

/**
 * Update the MCL system manually
 * This is automatically called by the MCL task, but can be called manually if needed
 */
void updateMCL();

/**
 * Get the current estimated pose from MCL
 * @return The estimated robot pose (x, y in inches, theta in degrees)
 */
lemlib::Pose getMCLPose();

/**
 * Calculate the pose error in global coordinates
 * @param reference_pose The reference pose to compare against
 * @return The error between estimated and reference pose in global coordinates
 */
lemlib::Pose getMCLError(const lemlib::Pose& reference_pose);

/**
 * Check if the MCL has converged (low uncertainty)
 * @return true if the localization has converged, false otherwise
 */
bool isMCLConverged();

/**
 * Get the current localization uncertainty
 * @return The particle spread as a measure of uncertainty
 */
double getMCLUncertainty();

/**
 * Reset the MCL system (reinitialize particles)
 * Useful for handling the "kidnapped robot" problem
 */
void resetMCL();

/**
 * Update the chassis pose with the MCL estimated pose
 * Only updates if MCL has converged and estimate is reliable
 * @param force_update If true, updates regardless of convergence status
 */
void updateChassisPoseFromMCL(bool force_update = false);

#endif
