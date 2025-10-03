#pragma once
#include <string>

const float FRONT_DIST_OFFSET = 5.503;
const float LEFT_DIST_OFFSET = 5.63;
const float RIGHT_DIST_OFFSET = 5.704;

const float MM_TO_IN = 0.0393701;

extern double raycastToWall(double globalX, double globalY, double heading, 
                     double sensorOffsetX, double sensorOffsetY, 
                     const std::string& direction);