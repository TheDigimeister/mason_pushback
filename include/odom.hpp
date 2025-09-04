#ifndef _ODOM_HPP_
#define _ODOM_HPP_

#include "pros/motor_group.hpp"
#include <cmath>

const float WHEEL_DIAMETER = 3.75;
const float TRACK_WIDTH = 11.5;

const float PI = 3.14159265359;
const float DEGREES_TO_RADIANS = PI / 180.0;
const float RADIANS_TO_DEGREES = 180.0 / PI;
const float TICKS_TO_DISTANCE = 10.0 * DEGREES_TO_RADIANS * WHEEL_DIAMETER / 2.0;

const float MM_TO_INCHES = 0.0393701;

struct Pose {
    float x;
    float y;
    float theta;
};

class OdometryState {
    private:
        Pose current_position;
        int previous_encoder_left;
        int previous_encoder_right;
        int previous_encoder_perp;
        int current_encoder_left;
        int current_encoder_right;
        int current_encoder_perp;
        float global_delta_x;
        float global_delta_y;
        float global_delta_theta;
        
        pros::MotorGroup &left_encoder;
        pros::MotorGroup &right_encoder;
        pros::MotorGroup *perpendicular_encoder; // Changed to pointer
        bool has_perpendicular_encoder;

    public:
        // Constructor with perpendicular encoder
        OdometryState(Pose current_position, 
                     pros::MotorGroup &left_encoder, 
                     pros::MotorGroup &right_encoder, 
                     pros::MotorGroup &perpendicular_encoder);
        
        // Constructor without perpendicular encoder
        OdometryState(Pose current_position, 
                     pros::MotorGroup &left_encoder, 
                     pros::MotorGroup &right_encoder);

      
        void odom_update();
        void odom_init(float starting_x, float starting_y, float starting_theta);
        float get_x_position() const;
        float get_y_position() const;
        float get_heading() const;
        float get_heading_degrees() const;
        float get_global_delta_x() const;
        float get_global_delta_y() const;
        float get_global_delta_theta() const;
        void set_position(float new_x, float new_y, float new_theta);
};

#endif