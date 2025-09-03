#include "main.h"

// Constructor with perpendicular encoder
OdometryState::OdometryState(Pose current_position, 
                           pros::MotorGroup &left_encoder, 
                           pros::MotorGroup &right_encoder, 
                           pros::MotorGroup &perpendicular_encoder)
    : current_position(current_position),
      left_encoder(left_encoder),
      right_encoder(right_encoder),
      perpendicular_encoder(&perpendicular_encoder),
      has_perpendicular_encoder(true) {}

// Constructor without perpendicular encoder
OdometryState::OdometryState(Pose current_position, 
                           pros::MotorGroup &left_encoder, 
                           pros::MotorGroup &right_encoder)
    : current_position(current_position),
      left_encoder(left_encoder),
      right_encoder(right_encoder),
      perpendicular_encoder(nullptr),
      has_perpendicular_encoder(false) {}

void OdometryState::odom_update() {
    current_encoder_left = left_encoder.get_position();
    current_encoder_right = right_encoder.get_position();
    
    if (has_perpendicular_encoder) {
        current_encoder_perp = perpendicular_encoder->get_position();
    }

    int delta_left = current_encoder_left - this->previous_encoder_left;
    int delta_right = current_encoder_right - this->previous_encoder_right;
    int delta_perp = has_perpendicular_encoder ? 
                     (current_encoder_perp - this->previous_encoder_perp) : 0;

    float distance_left = TICKS_TO_DISTANCE * delta_left;
    float distance_right = TICKS_TO_DISTANCE * delta_right;
    float distance_perp = TICKS_TO_DISTANCE * delta_perp;

    float delta_theta = (distance_right - distance_left) / TRACK_WIDTH;

    // X movement: perpendicular encoder if available, otherwise 0
    float local_x = has_perpendicular_encoder ? distance_perp : 0;
    float local_y = (distance_left + distance_right) / 2.0;

    float global_delta_x = 0;
    float global_delta_y = 0;

    if (fabs(delta_theta) < 0.001) {
        global_delta_x = local_x * cos(this->current_position.theta) - local_y * sin(this->current_position.theta);
        global_delta_y = local_x * sin(this->current_position.theta) + local_y * cos(this->current_position.theta);
    } else {
        float radius_y = local_y / delta_theta;
        float radius_x = local_x / delta_theta;

        float sin_delta = sin(delta_theta);
        float cos_delta = cos(delta_theta);

        global_delta_x = radius_y * sin_delta + radius_x * (cos_delta - 1);
        global_delta_y = radius_y * (1 - cos_delta) + radius_x * sin_delta;

        float temp_x = global_delta_x;

        global_delta_x = temp_x * cos(this->current_position.theta) - global_delta_y * sin(this->current_position.theta);
        global_delta_y = temp_x * sin(this->current_position.theta) + global_delta_y * cos(this->current_position.theta);
    }

    this->current_position.x += global_delta_x;
    this->current_position.y += global_delta_y;
    this->current_position.theta += delta_theta;

    // Normalize angle
    while (this->current_position.theta > PI) {
        this->current_position.theta -= 2 * PI;
    }
    while (this->current_position.theta < -PI) {
        this->current_position.theta += 2 * PI;
    }

    // Update previous encoder values
    this->previous_encoder_left = current_encoder_left;
    this->previous_encoder_right = current_encoder_right;
    if (has_perpendicular_encoder) {
        this->previous_encoder_perp = current_encoder_perp;
    }
}

void OdometryState::odom_init(float starting_x, float starting_y, float starting_theta) {
    this->current_position.x = starting_x;
    this->current_position.y = starting_y;
    this->current_position.theta = starting_theta;

    this->previous_encoder_left = left_encoder.get_position();
    this->previous_encoder_right = right_encoder.get_position();
    if (has_perpendicular_encoder) {
        this->previous_encoder_perp = perpendicular_encoder->get_position();
    }
}


float OdometryState::get_x_position() const {
    return this->current_position.x;
}

float OdometryState::get_y_position() const {
    return this->current_position.y;
}

float OdometryState::get_heading() const {
    return this->current_position.theta;
}

float OdometryState::get_heading_degrees() const {
    return this->current_position.theta * RADIANS_TO_DEGREES;
}

void OdometryState::set_position(float new_x, float new_y, float new_theta) {
    this->current_position.x = new_x;
    this->current_position.y = new_y;
    this->current_position.theta = new_theta;
}