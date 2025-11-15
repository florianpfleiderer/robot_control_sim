// Created on Fri Nov 14 2025 by Florian Pfleiderer

#include "robot.h"

Robot::Robot(
    double x, 
    double y, 
    double vx, 
    double vy
) noexcept : x_{x}, y_{y}, vx_{vx}, vy_{vy} {}

void Robot::update(double ax_cmd, double ay_cmd, double dt) noexcept {
    double ax = ax_cmd * motor_efficiency_ + disturbance_x_;
    double ay = ay_cmd * motor_efficiency_ + disturbance_y_;

    vx_ += ax * dt;
    vy_ += ay * dt;

    vx_ *= friction_constant;
    vy_ *= friction_constant;

    x_ += vx_ * dt;
    y_ += vy_ * dt;
}

std::pair<double, double> Robot::position() const noexcept {
    return {x_, y_};
}

std::pair<double, double> Robot::velocity() const noexcept {
    return {vx_, vy_};
}
