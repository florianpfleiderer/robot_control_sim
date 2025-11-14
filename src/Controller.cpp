// Created on Fri Nov 14 2025 by Florian Pfleiderer

#include <Controller.hpp>

Controller::Controller(
    double kp,
    double ki,
    double kd
) noexcept : kp_{kp}, ki_{ki}, kd_{kd} {}

[[nodiscard]] Coordinate<double> Controller::compute(
        double target_x,
        double target_y,
        double current_x, 
        double current_y,
        double dt
    ) noexcept {

    // error term
    Coordinate<double> error{target_x-current_x, target_y-current_y};
    
}