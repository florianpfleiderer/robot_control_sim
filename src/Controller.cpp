// Created on Fri Nov 14 2025 by Florian Pfleiderer

#include "controller.h"

Controller::Controller(double kp, double ki, double kd) noexcept
    : kp_ {kp},
      ki_ {ki},
      kd_ {kd} {}

[[nodiscard]] Coordinate<double>
Controller::compute(double target_x, double target_y, double current_x,
                    double current_y, double dt) noexcept {
  // avoid division by zero
  if (dt < 0.001)
    dt = 0.001;

  // error term
  Coordinate<double> error {target_x - current_x, target_y - current_y};
  Coordinate<double> incremental {error * dt};

  if (integral_leak_ > 0.0) {
    integral_error_ *= (1.0 - integral_leak_);
    integral_error_ += incremental;
  } else {
    integral_error_ += incremental;
  }

  integral_error_.x =
      std::clamp(integral_error_.x, -integral_limit_, integral_limit_);
  integral_error_.y =
      std::clamp(integral_error_.y, -integral_limit_, integral_limit_);

  Coordinate<double> derivative {0, 0};
  if (!first_call_) {
    Coordinate<double> raw_derivative = (error - prev_error_) / dt;
    // Low-pass filter: filtered = alpha * raw + (1 - alpha) * previous_filtered
    derivative = raw_derivative * derivative_filter_alpha_ +
                 filtered_derivative_ * (1.0 - derivative_filter_alpha_);
    filtered_derivative_ = derivative;
  } else {
    first_call_ = false;
  }

  prev_error_ = error;

  return error * kp_ + integral_error_ * ki_ + derivative * kd_;
}