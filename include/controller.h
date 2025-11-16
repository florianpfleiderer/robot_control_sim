// Created on Fri Nov 14 2025 by Florian Pfleiderer

#pragma once

#include "coordinate.h"
#include <algorithm>

class Controller {
  public:
  Controller(double kp = 1.0, double ki = 0.0, double kd = 0.0) noexcept;

  /**
   * @brief Computes next acceleration commands based on input.
   */
  [[nodiscard]] Coordinate<double> compute(double target_x, double target_y,
                                           double current_x, double current_y,
                                           double dt) noexcept;

  void reset() noexcept;

  private:
  double                  kp_ {};
  double                  ki_ {};
  double                  kd_ {};
  static constexpr double integral_limit_ {10.0};
  static constexpr double integral_leak_ {0.0};
  Coordinate<double>      integral_error_ {};
  Coordinate<double>      prev_error_ {};
  Coordinate<double>      filtered_derivative_ {0, 0};
  static constexpr double derivative_filter_alpha_ {0.2};
  bool                    first_call_ {true};
};