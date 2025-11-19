// Created on Fri Nov 14 2025 by Florian Pfleiderer

#pragma once

#include <utility> // std::pair

/**
 * The robot class knows the position and velocity of the robot.
 * @author Florian Pfleiderer
 */
class Robot {
  public:
  Robot(double x = 0.0, double y = 0.0, double vx = 0.0,
        double vy = 0.0) noexcept;

  /**
   * @brief Updates the Robot position given acceleration commands.
   */
  void update(double ax_cmd, double ay_cmd, double dt) noexcept;

  [[nodiscard]] std::pair<double, double> position() const noexcept;
  [[nodiscard]] std::pair<double, double> velocity() const noexcept;

  private:
  double                  x_ {};
  double                  y_ {};
  double                  vx_ {};
  double                  vy_ {};
  static constexpr double friction_constant {0.98};
  static constexpr double motor_efficiency_ {0.8};
  static constexpr double disturbance_x_ {1.5};
  static constexpr double disturbance_y_ {-0.9};
};
