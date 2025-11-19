// Created on Wed Nov 19 2025 by Florian Pfleiderer

#pragma once

#include <Eigen/Dense>

#include <vector>

/**
 * @brief Lightweight linear MPC controller for the simulator's double
 * integrator model.
 *
 * State vector layout: [p_x, v_x, p_y, v_y]^T
 * Control vector:      [a_x, a_y]^T
 */
class MPC {
  public:
  using StateVector   = Eigen::Matrix<double, 4, 1>;
  using ControlVector = Eigen::Matrix<double, 2, 1>;

  using StateMatrix       = Eigen::Matrix4d;
  using ControlMatrix     = Eigen::Matrix<double, 4, 2>;
  using ControlWeight     = Eigen::Matrix2d;
  using ControlGainMatrix = Eigen::Matrix<double, 2, 4>;

  /**
   * Construct an MPC controller.
   *
   * @param dt      Discrete sample time.
   * @param horizon Prediction horizon length (>= 1).
   */
  MPC(double dt, int horizon = 10);

  /**
   * @brief Configure quadratic cost weights.
   *
   * @param Q State weighting matrix.
   * @param R Control weighting matrix.
   */
  void setWeights(const StateMatrix &Q, const ControlWeight &R);

  /**
   * @brief Configure hard acceleration limits.
   */
  void setControlLimits(const ControlVector &min_u,
                        const ControlVector &max_u);

  /**
   * @brief Change prediction horizon length.
   */
  void setHorizon(int horizon);

  /**
   * @brief Change timestep and rebuild model matrices.
   */
  void setTimeStep(double dt);

  /**
   * @brief Compute control for a reference state.
   */
  [[nodiscard]] ControlVector computeControl(const StateVector &current_state,
                                             const StateVector &reference) const;

  /**
   * @brief Convenience helper using position targets and optional velocities.
   */
  [[nodiscard]] ControlVector
  computeControlToPosition(const StateVector &current_state,
                           const Eigen::Vector2d &position_target,
                           const Eigen::Vector2d &velocity_target =
                               Eigen::Vector2d::Zero()) const;

  [[nodiscard]] double timeStep() const noexcept {
    return dt_;
  }

  [[nodiscard]] int horizon() const noexcept {
    return horizon_;
  }

  private:
  void updatePredictionModel();
  void rebuildGains();
  [[nodiscard]] ControlVector clampControl(const ControlVector &u) const;

  double        dt_ {0.0};
  int           horizon_ {1};
  StateMatrix   Q_ {StateMatrix::Identity()};
  ControlWeight R_ {ControlWeight::Identity()};

  StateMatrix   A_ {StateMatrix::Identity()};
  ControlMatrix B_ {ControlMatrix::Zero()};

  ControlVector u_min_ {ControlVector::Constant(-2.0)};
  ControlVector u_max_ {ControlVector::Constant(2.0)};

  std::vector<ControlGainMatrix> gains_;
};
