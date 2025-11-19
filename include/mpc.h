// Created on Wed Nov 19 2025 by Florian Pfleiderer

#pragma once

#include <Eigen/Dense>
#include <vector>
#include <algorithm>
#include <stdexcept>

namespace {
constexpr double kDefaultPosWeight     = 25.0;
constexpr double kDefaultVelWeight     = 2.0;
constexpr double kDefaultControlWeight = 0.2;
} // namespace

/**
 * @brief Lightweight linear MPC controller for the simulator's double
 * integrator model.
 *
 * State vector layout: [p_x, v_x, p_y, v_y]^T
 * Control vector:      [a_x, a_y]^T
 */
template <typename Scalar = double, int StateDim = 4, int ControlDim = 2,
          int MeasDim = 2>
class MPC {
  public:
  using StateVector   = Eigen::Matrix<double, StateDim, 1>;
  using ControlVector = Eigen::Matrix<double, ControlDim, 1>;

  using StateMatrix       = Eigen::Matrix<double, StateDim, StateDim>;
  using ControlMatrix     = Eigen::Matrix<double, StateDim, ControlDim>;
  using ControlWeight     = Eigen::Matrix<double, ControlDim, ControlDim>;
  using ControlGainMatrix = Eigen::Matrix<double, ControlDim, StateDim>;

  /**
   * Construct an MPC controller.
   *
   * @param dt      Discrete sample time.
   * @param horizon Prediction horizon length (>= 1).
   */
  MPC(double dt, int horizon = 10) {
    if (dt <= 0.0) {
      throw std::invalid_argument("MPC::MPC - dt must be positive");
    }
    if (horizon <= 0) {
      throw std::invalid_argument("MPC::MPC - horizon must be >= 1");
    }

    dt_      = dt;
    horizon_ = horizon;

    Q_.setZero();
    Q_(0, 0) = kDefaultPosWeight;
    Q_(1, 1) = kDefaultVelWeight;
    Q_(2, 2) = kDefaultPosWeight;
    Q_(3, 3) = kDefaultVelWeight;

    R_.setIdentity();
    R_ *= kDefaultControlWeight;

    updatePredictionModel();
    rebuildGains();
  }

  /**
   * @brief Configure quadratic cost weights.
   *
   * @param Q State weighting matrix.
   * @param R Control weighting matrix.
   */
  void setWeights(const StateMatrix &Q, const ControlWeight &R) {
    Q_ = 0.5 * (Q + Q.transpose());
    R_ = 0.5 * (R + R.transpose());
    rebuildGains();
  }

  /**
   * @brief Configure hard acceleration limits.
   */
  void setControlLimits(const ControlVector &min_u,
                        const ControlVector &max_u) {
    if ((min_u.array() > max_u.array()).any()) {
      throw std::invalid_argument(
          "MPC::setControlLimits - min must be <= max for each axis");
    }
    u_min_ = min_u;
    u_max_ = max_u;
  }

  /**
   * @brief Change prediction horizon length.
   */
  void setHorizon(int horizon) {
    if (horizon <= 0) {
      throw std::invalid_argument("MPC::setHorizon - horizon must be >= 1");
    }
    horizon_ = horizon;
    rebuildGains();
  }

  /**
   * @brief Change timestep and rebuild model matrices.
   */
  void setTimeStep(double dt) {
    if (dt <= 0.0) {
      throw std::invalid_argument("MPC::setTimeStep - dt must be positive");
    }
    dt_ = dt;
    updatePredictionModel();
    rebuildGains();
  }

  /**
   * @brief Override the prediction model that the controller uses.
   *
   * The default double-integrator model can be replaced with any
   * StateDim×StateDim / StateDim×ControlDim pair that matches the Kalman
   * filter or plant you are controlling.
   */
  void setPredictionModel(const StateMatrix &A, const ControlMatrix &B) {
    A_ = A;
    B_ = B;
    rebuildGains();
  }

  /**
   * @brief Compute control for a reference state.
   */
  [[nodiscard]] ControlVector
  computeControl(const StateVector &current_state,
                 const StateVector &reference) const {
    if (gains_.empty()) {
      return clampControl(ControlVector::Zero());
    }

    const StateVector error = current_state - reference;
    ControlVector     u     = -gains_.front() * error;
    return clampControl(u);
  }

  /**
   * @brief Convenience helper using position targets and optional velocities.
   */
  [[nodiscard]] ControlVector computeControlToPosition(
      const StateVector &current_state, const Eigen::Vector2d &position_target,
      const Eigen::Vector2d &velocity_target = Eigen::Vector2d::Zero()) const {
    StateVector reference = StateVector::Zero();
    reference(0)          = position_target.x();
    reference(2)          = position_target.y();
    reference(1)          = velocity_target.x();
    reference(3)          = velocity_target.y();
    return computeControl(current_state, reference);
  }

  [[nodiscard]] double timeStep() const noexcept {
    return dt_;
  }

  [[nodiscard]] int horizon() const noexcept {
    return horizon_;
  }

  void updatePredictionModel() {
    A_.setIdentity();
    A_(0, 1) = dt_;
    A_(2, 3) = dt_;

    B_.setZero();
    const double half_dt2 = 0.5 * dt_ * dt_;
    B_(0, 0)              = half_dt2;
    B_(1, 0)              = dt_;
    B_(2, 1)              = half_dt2;
    B_(3, 1)              = dt_;
  }

  private:
  void rebuildGains() {
    gains_.assign(static_cast<std::size_t>(horizon_),
                  ControlGainMatrix::Zero());

    StateMatrix P = Q_;

    for (int i = horizon_ - 1; i >= 0; --i) {
      const ControlWeight S =
          R_ + B_.transpose() * P * B_; // Positive definite by construction

      Eigen::LDLT<ControlWeight> solver;
      solver.compute(S);
      const ControlGainMatrix K =
          solver.solve(B_.transpose() * P * A_); // Solves S * K = ...

      gains_[static_cast<std::size_t>(i)] = K;

      P = Q_ + A_.transpose() * P * (A_ - B_ * K);
    }
  }

  [[nodiscard]] ControlVector clampControl(const ControlVector &u) const {
    ControlVector limited = u;
    for (int idx = 0; idx < limited.size(); ++idx) {
      limited(idx) = std::clamp(limited(idx), u_min_(idx), u_max_(idx));
    }
    return limited;
  }

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

using MPC4D = MPC<double, 4, 2, 2>;
using MPC6D = MPC<double, 6, 2, 2>;
