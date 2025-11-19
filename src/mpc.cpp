// Created on Wed Nov 19 2025 by Florian Pfleiderer

#include "mpc.h"

#include <algorithm>
#include <stdexcept>

namespace {
constexpr double kDefaultPosWeight = 25.0;
constexpr double kDefaultVelWeight = 2.0;
constexpr double kDefaultControlWeight = 0.2;
} // namespace

MPC::MPC(double dt, int horizon) {
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

void MPC::setWeights(const StateMatrix &Q, const ControlWeight &R) {
  Q_ = 0.5 * (Q + Q.transpose());
  R_ = 0.5 * (R + R.transpose());
  rebuildGains();
}

void MPC::setControlLimits(const ControlVector &min_u,
                           const ControlVector &max_u) {
  if ((min_u.array() > max_u.array()).any()) {
    throw std::invalid_argument(
        "MPC::setControlLimits - min must be <= max for each axis");
  }
  u_min_ = min_u;
  u_max_ = max_u;
}

void MPC::setHorizon(int horizon) {
  if (horizon <= 0) {
    throw std::invalid_argument("MPC::setHorizon - horizon must be >= 1");
  }
  horizon_ = horizon;
  rebuildGains();
}

void MPC::setTimeStep(double dt) {
  if (dt <= 0.0) {
    throw std::invalid_argument("MPC::setTimeStep - dt must be positive");
  }
  dt_ = dt;
  updatePredictionModel();
  rebuildGains();
}

MPC::ControlVector MPC::computeControl(const StateVector &current_state,
                                       const StateVector &reference) const {
  if (gains_.empty()) {
    return clampControl(ControlVector::Zero());
  }

  const StateVector error = current_state - reference;
  ControlVector     u     = -gains_.front() * error;
  return clampControl(u);
}

MPC::ControlVector
MPC::computeControlToPosition(const StateVector &current_state,
                              const Eigen::Vector2d &position_target,
                              const Eigen::Vector2d &velocity_target) const {
  StateVector reference = StateVector::Zero();
  reference(0)          = position_target.x();
  reference(2)          = position_target.y();
  reference(1)          = velocity_target.x();
  reference(3)          = velocity_target.y();
  return computeControl(current_state, reference);
}

void MPC::updatePredictionModel() {
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

void MPC::rebuildGains() {
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

MPC::ControlVector MPC::clampControl(const ControlVector &u) const {
  ControlVector limited = u;
  for (int idx = 0; idx < limited.size(); ++idx) {
    limited(idx) =
        std::clamp(limited(idx), u_min_(idx), u_max_(idx));
  }
  return limited;
}
