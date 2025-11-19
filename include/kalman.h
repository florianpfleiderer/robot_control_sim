// Created on Sat Nov 15 2025 by Florian Pfleiderer

#pragma once

#include <Eigen/Dense>

/**
 * @brief Generic linear Kalman filter for discrete-time systems.
 *
 * The filter models a system of the form:
 *
 *   x_k = A x_{k-1} + B u_{k-1} + w_{k-1},   w_{k-1} ~ N(0, Q)
 *   z_k = H x_k       + v_k,                 v_k     ~ N(0, R)
 *
 * Template parameters allow configuring:
 *  - Scalar     : numeric type (e.g. double, float)
 *  - StateDim   : dimension of the state vector    x_k
 *  - ControlDim : dimension of the control vector  u_k
 *  - MeasDim    : dimension of the measurement     z_k
 *
 * Defaults are chosen for a 1D constant-acceleration model:
 *  - State:   [x, v]^T
 *  - Control: [a]^T
 *  - Meas:    [x_meas]^T
 *
 * @tparam Scalar     Numeric scalar type (default: double)
 * @tparam StateDim   Dimension of state vector (default: 2)
 * @tparam ControlDim Dimension of control input (default: 1)
 * @tparam MeasDim    Dimension of measurement vector (default: 1)
 *
 * @author Florian Pfleiderer
 */
template <typename Scalar = double, int StateDim = 2, int ControlDim = 1,
          int MeasDim = 1>
class KalmanFilter {
  public:
  // --- Public type aliases for clarity and safety ---
  using StateVector       = Eigen::Matrix<Scalar, StateDim, 1>;
  using ControlVector     = Eigen::Matrix<Scalar, ControlDim, 1>;
  using MeasurementVector = Eigen::Matrix<Scalar, MeasDim, 1>;

  using StateMatrix       = Eigen::Matrix<Scalar, StateDim, StateDim>;
  using ControlMatrix     = Eigen::Matrix<Scalar, StateDim, ControlDim>;
  using MeasurementMatrix = Eigen::Matrix<Scalar, MeasDim, StateDim>;

  using ProcessCovariance     = StateMatrix;
  using MeasurementCovariance = Eigen::Matrix<Scalar, MeasDim, MeasDim>;
  using GainMatrix            = Eigen::Matrix<Scalar, StateDim, MeasDim>;
  using InnovationCovariance  = MeasurementCovariance;

  /**
   * @brief Construct a Kalman filter with an initial state and covariance.
   *
   * The dynamic model (A, B), measurement model (H) and noise covariances
   * (Q, R) must be configured via the corresponding setters before
   * calling predict() / update().
   *
   * @param x0 Initial state estimate \f$\hat{x}_{0|0}\f$
   * @param P0 Initial covariance     \f$P_{0|0}\f$
   */
  KalmanFilter(const StateVector &x0, const StateMatrix &P0) : x_(x0), P_(P0) {
    // A defaults to identity; B, H, Q, R default to zero.
    A_.setIdentity();
    B_.setZero();
    H_.setZero();
    Q_.setZero();
    R_.setZero();
  }

  KalmanFilter()                                    = delete;
  KalmanFilter(const KalmanFilter &)                = default;
  KalmanFilter(KalmanFilter &&) noexcept            = default;
  KalmanFilter &operator=(const KalmanFilter &)     = default;
  KalmanFilter &operator=(KalmanFilter &&) noexcept = default;
  ~KalmanFilter()                                   = default;

  /**
   * @brief Set the state transition matrix A.
   *
   * Models the linear evolution of the state:
   *   x_k = A x_{k-1} + B u_{k-1} + w_{k-1}
   *
   * @param A State transition matrix (StateDim x StateDim).
   */
  void setStateTransitionMatrix(const StateMatrix &A) {
    A_ = A;
  }

  /**
   * @brief Set the control input matrix B.
   *
   * Maps control input u_{k-1} into the state space:
   *   x_k = A x_{k-1} + B u_{k-1} + w_{k-1}
   *
   * @param B Control matrix (StateDim x ControlDim).
   */
  void setControlMatrix(const ControlMatrix &B) {
    B_ = B;
  }

  /**
   * @brief Set the measurement matrix H.
   *
   * Models how the state is observed:
   *   z_k = H x_k + v_k
   *
   * @param H Measurement matrix (MeasDim x StateDim).
   */
  void setMeasurementMatrix(const MeasurementMatrix &H) {
    H_ = H;
  }

  /**
   * @brief Set the process noise covariance Q.
   *
   * Models uncertainty in the system dynamics / process model:
   *   w_k ~ N(0, Q)
   *
   * @param Q Process noise covariance (StateDim x StateDim).
   */
  void setProcessNoiseCovariance(const ProcessCovariance &Q) {
    Q_ = Q;
  }

  /**
   * @brief Set the measurement noise covariance R.
   *
   * Models uncertainty in the measurement:
   *   v_k ~ N(0, R)
   *
   * @param R Measurement noise covariance (MeasDim x MeasDim).
   */
  void setMeasurementNoiseCovariance(const MeasurementCovariance &R) {
    R_ = R;
  }

  /**
   * @brief Perform the prediction (time update) step.
   *
   * Propagates the state and covariance using the system model:
   *
   *   x_k|k-1 = A x_{k-1|k-1} + B u_{k-1}
   *   P_k|k-1 = A P_{k-1|k-1} A^T + Q
   *
   * @param u Control input at step k-1 (ControlDim x 1).
   */
  void predict(const ControlVector &u) {
    // State prediction.
    x_ = A_ * x_ + B_ * u;

    // Covariance prediction.
    P_ = A_ * P_ * A_.transpose() + Q_;
  }

  /**
   * @brief Perform the measurement update (correction) step.
   *
   * Incorporates a new measurement into the state estimate:
   *
   *   y_k = z_k - H x_k|k-1               (innovation)
   *   S_k = H P_k|k-1 H^T + R             (innovation covariance)
   *   K_k = P_k|k-1 H^T S_k^{-1}          (Kalman gain)
   *   x_k|k = x_k|k-1 + K_k y_k           (state update)
   *   P_k|k = (I - K_k H) P_k|k-1         (covariance update)
   *
   * @param z Measurement vector at step k (MeasDim x 1).
   */
  void update(const MeasurementVector &z) {
    // Innovation (measurement residual).
    const MeasurementVector y = z - H_ * x_;

    // Innovation covariance.
    const InnovationCovariance S = H_ * P_ * H_.transpose() + R_;

    // Kalman gain.
    const GainMatrix K = P_ * H_.transpose() * S.inverse();

    // State update.
    x_ = x_ + K * y;

    // Covariance update.
    const StateMatrix I = StateMatrix::Identity();
    P_                  = (I - K * H_) * P_;
  }

  /**
   * @brief Get the current state estimate.
   *
   * @return const reference to the current state vector \f$\hat{x}_{k|k}\f$.
   */
  const StateVector &state() const noexcept {
    return x_;
  }

  /**
   * @brief Get the current state covariance.
   *
   * @return const reference to the current covariance matrix \f$P_{k|k}\f$.
   */
  const StateMatrix &covariance() const noexcept {
    return P_;
  }

  private:
  // Current estimate and covariance (posterior after last update).
  StateVector x_; ///< \f$\hat{x}_{k|k}\f$
  StateMatrix P_; ///< \f$P_{k|k}\f$

  // System matrices.
  StateMatrix       A_; ///< State transition matrix.
  ControlMatrix     B_; ///< Control input matrix.
  MeasurementMatrix H_; ///< Measurement matrix.

  // Noise covariances.
  ProcessCovariance     Q_; ///< Process noise covariance.
  MeasurementCovariance R_; ///< Measurement noise covariance.
};

/**
 * @brief Convenient alias for a 1D position+velocity Kalman filter with
 * acceleration input.
 *
 * State:   [x, v]^T
 * Control: [a]^T
 * Meas:    [x_meas]^T
 */
using PositionVelocityKF1D = KalmanFilter<double, 2, 1, 1>;

/**
 * @brief Convenient alias for a 2D position+velocity Kalman filter with
 * acceleration input.
 *
 * State:   [x, v_x, y, v_y]^T
 * Control: [a_x, a_y]^T
 * Meas:    [x_meas, y_meas]^T
 *
 * This is essentially two coupled 1D filters (for x and y) sharing one
 * 4D state and a 2D measurement.
 *
 * @author Florian Pfleiderer
 */
using PositionVelocityKF2D = KalmanFilter<double, 4, 2, 2>;

/**
 * @brief Alias for a state extension with constant offset.
 * 
 * State:   [x, v_x, y, v_y, d_x, d_y]^T
 * Control: [a_x, a_y]^T
 * Meas:    [x_meas, y_meas]^T
 * 
 * This accounts for an unknown constan force in the phsyical environment.
 * 
 * @author Florian Pfleiderer
 */
using PositionVelocityDisturbanceKF2D = KalmanFilter<double, 6, 2, 2>;
