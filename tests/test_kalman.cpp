#include "kalman.h"

#include <Eigen/Dense>
#include <gtest/gtest.h>

namespace {

PositionVelocityKF1D make_constant_velocity_kf(double dt) {
  Eigen::Matrix<double, 2, 1> x0;
  x0 << 0.0, 0.0;
  Eigen::Matrix2d P0 = Eigen::Matrix2d::Identity() * 1.0;

  PositionVelocityKF1D kf {x0, P0};

  Eigen::Matrix2d A;
  A << 1.0, dt,
       0.0, 1.0;
  Eigen::Matrix<double, 2, 1> B;
  B << 0.5 * dt * dt, dt;
  Eigen::Matrix<double, 1, 2> H;
  H << 1.0, 0.0;
  Eigen::Matrix2d Q = Eigen::Matrix2d::Identity() * 1e-4;
  Eigen::Matrix<double, 1, 1> R;
  R << 1e-2;

  kf.setStateTransitionMatrix(A);
  kf.setControlMatrix(B);
  kf.setMeasurementMatrix(H);
  kf.setProcessNoiseCovariance(Q);
  kf.setMeasurementNoiseCovariance(R);
  return kf;
}

}  // namespace

TEST(KalmanFilter, PredictWithZeroControlLeavesZeroStateUnchanged) {
  auto                        kf = make_constant_velocity_kf(0.05);
  Eigen::Matrix<double, 1, 1> u;
  u << 0.0;
  kf.predict(u);
  const auto &x = kf.state();
  EXPECT_DOUBLE_EQ(x(0), 0.0);
  EXPECT_DOUBLE_EQ(x(1), 0.0);
}

TEST(KalmanFilter, ConvergesToConsistentPositionMeasurement) {
  auto                        kf = make_constant_velocity_kf(0.05);
  Eigen::Matrix<double, 1, 1> u;
  u << 0.0;
  Eigen::Matrix<double, 1, 1> z;
  z << 5.0;
  for (int i = 0; i < 200; ++i) {
    kf.predict(u);
    kf.update(z);
  }
  EXPECT_NEAR(kf.state()(0), 5.0, 0.2);
}

TEST(KalmanFilter, CovarianceShrinksAfterRepeatedUpdates) {
  auto                        kf = make_constant_velocity_kf(0.05);
  Eigen::Matrix<double, 1, 1> u;
  u << 0.0;
  Eigen::Matrix<double, 1, 1> z;
  z << 0.0;

  kf.predict(u);
  kf.update(z);
  const double initial_trace = kf.covariance().trace();

  for (int i = 0; i < 50; ++i) {
    kf.predict(u);
    kf.update(z);
  }
  EXPECT_LT(kf.covariance().trace(), initial_trace);
}
