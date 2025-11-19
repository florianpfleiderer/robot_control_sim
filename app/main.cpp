// Created on Fri Nov 14 2025 by Florian Pfleiderer

#include "robot.h"
#include "pid.h"
#include "sensor.h"
#include "kalman.h"
#include "mpc.h"

#include <Eigen/Dense>
#include <iostream>
#include <fstream>
#include <vector>

#ifndef USE_MPC_CONTROLLER
#define USE_MPC_CONTROLLER 0
#endif

#ifndef USE_DISTURBANCE_CORRECTION
#define USE_DISTURBANCE_CORRECTION 1
#endif

int main() {
  Robot        robot {};
  const double sensor_noise = 0.05;
  Sensor       sensor {sensor_noise};

  const double    target_x = 5.0;
  const double    target_y = 5.0;
  const double    dt       = 0.05;
  Eigen::Vector2d position_target;
  position_target << target_x, target_y;

  std::vector<double> true_xs, true_ys, sensor_xs, sensor_ys;
  std::vector<double> kf_xs, kf_ys;
  std::vector<double> ctrl_ax, ctrl_ay;

#if USE_MPC_CONTROLLER
  MPC            mpc_controller {dt, 20};
  constexpr auto controller_name = "MPC";
#else
  PID            pid {0.75, 0.1, 0.6};
  constexpr auto controller_name = "PID";
#endif

#if USE_DISTURBANCE_CORRECTION
  using KF2D = PositionVelocityDisturbanceKF2D;

  // Initialise Kalman filter using the robot's initial state and a small
  // covariance
  const auto [x0_init, y0_init]   = robot.position();
  const auto [vx0_init, vy0_init] = robot.velocity();

  Eigen::Matrix<double, 6, 1> x0vec;
  x0vec << x0_init, vx0_init, y0_init, vy0_init, 0, 0;

  Eigen::Matrix<double, 6, 6> P0 = Eigen::Matrix<double, 6, 6>::Identity() * 1e-2;

  KF2D kf {x0vec, P0};

  // Configure A, B, H, Q, R using dt
  Eigen::Matrix<double, 6, 6> A = Eigen::Matrix<double, 6, 6>::Identity();
  A(0, 1)                       = dt;
  A(2, 3)                       = dt;
  A(1, 4)                       = dt; // vx += dx*dt
  A(3, 5)                       = dt; // vy += dy*dt

  Eigen::Matrix<double, 6, 2> B        = Eigen::Matrix<double, 6, 2>::Zero();
  const double                half_dt2 = 0.5 * dt * dt;
  B(0, 0)                              = half_dt2;
  B(1, 0)                              = dt;
  B(2, 1)                              = half_dt2;
  B(3, 1)                              = dt;

  Eigen::Matrix<double, 2, 6> H = Eigen::Matrix<double, 2, 6>::Zero();
  H(0, 0)                       = 1.0;
  H(1, 2)                       = 1.0;

  Eigen::Matrix<double, 6, 6> Q = Eigen::Matrix<double, 6, 6>::Zero();
  Q.diagonal() << 1e-4, 1e-4, 1e-4, 1e-4;
  Q(4, 4) = Q(5, 5) = 1e-2; // let disturbance adapt faster

  Eigen::Matrix2d R     = Eigen::Matrix2d::Zero();
  const double    r_val = sensor_noise * sensor_noise;
  R.diagonal() << r_val, r_val;

  kf.setStateTransitionMatrix(A);
  kf.setControlMatrix(B);
  kf.setMeasurementMatrix(H);
  kf.setProcessNoiseCovariance(Q);
  kf.setMeasurementNoiseCovariance(R);
#else
  using KF2D = PositionVelocityKF2D;

  // Initialise Kalman filter using the robot's initial state and a small
  // covariance
  const auto [x0_init, y0_init]   = robot.position();
  const auto [vx0_init, vy0_init] = robot.velocity();

  Eigen::Vector4d x0vec;
  x0vec << x0_init, vx0_init, y0_init, vy0_init;

  Eigen::Matrix4d P0 = Eigen::Matrix4d::Identity() * 1e-2;

  KF2D kf {x0vec, P0};

  // Configure A, B, H, Q, R using dt
  Eigen::Matrix4d A = Eigen::Matrix4d::Identity();
  A(0, 1)           = dt;
  A(2, 3)           = dt;

  Eigen::Matrix<double, 4, 2> B        = Eigen::Matrix<double, 4, 2>::Zero();
  const double                half_dt2 = 0.5 * dt * dt;
  B(0, 0)                              = half_dt2;
  B(1, 0)                              = dt;
  B(2, 1)                              = half_dt2;
  B(3, 1)                              = dt;

  Eigen::Matrix<double, 2, 4> H = Eigen::Matrix<double, 2, 4>::Zero();
  H(0, 0)                       = 1.0;
  H(1, 2)                       = 1.0;

  Eigen::Matrix4d Q = Eigen::Matrix4d::Zero();
  Q.diagonal() << 1e-4, 1e-4, 1e-4, 1e-4;

  Eigen::Matrix2d R     = Eigen::Matrix2d::Zero();
  const double    r_val = sensor_noise * sensor_noise;
  R.diagonal() << r_val, r_val;

  kf.setStateTransitionMatrix(A);
  kf.setControlMatrix(B);
  kf.setMeasurementMatrix(H);
  kf.setProcessNoiseCovariance(Q);
  kf.setMeasurementNoiseCovariance(R);
#endif

  Eigen::Vector2d last_control = Eigen::Vector2d::Zero();

  for (int step = 0; step < 1000; ++step) {
    auto [true_x, true_y] = robot.position();

    // Predict the Kalman filter forward using the previous control input.
    kf.predict(last_control);

    auto [sx, sy] = sensor.read(true_x, true_y);

    Eigen::Vector2d z;
    z << sx, sy;
    kf.update(z);

    const auto  &xhat = kf.state();
    const double x_kf = xhat(0);
    const double y_kf = xhat(2);

    double ax = 0.0;
    double ay = 0.0;

#if USE_MPC_CONTROLLER
    MPC::StateVector   estimated_state = xhat;
    MPC::ControlVector control_cmd = mpc_controller.computeControlToPosition(
        estimated_state, position_target);
    ax = control_cmd(0);
    ay = control_cmd(1);
#else
    auto pid_control = pid.compute(target_x, target_y, x_kf, y_kf, dt);
    ax               = pid_control.x;
    ay               = pid_control.y;
#endif

    robot.update(ax, ay, dt);
    last_control << ax, ay;

    true_xs.push_back(true_x);
    true_ys.push_back(true_y);
    sensor_xs.push_back(sx);
    sensor_ys.push_back(sy);
    kf_xs.push_back(x_kf);
    kf_ys.push_back(y_kf);
    ctrl_ax.push_back(ax);
    ctrl_ay.push_back(ay);

    if (step % 20 == 0) {
      std::cout << "Step " << step << " true=(" << true_x << ", " << true_y
                << ")"
                << " sensor=(" << sx << ", " << sy << ")"
                << " kf=(" << x_kf << ", " << y_kf << ")\n";
    }
  }

  // Save to CSV file
  std::ofstream file("trajectory_data.csv");
  file
      << "true_x,true_y,sensor_x,sensor_y,x_kf,y_kf,ax_cmd,ay_cmd,controller\n";
  for (size_t i = 0; i < true_xs.size(); ++i) {
    file << true_xs[i] << "," << true_ys[i] << "," << sensor_xs[i] << ","
         << sensor_ys[i] << "," << kf_xs[i] << "," << kf_ys[i] << ","
         << ctrl_ax[i] << "," << ctrl_ay[i] << "," << controller_name << "\n";
  }
  file.close();

  return 0;
}
