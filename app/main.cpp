// Created on Fri Nov 14 2025 by Florian Pfleiderer

#include "robot.h"
#include "controller.h"
#include "sensor.h"
#include "kalman.h"

#include <Eigen/Dense>
#include <iostream>
#include <fstream>
#include <vector>

int main() {
    Robot robot{};
    const double sensor_noise = 0.05;
    Sensor sensor{sensor_noise};
    Controller controller{0.75, 0.1, 0.6};

    const double target_x = 5.0;
    const double target_y = 5.0;
    const double dt       = 0.05;

    std::vector<double> true_xs, true_ys, sensor_xs, sensor_ys;
    std::vector<double> kf_xs, kf_ys;

    using KF2D = PositionVelocityKF2D;

    // Initialize Kalman filter using the robot's initial state and a small covariance
    const auto [x0_init, y0_init] = robot.position();
    const auto [vx0_init, vy0_init] = robot.velocity();

    Eigen::Vector4d x0vec;
    x0vec << x0_init, vx0_init, y0_init, vy0_init;

    Eigen::Matrix4d P0 = Eigen::Matrix4d::Identity() * 1e-2;

    KF2D kf{x0vec, P0};

    // Configure A, B, H, Q, R using dt
    Eigen::Matrix4d A = Eigen::Matrix4d::Identity();
    A(0,1) = dt;
    A(2,3) = dt;

    Eigen::Matrix<double,4,2> B = Eigen::Matrix<double,4,2>::Zero();
    const double half_dt2 = 0.5 * dt * dt;
    B(0,0) = half_dt2; B(1,0) = dt;
    B(2,1) = half_dt2; B(3,1) = dt;

    Eigen::Matrix<double,2,4> H = Eigen::Matrix<double,2,4>::Zero();
    H(0,0) = 1.0; H(1,2) = 1.0;

    Eigen::Matrix4d Q = Eigen::Matrix4d::Zero();
    Q.diagonal() << 1e-4, 1e-4, 1e-4, 1e-4;

    Eigen::Matrix2d R = Eigen::Matrix2d::Zero();
    const double r_val = sensor_noise * sensor_noise;
    R.diagonal() << r_val, r_val;

    kf.setStateTransitionMatrix(A);
    kf.setControlMatrix(B);
    kf.setMeasurementMatrix(H);
    kf.setProcessNoiseCovariance(Q);
    kf.setMeasurementNoiseCovariance(R);

    for (int step = 0; step < 1000; ++step) {
        auto [true_x, true_y] = robot.position();
        auto [sx, sy] = sensor.read(true_x, true_y);
        auto [ax, ay] = controller.compute(target_x, target_y, sx, sy, dt);

        // Kalman: predict using control (ax,ay) then update with measurement (sx,sy)
        Eigen::Vector2d u;
        u << ax, ay;
        kf.predict(u);

        Eigen::Vector2d z;
        z << sx, sy;
        kf.update(z);

        // Extract filtered position estimate
        const auto& xhat = kf.state();
        const double x_kf = xhat(0);
        const double y_kf = xhat(2);

        // Now apply control to the ground-truth robot
        robot.update(ax, ay, dt);

        true_xs.push_back(true_x);
        true_ys.push_back(true_y);
        sensor_xs.push_back(sx);
        sensor_ys.push_back(sy);
        kf_xs.push_back(x_kf);
        kf_ys.push_back(y_kf);

        if (step % 20 == 0) {
            std::cout << "Step " << step
                      << " true=(" << true_x << ", " << true_y << ")"
                      << " sensor=(" << sx << ", " << sy << ")"
                      << " kf=(" << x_kf << ", " << y_kf << ")\n";
        }
    }

    // Save to CSV file
    std::ofstream file("trajectory_data.csv");
    file << "true_x,true_y,sensor_x,sensor_y,x_kf,y_kf\n";
    for (size_t i = 0; i < true_xs.size(); ++i) {
        file << true_xs[i] << "," << true_ys[i] << ","
             << sensor_xs[i] << "," << sensor_ys[i] << ","
             << kf_xs[i] << "," << kf_ys[i] << "\n";
    }
    file.close();

    return 0;
}
