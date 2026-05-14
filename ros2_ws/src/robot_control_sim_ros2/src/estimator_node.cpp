// estimator_node — 6-state position/velocity/disturbance KF.
// Mirrors the configuration in app/main.cpp lines 49-94.

#include "kalman.h"

#include <Eigen/Dense>
#include <geometry_msgs/msg/accel_stamped.hpp>
#include <geometry_msgs/msg/point_stamped.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>
#include <tf2_ros/transform_broadcaster.h>

#include <memory>

using KF2D = PositionVelocityDisturbanceKF2D;

class EstimatorNode : public rclcpp::Node {
  public:
  EstimatorNode() : Node("estimator_node") {
    dt_                 = this->declare_parameter<double>("dt", 0.05);
    const double r_std  = this->declare_parameter<double>("measurement_stddev", 0.05);
    const double q_pv   = this->declare_parameter<double>("process_noise_pv", 1e-4);
    const double q_dist = this->declare_parameter<double>("process_noise_disturbance", 1e-2);

    Eigen::Matrix<double, 6, 1> x0 = Eigen::Matrix<double, 6, 1>::Zero();
    Eigen::Matrix<double, 6, 6> P0 = Eigen::Matrix<double, 6, 6>::Identity() * 1e-2;
    kf_                            = std::make_unique<KF2D>(x0, P0);

    Eigen::Matrix<double, 6, 6> A = Eigen::Matrix<double, 6, 6>::Identity();
    A(0, 1) = dt_;
    A(2, 3) = dt_;
    A(1, 4) = dt_;
    A(3, 5) = dt_;

    Eigen::Matrix<double, 6, 2> B        = Eigen::Matrix<double, 6, 2>::Zero();
    const double                half_dt2 = 0.5 * dt_ * dt_;
    B(0, 0) = half_dt2;
    B(1, 0) = dt_;
    B(2, 1) = half_dt2;
    B(3, 1) = dt_;

    Eigen::Matrix<double, 2, 6> H = Eigen::Matrix<double, 2, 6>::Zero();
    H(0, 0)                       = 1.0;
    H(1, 2)                       = 1.0;

    Eigen::Matrix<double, 6, 6> Q = Eigen::Matrix<double, 6, 6>::Zero();
    Q.diagonal() << q_pv, q_pv, q_pv, q_pv, q_dist, q_dist;

    Eigen::Matrix2d R = Eigen::Matrix2d::Identity() * (r_std * r_std);

    kf_->setStateTransitionMatrix(A);
    kf_->setControlMatrix(B);
    kf_->setMeasurementMatrix(H);
    kf_->setProcessNoiseCovariance(Q);
    kf_->setMeasurementNoiseCovariance(R);

    odom_pub_ = this->create_publisher<nav_msgs::msg::Odometry>("/robot/odom", 10);
    tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);

    cmd_sub_ = this->create_subscription<geometry_msgs::msg::AccelStamped>(
        "/cmd_accel", 10,
        [this](geometry_msgs::msg::AccelStamped::SharedPtr msg) {
          last_u_ << msg->accel.linear.x, msg->accel.linear.y;
        });

    meas_sub_ = this->create_subscription<geometry_msgs::msg::PointStamped>(
        "/robot/measurement", 10,
        [this](geometry_msgs::msg::PointStamped::SharedPtr msg) {
          kf_->predict(last_u_);
          Eigen::Vector2d z;
          z << msg->point.x, msg->point.y;
          kf_->update(z);
          publish(msg->header.stamp);
        });
  }

  private:
  void publish(const rclcpp::Time &stamp) {
    const auto &x = kf_->state();

    nav_msgs::msg::Odometry odom;
    odom.header.stamp            = stamp;
    odom.header.frame_id         = "map";
    odom.child_frame_id          = "base_link";
    odom.pose.pose.position.x    = x(0);
    odom.pose.pose.position.y    = x(2);
    odom.pose.pose.orientation.w = 1.0;
    odom.twist.twist.linear.x    = x(1);
    odom.twist.twist.linear.y    = x(3);
    odom_pub_->publish(odom);

    geometry_msgs::msg::TransformStamped tf;
    tf.header.stamp            = stamp;
    tf.header.frame_id         = "odom";
    tf.child_frame_id          = "base_link";
    tf.transform.translation.x = x(0);
    tf.transform.translation.y = x(2);
    tf.transform.rotation.w    = 1.0;
    tf_broadcaster_->sendTransform(tf);
  }

  double                                         dt_ {0.05};
  Eigen::Vector2d                                last_u_ {Eigen::Vector2d::Zero()};
  std::unique_ptr<KF2D>                          kf_;
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;

  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr             odom_pub_;
  rclcpp::Subscription<geometry_msgs::msg::AccelStamped>::SharedPtr cmd_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PointStamped>::SharedPtr meas_sub_;
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<EstimatorNode>());
  rclcpp::shutdown();
  return 0;
}
