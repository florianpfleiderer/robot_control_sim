// controller_node — PID, MPC4D, or MPC6D depending on parameter.

#include "mpc.h"
#include "pid.h"

#include <Eigen/Dense>
#include <geometry_msgs/msg/accel_stamped.hpp>
#include <geometry_msgs/msg/vector3_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>

#include <memory>
#include <stdexcept>
#include <string>

class ControllerNode : public rclcpp::Node {
  public:
  ControllerNode() : Node("controller_node") {
    dt_       = this->declare_parameter<double>("dt", 0.05);
    target_x_ = this->declare_parameter<double>("target_x", 5.0);
    target_y_ = this->declare_parameter<double>("target_y", 5.0);
    type_     = this->declare_parameter<std::string>("controller_type", "mpc6d");

    const int    horizon = this->declare_parameter<int>("mpc_horizon", 20);
    const double q_pos   = this->declare_parameter<double>("mpc_q_position", 25.0);
    const double q_vel   = this->declare_parameter<double>("mpc_q_velocity", 2.0);
    const double q_dist  = this->declare_parameter<double>("mpc_q_disturbance", 1e-2);
    const double r_ctrl  = this->declare_parameter<double>("mpc_r_control", 0.2);

    const double kp = this->declare_parameter<double>("pid_kp", 0.75);
    const double ki = this->declare_parameter<double>("pid_ki", 0.1);
    const double kd = this->declare_parameter<double>("pid_kd", 0.6);

    pub_ = this->create_publisher<geometry_msgs::msg::AccelStamped>(
        "/cmd_accel", 10);

    if (type_ == "pid") {
      pid_ = std::make_unique<PID>(kp, ki, kd);
    } else if (type_ == "mpc4d") {
      mpc4d_ = std::make_unique<MPC4D>(dt_, horizon);
      configure_mpc4d(q_pos, q_vel, r_ctrl);
    } else if (type_ == "mpc6d") {
      mpc6d_ = std::make_unique<MPC6D>(dt_, horizon);
      configure_mpc6d(q_pos, q_vel, q_dist, r_ctrl);
    } else {
      RCLCPP_FATAL(this->get_logger(), "Unknown controller_type '%s'",
                   type_.c_str());
      throw std::runtime_error("invalid controller_type");
    }

    sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
        "/robot/odom", 10,
        [this](nav_msgs::msg::Odometry::SharedPtr msg) { on_odom(*msg); });

    disturbance_sub_ = this->create_subscription<geometry_msgs::msg::Vector3Stamped>(
        "/robot/disturbance", 10,
        [this](geometry_msgs::msg::Vector3Stamped::SharedPtr msg) {
          last_disturbance_ << msg->vector.x, msg->vector.y;
        });
  }

  private:
  void configure_mpc4d(double q_pos, double q_vel, double r_ctrl) {
    Eigen::Matrix4d A = Eigen::Matrix4d::Identity();
    A(0, 1)           = dt_;
    A(2, 3)           = dt_;

    Eigen::Matrix<double, 4, 2> B  = Eigen::Matrix<double, 4, 2>::Zero();
    const double                h2 = 0.5 * dt_ * dt_;
    B(0, 0) = h2;
    B(1, 0) = dt_;
    B(2, 1) = h2;
    B(3, 1) = dt_;
    mpc4d_->setPredictionModel(A, B);

    MPC4D::StateMatrix Q = MPC4D::StateMatrix::Zero();
    Q(0, 0) = q_pos;
    Q(1, 1) = q_vel;
    Q(2, 2) = q_pos;
    Q(3, 3) = q_vel;
    MPC4D::ControlWeight R = MPC4D::ControlWeight::Identity() * r_ctrl;
    mpc4d_->setWeights(Q, R);
  }

  void configure_mpc6d(double q_pos, double q_vel, double q_dist,
                       double r_ctrl) {
    Eigen::Matrix<double, 6, 6> A = Eigen::Matrix<double, 6, 6>::Identity();
    A(0, 1) = dt_;
    A(2, 3) = dt_;
    A(1, 4) = dt_;
    A(3, 5) = dt_;

    Eigen::Matrix<double, 6, 2> B  = Eigen::Matrix<double, 6, 2>::Zero();
    const double                h2 = 0.5 * dt_ * dt_;
    B(0, 0) = h2;
    B(1, 0) = dt_;
    B(2, 1) = h2;
    B(3, 1) = dt_;
    mpc6d_->setPredictionModel(A, B);

    MPC6D::StateMatrix Q = MPC6D::StateMatrix::Zero();
    Q(0, 0) = q_pos;
    Q(1, 1) = q_vel;
    Q(2, 2) = q_pos;
    Q(3, 3) = q_vel;
    Q(4, 4) = q_dist;
    Q(5, 5) = q_dist;
    MPC6D::ControlWeight R = MPC6D::ControlWeight::Identity() * r_ctrl;
    mpc6d_->setWeights(Q, R);
  }

  void on_odom(const nav_msgs::msg::Odometry &msg) {
    double          ax = 0.0;
    double          ay = 0.0;
    Eigen::Vector2d target;
    target << target_x_, target_y_;

    if (type_ == "pid") {
      auto c = pid_->compute(target_x_, target_y_, msg.pose.pose.position.x,
                             msg.pose.pose.position.y, dt_);
      ax = c.x;
      ay = c.y;
    } else if (type_ == "mpc4d") {
      MPC4D::StateVector x;
      x << msg.pose.pose.position.x, msg.twist.twist.linear.x,
          msg.pose.pose.position.y, msg.twist.twist.linear.y;
      auto u = mpc4d_->computeControlToPosition(x, target);
      ax = u(0);
      ay = u(1);
    } else { // mpc6d
      MPC6D::StateVector x;
      x(0) = msg.pose.pose.position.x;
      x(1) = msg.twist.twist.linear.x;
      x(2) = msg.pose.pose.position.y;
      x(3) = msg.twist.twist.linear.y;
      x(4) = last_disturbance_(0);
      x(5) = last_disturbance_(1);
      auto u = mpc6d_->computeControlToPosition(x, target);
      ax = u(0);
      ay = u(1);
    }

    geometry_msgs::msg::AccelStamped out;
    out.header.stamp    = msg.header.stamp;
    out.header.frame_id = "base_link";
    out.accel.linear.x  = ax;
    out.accel.linear.y  = ay;
    pub_->publish(out);
  }

  double      dt_ {0.05};
  double      target_x_ {5.0};
  double      target_y_ {5.0};
  std::string type_ {"mpc6d"};

  std::unique_ptr<PID>   pid_;
  std::unique_ptr<MPC4D> mpc4d_;
  std::unique_ptr<MPC6D> mpc6d_;

  Eigen::Vector2d last_disturbance_ {Eigen::Vector2d::Zero()};

  rclcpp::Publisher<geometry_msgs::msg::AccelStamped>::SharedPtr      pub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr            sub_;
  rclcpp::Subscription<geometry_msgs::msg::Vector3Stamped>::SharedPtr disturbance_sub_;
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ControllerNode>());
  rclcpp::shutdown();
  return 0;
}
