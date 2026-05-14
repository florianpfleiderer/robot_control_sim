// plant_node — integrates the Robot dynamics and publishes ground-truth pose/twist.

#include "robot.h"

#include <geometry_msgs/msg/accel_stamped.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <rclcpp/rclcpp.hpp>

#include <chrono>
#include <memory>

class PlantNode : public rclcpp::Node {
  public:
  PlantNode() : Node("plant_node") {
    dt_ = this->declare_parameter<double>("dt", 0.05);
    const double initial_x = this->declare_parameter<double>("initial_x", 5.0);
    const double initial_y = this->declare_parameter<double>("initial_y", 5.0);
    robot_ = Robot(initial_x, initial_y);

    pose_pub_  = this->create_publisher<geometry_msgs::msg::PoseStamped>(
        "/robot/true_pose", 10);
    twist_pub_ = this->create_publisher<geometry_msgs::msg::TwistStamped>(
        "/robot/true_twist", 10);

    cmd_sub_ = this->create_subscription<geometry_msgs::msg::AccelStamped>(
        "/cmd_accel", 10,
        [this](geometry_msgs::msg::AccelStamped::SharedPtr msg) {
          last_ax_ = msg->accel.linear.x;
          last_ay_ = msg->accel.linear.y;
        });

    const auto period = std::chrono::duration<double>(dt_);
    timer_            = this->create_wall_timer(
        std::chrono::duration_cast<std::chrono::nanoseconds>(period),
        [this]() { step(); });
  }

  private:
  void step() {
    robot_.update(last_ax_, last_ay_, dt_);
    const auto [x, y]   = robot_.position();
    const auto [vx, vy] = robot_.velocity();
    const auto stamp    = this->now();

    geometry_msgs::msg::PoseStamped pose;
    pose.header.stamp       = stamp;
    pose.header.frame_id    = "map";
    pose.pose.position.x    = x;
    pose.pose.position.y    = y;
    pose.pose.orientation.w = 1.0;
    pose_pub_->publish(pose);

    geometry_msgs::msg::TwistStamped twist;
    twist.header.stamp    = stamp;
    twist.header.frame_id = "base_link";
    twist.twist.linear.x  = vx;
    twist.twist.linear.y  = vy;
    twist_pub_->publish(twist);
  }

  Robot  robot_ {};
  double dt_ {0.05};
  double last_ax_ {0.0};
  double last_ay_ {0.0};

  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr     pose_pub_;
  rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr    twist_pub_;
  rclcpp::Subscription<geometry_msgs::msg::AccelStamped>::SharedPtr cmd_sub_;
  rclcpp::TimerBase::SharedPtr                                      timer_;
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<PlantNode>());
  rclcpp::shutdown();
  return 0;
}
