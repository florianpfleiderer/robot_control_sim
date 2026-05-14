// sensor_node — adds Gaussian noise to /robot/true_pose and publishes /robot/measurement.

#include "sensor.h"

#include <geometry_msgs/msg/point_stamped.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <rclcpp/rclcpp.hpp>

#include <memory>

class SensorNode : public rclcpp::Node {
  public:
  SensorNode() : Node("sensor_node") {
    const double noise = this->declare_parameter<double>("noise_stddev", 0.05);
    sensor_            = std::make_unique<Sensor>(noise);

    pub_ = this->create_publisher<geometry_msgs::msg::PointStamped>(
        "/robot/measurement", 10);

    sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
        "/robot/true_pose", 10,
        [this](geometry_msgs::msg::PoseStamped::SharedPtr msg) {
          const auto [sx, sy] =
              sensor_->read(msg->pose.position.x, msg->pose.position.y);

          geometry_msgs::msg::PointStamped out;
          out.header  = msg->header;
          out.point.x = sx;
          out.point.y = sy;
          pub_->publish(out);
        });
  }

  private:
  std::unique_ptr<Sensor>                                          sensor_;
  rclcpp::Publisher<geometry_msgs::msg::PointStamped>::SharedPtr   pub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr sub_;
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<SensorNode>());
  rclcpp::shutdown();
  return 0;
}
